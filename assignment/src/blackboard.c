
/* ======================================================================================
 * SECTION 1: INCLUDES AND GLOBALS
 * Standard headers and global state variables for the game world.
 * ====================================================================================== */
#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h> 
#include <sys/file.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "app_blackboard.h"
#include "app_common.h"
#include "process_pid.h"
#include "log.h"

#define BUFSZ 256
#define M_PI 3.14159265358979323846

// --- STATE MONITORING STRUCTS (ADDED) ---
typedef enum {
    STATE_INIT,
    STATE_IDLE,                 // Waiting in select()
    STATE_PROCESSING_INPUT,     // Handling Keyboard Input
    STATE_UPDATING_MAP,         // Updating Obstacles/Targets
    STATE_RENDERING,            // Drawing to Ncurses
    STATE_BROADCASTING          // Sending data via Pipes
} BBProcessState;

typedef enum {
    // Stati Server
    SV_SEND_DRONE,      // 0: Invia coords drone
    SV_WAIT_DOK,        // 1: Attende ACK "dok" (polling)
    SV_SEND_REQ_OBST,   // 2: Invia richiesta "obst"
    SV_WAIT_OBST_DATA,  // 3: Attende dati ostacolo (polling)
    
    // Stati Client
    CL_WAIT_COMMAND,    // 0: Attende comando iniziale (polling)
    CL_WAIT_DRONE_DATA, // 1: Attende dati drone server (polling)
    CL_SEND_OBST_DATA,  // 2: Invia propri dati
    CL_WAIT_POK         // 3: Attende ACK "pok" (polling)
} NetState;

static NetState net_state; // Stato corrente

typedef struct {
    volatile BBProcessState current_state;
    time_t last_state_change;
} BBMonitor;

// Global Monitor Instance
static BBMonitor bb_monitor = {STATE_INIT, 0};
static float alpha = 0.0f;

// Helper to update state
void set_state(BBProcessState new_state) {
    bb_monitor.current_state = new_state;
    bb_monitor.last_state_change = time(NULL);
}

// Globals
static int current_mode = MODE_STANDALONE;
static int net_fd = -1; // Descrittore socket (sia per client che server dopo accept)

// ----------------------------------------

// Timers for periodic events (Obstacle movement)
static struct timespec last_obst_change = {0, 0};
#define OBSTACLE_PERIOD_SEC 5

// UI Optimization (Cache)
static char last_status[256] = "";

// Dynamic Game Entities
static float current_x = 1.0f, current_y = 1.0f; // Drone Locale
static float remote_x = -1.0f, remote_y = -1.0f; // Drone Remoto
static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;
static int target_reached = 0;

// System handles
static WINDOW *status_win = NULL;
static pid_t watchdog_pid = -1;

#define BB_LOG_STATE(msg) \
    logMessage(LOG_PATH, "[BB][%s] %s", state_to_str(bb_monitor.current_state), msg)

    // --- PROTOTIPI DI FUNZIONE ---
void reposition_and_redraw(WINDOW **win_ptr, int req_h, int req_w);

/* ======================================================================================
 * SECTION: COORDINATE CONVERSION & PROTOCOL
 * ====================================================================================== */

// Imposta QUALSIASI file descriptor (socket o pipe) in non-blocking
void set_non_blocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

void send_msg(int fd, const char *fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    if (len > 0 && buffer[len-1] != '\n') {
        strcat(buffer, "\n");
    }
    // In non-blocking write potrebbe fallire se buffer pieno, 
    // ma per messaggi piccoli assumiamo successo per semplicità.
    write(fd, buffer, strlen(buffer));
}

// *** FUNZIONE CHIAVE ***
// Legge dal socket NON BLOCCANTE accumulando dati finché non trova '\n'
// Ritorna: 1 se ha una riga completa, 0 se deve riprovare dopo, -1 errore
int recv_line_nonblock(int fd, char *out, int out_sz) {
    static char rx_buf[BUFSZ];
    static int rx_len = 0;

    // 1. Prova a leggere
    int n = read(fd, rx_buf + rx_len, BUFSZ - rx_len - 1);
    
    if (n > 0) {
        rx_len += n;
        rx_buf[rx_len] = '\0';
    } 
    else if (n < 0) {
        // Se errno è EAGAIN o EWOULDBLOCK, significa "non c'è nulla ora, riprova dopo"
        if (errno != EAGAIN && errno != EWOULDBLOCK) return -1; // Errore vero
    } 
    else if (n == 0) {
        return -1; // Socket chiuso
    }

    // 2. Controlla se abbiamo un newline nel buffer accumulato
    char *newline = strchr(rx_buf, '\n');
    if (newline) {
        *newline = '\0'; // Termina stringa
        strncpy(out, rx_buf, out_sz - 1); // Copia riga in out
        
        // 3. Shift del buffer (sposta i dati avanzati all'inizio)
        int line_len = (newline - rx_buf) + 1;
        int remaining = rx_len - line_len;
        if (remaining > 0) memmove(rx_buf, newline + 1, remaining);
        rx_len = remaining;
        rx_buf[rx_len] = '\0';

        return 1; // ABBIAMO UNA RIGA!
    }

    return 0; // NON ABBIAMO ANCORA UNA RIGA INTERA
}

// Versione bloccante solo per handshake iniziale
int recv_line_block(int fd, char *buffer, int size) {
    int i = 0; char c; memset(buffer, 0, size);
    while (i < size - 1) {
        if (read(fd, &c, 1) <= 0) return 0;
        if (c == '\n') break;
        buffer[i++] = c;
    }
    buffer[i] = '\0';
    return 1;
}

/* ======================================================================================
 * CORREZIONE: NESSUNA INVERSIONE DELL'ASSE Y
 * ====================================================================================== */

void local_to_virt(float lx, float ly, float *vx, float *vy) { 
    // Input: lx, ly (Locali Ncurses: 0,0 in alto a sx)
    float x = lx;
    float y = ly; 
    
    // Applica rotazione SOLO se necessaria (se alpha != 0)
    if(alpha != 0.0f) {
        float cos_a = cosf(alpha);
        float sin_a = sinf(alpha);
        
        float temp_x = x * cos_a - y * sin_a;
        float temp_y = x * sin_a + y * cos_a;
        
        x = temp_x;
        y = temp_y;
    }
    
    *vx = x;
    *vy = y;
}

void virt_to_local(float vx, float vy, float *lx, float *ly) { 
    // Input: vx, vy (Virtuali)
    float x = vx;
    float y = vy;
    
    // Applica rotazione inversa SOLO se necessaria
    if(alpha != 0.0f) {
        float cos_a = cosf(-alpha);
        float sin_a = sinf(-alpha);
        
        float temp_x = x * cos_a - y * sin_a;
        float temp_y = x * sin_a + y * cos_a;
        
        x = temp_x;
        y = temp_y;
    }
    
    *lx = x;
    *ly = y;
}

/* ======================================================================================
 * SECTION 3: CONNECTION & HANDSHAKE
 * ====================================================================================== */

int init_server() {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {0}; int opt=1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    a.sin_family=AF_INET; a.sin_addr.s_addr=INADDR_ANY; a.sin_port=htons(port_number);
    bind(s,(struct sockaddr*)&a,sizeof(a)); listen(s,1);
    mvprintw(LINES/2, COLS/2-10, "WAITING CLIENT..."); refresh();
    return accept(s,NULL,NULL);
}

int init_client() {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family=AF_INET; a.sin_port=htons(port_number);
    inet_pton(AF_INET, server_address, &a.sin_addr);
    while(connect(s,(struct sockaddr*)&a,sizeof(a))<0) {
        mvprintw(LINES/2, COLS/2-15, "CONNECTING..."); refresh(); sleep(1);
    }
    return s;
}

// NOTA: win è WINDOW** (doppio puntatore)
void protocol_handshake(int mode, int fd, int *w, int *h, WINDOW** win) {
    char buf[256];
    
    if (mode == MODE_SERVER) {
        logMessage(LOG_PATH_SC, "[HANDSHAKE][SERVER] Starting handshake. Sending 'ok'...");
        send_msg(fd, "ok");

        // Attesa risposta "ook"
        if (!recv_line_block(fd, buf, 256)) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][SERVER][ERROR] Client disconnected or read error.");
            exit(1);
        }
        // MODIFICA: Controllo stringa secca, senza spazi o dati extra
        if (strcmp(buf, "ook") != 0) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][SERVER][ERROR] Invalid response: expected 'ook', got '%s'", buf);
            exit(1);
        }
        
        logMessage(LOG_PATH_SC, "[HANDSHAKE][SERVER] Received 'ook'. Sending map size: %d x %d", *w, *h);

        // Il server invia le PROPRIE dimensioni
        send_msg(fd, "size %d %d", *w, *h);
        
        // Attesa conferma dimensioni "sok"
        if (!recv_line_block(fd, buf, 256)) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][SERVER][ERROR] Failed to receive size ack.");
            exit(1);
        }

        // MODIFICA: Rimosso sscanf di rw/rh. Ci aspettiamo SOLO "sok"
        int cw, ch;
        if (sscanf(buf, "sok %d %d", &cw, &ch) == 2) {
            logMessage(LOG_PATH_SC,
                "[HANDSHAKE][SERVER] Client confirmed size: %d x %d",
                cw, ch);
        } else {
            logMessage(LOG_PATH_SC,
                "[HANDSHAKE][SERVER][ERROR] Malformed sok: '%s'", buf);
            exit(1);
        }
    
    } else { // CLIENT
        logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT] Waiting for server 'ok'...");

        if (!recv_line_block(fd, buf, 256)) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT][ERROR] Server disconnected or read error.");
            exit(1);
        }
        if (strcmp(buf, "ok") != 0) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT][ERROR] Invalid handshake init: expected 'ok', got '%s'", buf);
            exit(1);
        }
        
        logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT] Received 'ok'. Sending response 'ook'...");
        // MODIFICA: Invio ack pulito
        send_msg(fd, "ook");

        logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT] Waiting for map size...");
        if (!recv_line_block(fd, buf, 256)) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT][ERROR] Failed to receive map size.");
            exit(1);
        }
        
        // 1. Riceve dimensioni dal Server
        if (sscanf(buf, "size %d %d", w, h) != 2) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT][ERROR] Malformed size packet: '%s'", buf);
            exit(1);
        }
        
        logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT] Received map size: %d x %d. Sending confirmation...", *w, *h);

        // 2. Conferma ricezione
        // MODIFICA: Invio SOLO "sok", senza parametri (risolve anche il bug dei puntatori w,h nella send_msg)
        send_msg(fd, "sok %d %d", *w, *h);
        
        // 3. APPLICA LE DIMENSIONI IMMEDIATAMENTE
        logMessage(LOG_PATH_SC, "[HANDSHAKE][CLIENT] Applying window resize and finishing handshake.");
        
        reposition_and_redraw(win, *h, *w);
    }
}

/* ======================================================================================
 * SECTION 2: WATCHDOG INTEGRATION
 * Functions to register PID and respond to health checks (heartbeats).
 * ====================================================================================== */

void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", BB_PID_TAG, getpid());
    logMessage(LOG_PATH, "[BB] PID published securely");
}

void wait_for_watchdog_pid() {
    FILE *fp;
    char line[256], tag[128];
    int pid_temp;
    bool wd_found = false;

    logMessage(LOG_PATH, "[BB] Waiting for Watchdog...");

    while (!wd_found) {
        fp = fopen(PID_FILE_PATH, "r");
        if (fp) {
            while (fgets(line, sizeof(line), fp)) {
                if (sscanf(line, "%s %d", tag, &pid_temp) == 2) {
                    if (strcmp(tag, WD_PID_TAG) == 0) {
                        watchdog_pid = (pid_t)pid_temp;
                        wd_found = true;
                        break;
                    }
                }
            }
            fclose(fp);
        }
        if (!wd_found) usleep(200000);
    }
    logMessage(LOG_PATH, "[BB] Watchdog found (PID %d)", watchdog_pid);
}

// Signal Handler: Responds to Watchdog PING
void watchdog_ping_handler(int sig) {
    (void)sig; 
    if (watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

/* ======================================================================================
 * SECTION 3: NCURSES WINDOW MANAGEMENT
 * Helpers to create, destroy, and manage terminal windows.
 * ====================================================================================== */

WINDOW* create_window(int height, int width, int starty, int startx) {
    WINDOW *win = newwin(height, width, starty, startx);
    keypad(win, TRUE);
    box(win, 0, 0);
    wnoutrefresh(win);
    return win;
}

void destroy_window(WINDOW *win) {
    if (!win) return;
    werase(win);
    wnoutrefresh(win);
    doupdate();
    delwin(win);
}

/* ======================================================================================
 * SECTION 4: RENDERING ENGINE
 * Draws the Map, Drone, Obstacles, and Targets.
 * ====================================================================================== */

void draw_background(WINDOW *win) {
    werase(win);
    box(win, 0, 0);
}

void draw_obstacles(WINDOW *win) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    for (int i = 0; i < num_obstacles; i++) {
        int ox = obstacles[i].x;
        int oy = obstacles[i].y;
        if (ox > 0 && ox < max_x - 1 && oy > 0 && oy < max_y - 1) {
            wattron(win, COLOR_PAIR(2));
            mvwprintw(win, oy, ox, "O"); 
            wattroff(win, COLOR_PAIR(2));
        }
    }
}

void draw_targets(WINDOW *win) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    for (int i = 0; i < num_targets; i++) {
        int tx = targets[i].x;
        int ty = targets[i].y;
        if (tx > 0 && tx < max_x - 1 && ty > 0 && ty < max_y - 1) {
            wattron(win, COLOR_PAIR(3));
            mvwprintw(win, ty, tx, "%d", i + target_reached);
            wattroff(win, COLOR_PAIR(3));
        }
    }
}

void draw_drone(WINDOW *win, float x, float y) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    int ix = (int)x;
    int iy = (int)y;

    // Safety clamping
    if (ix >= max_x - 1) ix = max_x - 2;
    if (iy >= max_y - 1) iy = max_y - 2;
    if (ix < 1) ix = 1;
    if (iy < 1) iy = 1;

    wattron(win, COLOR_PAIR(1));
    mvwprintw(win, iy, ix, "+");
    wattroff(win, COLOR_PAIR(1));
}

// Master refresh function
void redraw_scene(WINDOW *win) {
    set_state(STATE_RENDERING); // Update State
    draw_background(win);
    if(current_mode == MODE_STANDALONE){

        draw_targets(win);
    }
    draw_obstacles(win);
    draw_drone(win, current_x, current_y);

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();
}

/* ======================================================================================
 * SECTION 5: GAME LOGIC HELPERS
 * Logic for collision checks and object generation.
 * ====================================================================================== */

int overlaps_target(int x, int y) {
    for (int i = 0; i < num_targets; i++) {
        if (targets[i].x == x && targets[i].y == y) return 1;
    }
    return 0;
}

// Logic to move a random obstacle (called periodically)
void generate_new_obstacle(int idx, int width, int height) {
    int valid;
    do {
        valid = 1;
        obstacles[idx].x = rand() % (width - 2) + 1;
        obstacles[idx].y = rand() % (height - 2) + 1;

        // Check vs other obstacles
        for (int i = 0; i < num_obstacles; i++) {
            if (i != idx && obstacles[idx].x == obstacles[i].x && obstacles[idx].y == obstacles[i].y) {
                valid = 0; break;
            }
        }
        // Check vs targets
        if (valid && overlaps_target(obstacles[idx].x, obstacles[idx].y)) {
            valid = 0;
        }
    } while (!valid);
}

void generate_new_target(int idx, int width, int height) {
    int valid;
    do {
        valid = 1;
        targets[idx].x = rand() % (width - 2) + 1;
        targets[idx].y = rand() % (height - 2) + 1;

        // 1. Controllo sovrapposizione con Ostacoli
        for (int i = 0; i < num_obstacles; i++) {
            // Nota: Corretto anche qui per confrontare target vs ostacolo
            if (targets[idx].x == obstacles[i].x && targets[idx].y == obstacles[i].y) {
                valid = 0; break;
            }
        }

        // 2. Controllo sovrapposizione con ALTRI Target (FIX QUI)
        if (valid) {
            for (int k = 0; k < num_targets; k++) {
                // IMPORTANTE: k != idx assicura che non controlli se stesso
                if (k != idx && targets[k].x == targets[idx].x && targets[k].y == targets[idx].y) {
                    valid = 0; 
                    break;
                }
            }
        }
        
    } while (!valid);

    logMessage(LOG_PATH, "[BB] New target %d position: %d %d", idx, targets[idx].x, targets[idx].y);
}

/* ======================================================================================
 * SECTION 6: STATUS BAR AND RESIZE HANDLING
 * Updates the top info bar and handles window resizing events.
 * ====================================================================================== */

void update_dynamic(float x, float y, float drn_Fx, float drn_Fy, float obst_Fx, float obst_Fy, float wall_Fx, float wall_Fy, float targ_Fx, float targ_Fy)
{
    if (!status_win) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "x=%.4f y=%.4f | drn(%.4f %.4f) | obst(%.4f %.4f) | wall(%.4f %.4f) | targ(%.4f %.4f)",
        x, y, drn_Fx, drn_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, targ_Fx, targ_Fy
    );

    if (strcmp(buffer, last_status) != 0) {
        strcpy(last_status, buffer);
        werase(status_win);
        mvwprintw(status_win, 0, 0, "%s", buffer);
        wnoutrefresh(status_win);
        doupdate();
    }
}

// Aggiunti argomenti req_h, req_w
void reposition_and_redraw(WINDOW **win_ptr, int req_h, int req_w) {
    
    // Se non sono richieste dimensioni specifiche (0), usa quelle del terminale attuale
    if (req_h == 0 || req_w == 0) {
        if (is_term_resized(LINES, COLS)) {
            resize_term(0, 0);
        }
        req_w = COLS;
        req_h = LINES - 1;
    } else {
        // Se il server ci ha dato dimensioni, forziamo la struttura interna di ncurses
        // +1 perché req_h è l'area di gioco, ma ci serve 1 riga per la status bar
        resize_term(req_h + 1, req_w);
    }

    int startx = 0;
    int starty = 1;

    if (*win_ptr != NULL) {
        // Proviamo a ridimensionare
        if (wresize(*win_ptr, req_h, req_w) == ERR || mvwin(*win_ptr, starty, startx) == ERR) {
            destroy_window(*win_ptr);
            *win_ptr = create_window(req_h, req_w, starty, startx);
            
            // Ridimensiona anche la status bar
            wresize(status_win, 1, req_w);
            mvwin(status_win, 0, 0);
        }
    } else {
        *win_ptr = create_window(req_h, req_w, starty, startx);
        status_win = newwin(1, req_w, 0, 0);
    }

    werase(status_win);
    box(*win_ptr, 0, 0);
    redraw_scene(*win_ptr);
    logMessage(LOG_PATH, "[BB] Window Resized to: %dx%d", req_w, req_h);
}

/* ======================================================================================
 * SECTION 7: IPC (BROADCASTING)
 * Helper functions to send window dimensions to other processes.
 * ====================================================================================== */

void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ) {
    set_state(STATE_BROADCASTING); // Update State
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
    if(current_mode == MODE_STANDALONE){
        write(fd_obst,  &msg, sizeof(msg));
        write(fd_targ,  &msg, sizeof(msg));
    }
    
}

void send_resize(WINDOW *win, int fd_drone) {
    set_state(STATE_BROADCASTING); // Update State
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);
    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);
    write(fd_drone, &msg, sizeof(msg));
}

/* ======================================================================================
 * SECTION 8: MAIN EXECUTION
 * Setup, Watchdog Synchronization, I/O Multiplexing Loop.
 * ====================================================================================== */

int main(int argc, char *argv[]) {
    // A. Parse Arguments (Pipe File Descriptors)
    if (argc < 12) {
        fprintf(stderr, "[BB] Error: Needed 7 file descriptors, received %d\n", argc-1);
        return 1;
    }

    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[2]);
    int fd_drone_write = atoi(argv[3]);
    int fd_obst_write  = atoi(argv[4]);
    int fd_obst_read   = atoi(argv[5]);
    int fd_targ_write  = atoi(argv[6]);
    int fd_targ_read   = atoi(argv[7]);
    int fd_wd_write    = atoi(argv[8]);
    current_mode = atoi(argv[9]);
    if (argv[10]) {
        strncpy(server_address, argv[10], sizeof(server_address) - 1);
        server_address[sizeof(server_address) - 1] = '\0'; // Ensure null-termination
    }
    port_number = atoi(argv[11]);


    signal(SIGPIPE, SIG_IGN);


    if (current_mode != MODE_STANDALONE) {
        num_obstacles = 0;
        num_targets = 0;
        watchdog_pid = -1;
    }


    // B. Setup Watchdog Signal Handler
    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    // Setup WD Signal (Solo in MODE_STANDALONE)
    if (current_mode == MODE_STANDALONE) {
        signal(SIGUSR1, watchdog_ping_handler);
        wait_for_watchdog_pid();
    }

    // Pubblicazione PID
    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        // Se fallisce qui, assicurati che la cartella /tmp o logs esista, ma qui non usiamo logMessage per evitare ricorsioni se il log fallisce
        perror("[BB] Error opening PID file");
        exit(1);
    }

    logMessage(LOG_PATH_SC, "ip address: %s, port number: %d", server_address, port_number);

    int fd_pid = fileno(fp_pid);
    flock(fd_pid, LOCK_EX); // Acquire Lock
    publish_my_pid(fp_pid);
    fflush(fp_pid);
    flock(fd_pid, LOCK_UN); // Release Lock
    fclose(fp_pid);

    // D. Initialize Ncurses
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    curs_set(0);
    start_color();
    use_default_colors();
    init_pair(1, COLOR_BLUE, -1);
    init_pair(2, COLOR_RED, -1);
    init_pair(3, COLOR_GREEN, -1);
    refresh();

    // --- NETWORK HANDSHAKE (PROTOCOL V2) ---
    WINDOW *win = NULL;
    int win_h = LINES - 1, win_w = COLS;
    
    // Inizializzazione Socket differenziata
    if (current_mode != MODE_STANDALONE) {
        net_fd = (current_mode == MODE_SERVER) ? init_server() : init_client();
        if (net_fd < 0) { endwin(); return 1; }
        
        protocol_handshake(current_mode, net_fd, &win_w, &win_h, &win);
        if (current_mode == MODE_CLIENT) resizeterm(win_h + 1, win_w);
        
        // *** FONDAMENTALE: SOCKET NON-BLOCKING DOPO HANDSHAKE ***
        set_non_blocking(net_fd);
        net_state = (current_mode == MODE_SERVER) ? SV_SEND_DRONE : CL_WAIT_COMMAND;
    }

    // *** FONDAMENTALE: PIPE NON-BLOCKING ***
    // Altrimenti read(fd_input) bloccherebbe il loop se non si preme nulla
    set_non_blocking(fd_input_read);
    set_non_blocking(fd_drone_read);

    // E. Initial Window Creation & IPC
    status_win = newwin(1, COLS, 0, 0);
    win = create_window(LINES - 1, COLS, 1, 0);
    
    reposition_and_redraw(&win, 0, 0);
    send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write);


    // Variables for multiplexing and logic
    float drn_Fx = 0.0f, drn_Fy = 0.0f;
    float obst_Fx = 0.0f, obst_Fy = 0.0f;
    float wall_Fx = 0.0f, wall_Fy = 0.0f;
    float targ_Fx = 0.0f, targ_Fy = 0.0f;

    logMessage(LOG_PATH, "[BB] Ready and GUI started");

    fd_set readfds;
    struct timeval tv;
    Message msg;

    obstacles = malloc(sizeof(Point)); 
    num_obstacles = 0;

    char net_buf[256];
    float vx, vy, rx, ry; // Virtual coords
    
    // --- MAIN LOOP ---
    while (1) {
        set_state(STATE_IDLE); // Mark as Idle before select()

        // 1. Handle UI Input (Exit / Resize)
        int ch = getch();
        if (ch != ERR) {
            set_state(STATE_PROCESSING_INPUT);
            if (ch == 'q') break;
            if (ch == KEY_RESIZE) {
                reposition_and_redraw(&win, 0, 0);
                send_resize(win, fd_drone_write);
            }
        }

        // 2. Periodic Obstacle Logic
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (current_mode == MODE_STANDALONE && num_obstacles > 0 && now.tv_sec - last_obst_change.tv_sec >= OBSTACLE_PERIOD_SEC) {
            set_state(STATE_UPDATING_MAP); // Update state
            last_obst_change = now;
            
            int idx = rand() % num_obstacles;
            int max_y, max_x;
            getmaxyx(win, max_y, max_x);
            generate_new_obstacle(idx, max_x, max_y);
            
            redraw_scene(win); // Internally sets STATE_RENDERING

            // Broadcast update
            set_state(STATE_BROADCASTING); // Update state
            Message m;
            m.type = MSG_TYPE_OBSTACLES;
            snprintf(m.data, sizeof(m.data), "%d", num_obstacles);
            write(fd_drone_write, &m, sizeof(m));
            write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
        }

        // 3. I/O Multiplexing (Select)
        FD_ZERO(&readfds);
        FD_SET(fd_input_read, &readfds);
        FD_SET(fd_drone_read, &readfds);
        if(current_mode == MODE_STANDALONE){
            FD_SET(fd_obst_read, &readfds);
            FD_SET(fd_targ_read, &readfds); 
        }
        
        int max_fd = fd_input_read;
        if (fd_drone_read > max_fd) max_fd = fd_drone_read;
        if(current_mode == MODE_STANDALONE){
            if (fd_obst_read > max_fd) max_fd = fd_obst_read;
            if (fd_targ_read > max_fd) max_fd = fd_targ_read;
        }
        
        max_fd += 1;

        tv.tv_sec  = 0;
        tv.tv_usec = 30000; // 50ms Timeout

        int ret = select(max_fd, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // 4. Handle Input Process Data
        if (FD_ISSET(fd_input_read, &readfds)) {
            set_state(STATE_PROCESSING_INPUT); // Update State
            char buf[80];
            ssize_t n = read(fd_input_read, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                if (buf[0] == 'q'){
                    write(fd_wd_write, buf, sizeof(buf));
                    break;
                }
                logMessage(LOG_PATH_SC, "[BB] Input received: %c", buf[0]);
                // Forward key to Drone
                msg.type = MSG_TYPE_INPUT;
                snprintf(msg.data, sizeof(msg.data), "%s", buf);
                write(fd_drone_write, &msg, sizeof(Message));
                
            }
        }

        // 5. Handle Drone Process Data
        if (FD_ISSET(fd_drone_read, &readfds)) {
            set_state(STATE_UPDATING_MAP); // Generic map/physics update state
            if (read(fd_drone_read, &msg, sizeof(msg)) > 0) {
                switch (msg.type) {

                case MSG_TYPE_POSITION:{
                    sscanf(msg.data, "%f %f", &current_x, &current_y);

                    redraw_scene(win); // Sets STATE_RENDERING
                    
                    // Collision Logic: Drone vs Targets
                    int dx = (int)current_x;
                    int dy = (int)current_y;

                    //logMessage(LOG_PATH_SC, "[BB] Drone position: x = %f, y = %f", current_x, current_y);

                    if(current_mode == MODE_STANDALONE){
                        for (int i = 0; i < num_targets; i++) {
                        if (dx == (int)targets[i].x && dy == (int)targets[i].y) {
                            // Remove target
                            if(i == 0){
                                logMessage(LOG_PATH, "[BB] Expected target reached");
                                for (int j = i; j < num_targets - 1; j++) targets[j] = targets[j + 1];
                                target_reached++;
                                num_targets--;
                                
                                // Notify Drone
                                set_state(STATE_BROADCASTING);
                                Message out_msg;
                                out_msg.type = MSG_TYPE_TARGETS;
                                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                                write(fd_drone_write, &out_msg, sizeof(out_msg));
                                write(fd_drone_write, targets, sizeof(Point) * num_targets);
                            }
                            else if(i != 0){

                                logMessage(LOG_PATH, "[BB] Not expected target reached");
                                targets[i].x = 0;
                                targets[i].y = 0;

                                int max_y, max_x;
                                getmaxyx(win, max_y, max_x);
                                generate_new_target(i, max_x, max_y);

                                set_state(STATE_BROADCASTING);
                                Message out_msg;
                                out_msg.type = MSG_TYPE_TARGETS;
                                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                                write(fd_drone_write, &out_msg, sizeof(out_msg));
                                write(fd_drone_write, targets, sizeof(Point) * num_targets);
                            }
                            

                            // Win Condition: Trigger Respawn
                            if (num_targets == 0) {
                                logMessage(LOG_PATH, "[BB] ALL TARGETS CLEARED");
                                Message out_msg;
                                out_msg.type = MSG_TYPE_OBSTACLES;
                                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obstacles);
                                write(fd_targ_write, &out_msg, sizeof(out_msg));
                                write(fd_targ_write, obstacles, sizeof(Point) * num_obstacles);
                            }
                            redraw_scene(win);
                            break; 
                        }
                    }
                    }
                    
                    break;
                }

                case MSG_TYPE_FORCE: {
                    if (sscanf(msg.data, "%f %f %f %f %f %f %f %f",
                            &drn_Fx, &drn_Fy, &obst_Fx, &obst_Fy, &wall_Fx, &wall_Fy, &targ_Fx, &targ_Fy) == 8) {
                        update_dynamic(current_x, current_y, drn_Fx, drn_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, targ_Fx, targ_Fy);
                    }
                    break;
                }
                    default: break;
                }
            }
        }

        // 6. Handle Obstacle Process Data
        if (FD_ISSET(fd_obst_read, &readfds)) {
            set_state(STATE_UPDATING_MAP);
            if (read(fd_obst_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_OBSTACLES) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(obstacles);
                    obstacles = malloc(sizeof(Point) * count);
                    read(fd_obst_read, obstacles, sizeof(Point) * count);
                    num_obstacles = count;
                    
                    logMessage(LOG_PATH, "[BB] received %d obstacles", num_obstacles);
                    
                       // Distribute obstacles to Drone & Target
                    set_state(STATE_BROADCASTING);
                    Message out_msg;
                    out_msg.type = MSG_TYPE_OBSTACLES;
                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obstacles);      
                    
                    write(fd_drone_write, &out_msg, sizeof(out_msg));
                    write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
                    
                    write(fd_targ_write, &out_msg, sizeof(out_msg));
                    write(fd_targ_write, obstacles, sizeof(Point) * num_obstacles);
                }
                redraw_scene(win);
            }
        }

        // 7. Handle Target Process Data
        if (FD_ISSET(fd_targ_read, &readfds)) {
            set_state(STATE_UPDATING_MAP);
            if (read(fd_targ_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_TARGETS) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(targets);
                    targets = malloc(sizeof(Point) * count);
                    read(fd_targ_read, targets, sizeof(Point) * count);
                    num_targets = count;

                    // Distribute targets to Drone & Obstacle
                    set_state(STATE_BROADCASTING);
                    Message out_msg;
                    out_msg.type = MSG_TYPE_TARGETS;
                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                    
                    write(fd_drone_write, &out_msg, sizeof(out_msg));
                    write(fd_drone_write, targets, sizeof(Point) * num_targets);
                    
                    write(fd_obst_write, &out_msg, sizeof(out_msg));
                    write(fd_obst_write, targets, sizeof(Point) * num_targets);
                }
                redraw_scene(win);
            }
        }

        if (current_mode != MODE_STANDALONE && net_fd >= 0) {
            
            // --- SERVER LOGIC ---
            if (current_mode == MODE_SERVER) {
                switch (net_state) {
                    case SV_SEND_DRONE:
                        // 1. Invio coordinate drone
                        // NOTA: Rimossa win_h dalla chiamata (non richiesta dalla firma della funzione)
                        local_to_virt(current_x, current_y, &vx, &vy);
                        
                        // Invio header e dati
                        send_msg(net_fd, "drone"); 
                        send_msg(net_fd, "%f %f", vx, vy);
                        
                        // Transizione immediata all'attesa dell'ACK
                        net_state = SV_WAIT_DOK;
                        break;

                    case SV_WAIT_DOK:
                        // 2. Attesa ACK "dok"
                        if (recv_line_nonblock(net_fd, net_buf, sizeof(net_buf)) == 1) {
                            
                            // Controllo rigoroso per "dok"
                            float dx, dy;
                            if (sscanf(net_buf, "dok %f %f", &dx, &dy) == 2) {
                                logMessage(LOG_PATH_SC, "[SERVER] dok ricevuto (%f %f)", dx, dy);
                                net_state = SV_SEND_REQ_OBST;
                            }
                            // Gestione uscita
                            else if (strcmp(net_buf, "q") == 0) {
                                logMessage(LOG_PATH_SC, "[SERVER] Segnale di uscita ricevuto.");
                                goto exit_loop;
                            }
                        }
                        break;

                    case SV_SEND_REQ_OBST:
                        // 3. Invio richiesta ostacoli
                        send_msg(net_fd, "obst");
                        net_state = SV_WAIT_OBST_DATA;
                        break;

                    case SV_WAIT_OBST_DATA:
                        // 4. Attesa dati ostacolo e invio ACK finale "pok"
                        if (recv_line_nonblock(net_fd, net_buf, sizeof(net_buf)) == 1) {
                            if (sscanf(net_buf, "%f %f", &rx, &ry) == 2) {
                                
                                // Aggiornamento logica locale con dati ricevuti
                                virt_to_local(rx, ry, &remote_x, &remote_y);
                                num_obstacles = 1; 
                                obstacles[0].x = (int)remote_x; 
                                obstacles[0].y = (int)remote_y;
                                
                                // Notifica ai processi interni (UI/Fisica)
                                msg.type = MSG_TYPE_OBSTACLES; 
                                sprintf(msg.data, "1");
                                write(fd_drone_write, &msg, sizeof(msg));
                                write(fd_drone_write, obstacles, sizeof(Point));
                                redraw_scene(win);

                                // *** PUNTO CHIAVE RICHIESTO ***
                                // Invio SOLO "pok" e resetto il ciclo immediatamente
                                logMessage(LOG_PATH_SC, "[SERVER] Dati ricevuti. Invio 'pok'.");
                                send_msg(net_fd, "pok %f %f", rx, ry);
                                
                                net_state = SV_SEND_DRONE; // Ricomincia il ciclo da capo
                            } 
                        }
                        break;
                    default: break;
                }
            } 
            
            // --- CLIENT LOGIC ---
            else if (current_mode == MODE_CLIENT) {
                switch (net_state) {
                    case CL_WAIT_COMMAND:
                        // 1. Attesa comandi dal server ("drone", "obst", "q")
                        if (recv_line_nonblock(net_fd, net_buf, sizeof(net_buf)) == 1) {
                            
                            if (strcmp(net_buf, "drone") == 0) {
                                net_state = CL_WAIT_DRONE_DATA;
                            }
                            else if (strcmp(net_buf, "obst") == 0) {
                                net_state = CL_SEND_OBST_DATA;
                            }
                            else if (strcmp(net_buf, "q") == 0) { 
                                // Uscita pulita: Mando qok e chiudo, nient'altro.
                                logMessage(LOG_PATH_SC, "[CLIENT] Quit ricevuto. Invio 'qok' ed esco.");
                                send_msg(net_fd, "qok"); 
                                goto exit_loop; 
                            }
                        }
                        break;

                    case CL_WAIT_DRONE_DATA:
                        // 2. Ricezione dati drone server -> Invio "dok"
                        if (recv_line_nonblock(net_fd, net_buf, sizeof(net_buf)) == 1) {
                            if (sscanf(net_buf, "%f %f", &rx, &ry) == 2) {
                                
                                // Aggiornamento locale
                                virt_to_local(rx, ry, &remote_x, &remote_y);
                                num_obstacles = 1; 
                                obstacles[0].x = (int)remote_x; 
                                obstacles[0].y = (int)remote_y;
                                
                                msg.type = MSG_TYPE_OBSTACLES; 
                                sprintf(msg.data, "1");
                                write(fd_drone_write, &msg, sizeof(msg));
                                write(fd_drone_write, obstacles, sizeof(Point));
                                redraw_scene(win);

                                // *** PUNTO CHIAVE RICHIESTO ***
                                // Invio SOLO "dok" e torno in attesa
                                logMessage(LOG_PATH_SC, "[CLIENT] Dati drone ricevuti. Invio 'dok'.");
                                send_msg(net_fd, "dok %f %f", rx, ry);
                                
                                net_state = CL_WAIT_COMMAND;
                            }
                        }
                        break;

                    case CL_SEND_OBST_DATA:
                        // 3. Invio propri dati (ostacolo per l'altro)
                        local_to_virt(current_x, current_y, &vx, &vy);
                        logMessage(LOG_PATH_SC, "[CLIENT] Invio posizione locale: %f, %f", vx, vy);
                        
                        send_msg(net_fd, "%f %f", vx, vy);
                        net_state = CL_WAIT_POK;
                        break;

                    case CL_WAIT_POK:
                        // 4. Attesa conferma "pok"
                        if (recv_line_nonblock(net_fd, net_buf, sizeof(net_buf)) == 1) {
                            float ox, oy;
                            if (sscanf(net_buf, "pok %f %f", &ox, &oy) == 2) {
                                logMessage(LOG_PATH_SC, "[CLIENT] pok ricevuto (%f %f)", ox, oy);
                                net_state = CL_WAIT_COMMAND;
                            }
                        }
                        break;
                    default: break;
                }
            }
        }

        // 4. THROTTLING
        // Senza select, questo while girerebbe milioni di volte al secondo (CPU 100%).
        // Mettiamo un piccolo sleep per dare respiro alla CPU (es. 10ms = ~100fps)
        usleep(10000);

    }

    exit_loop:
    if (net_fd >= 0) close(net_fd);
    destroy_window(win);
    free(obstacles);
    endwin();
    return 0;
}