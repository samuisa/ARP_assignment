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

// --- STATE MONITORING STRUCTS (ADDED) ---
typedef enum {
    STATE_INIT,
    STATE_IDLE,                 // Waiting in select()
    STATE_PROCESSING_INPUT,     // Handling Keyboard Input
    STATE_UPDATING_MAP,         // Updating Obstacles/Targets
    STATE_RENDERING,            // Drawing to Ncurses
    STATE_BROADCASTING          // Sending data via Pipes
} BBProcessState;

typedef struct {
    volatile BBProcessState current_state;
    time_t last_state_change;
} BBMonitor;

// Global Monitor Instance
static BBMonitor bb_monitor = {STATE_INIT, 0};

// Helper to update state
void set_state(BBProcessState new_state) {
    bb_monitor.current_state = new_state;
    bb_monitor.last_state_change = time(NULL);
}

// Globals
static int current_mode = MODE_STANDALONE;
static int sock_fd = -1; // Network socket
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


static const char *state_to_str(BBProcessState s) {
    switch (s) {
        case STATE_INIT: return "INIT";
        case STATE_IDLE: return "IDLE";
        case STATE_PROCESSING_INPUT: return "PROCESSING_INPUT";
        case STATE_UPDATING_MAP: return "UPDATING_MAP";
        case STATE_RENDERING: return "RENDERING";
        case STATE_BROADCASTING: return "BROADCASTING";
        default: return "UNKNOWN";
    }
}

#define BB_LOG_STATE(msg) \
    logMessage(LOG_PATH, "[BB][%s] %s", state_to_str(bb_monitor.current_state), msg)


/* ======================================================================================
 * SECTION: COORDINATE CONVERSION & PROTOCOL
 * ====================================================================================== */

// Convert Ncurses (0,0 Top-Left) -> Virtual (0,0 Bottom-Left)
void coords_local_to_virt(float lx, float ly, int win_h, float *vx, float *vy) {
    *vx = lx;
    *vy = (float)win_h - ly;
}

// Convert Virtual (0,0 Bottom-Left) -> Ncurses (0,0 Top-Left)
void coords_virt_to_local(float vx, float vy, int win_h, float *lx, float *ly) {
    *lx = vx;
    *ly = (float)win_h - vy;
}

// Send Message with ACK waiting
void net_send_pos(int fd, float x, float y, int win_h) {
    if (fd < 0) return;
    
    float vx, vy;
    coords_local_to_virt(x, y, win_h, &vx, &vy); // Convert to virtual

    char buf[64];
    snprintf(buf, sizeof(buf), "%.2f %.2f", vx, vy);
    
    // Write
    if (write(fd, buf, strlen(buf)) < 0) return;

    // Wait ACK (Blocking for simplicity as per requirements)
    char ack[2];
    read(fd, ack, 1); 
}

// Receive Message and send ACK
int net_recv_pos(int fd, float *rx, float *ry, int win_h) {
    if (fd < 0) return 0;
    
    char buf[64];
    memset(buf, 0, sizeof(buf));
    
    ssize_t n = read(fd, buf, sizeof(buf)-1);
    if (n <= 0) return 0; // Disconnected or error
    
    // Send ACK immediately
    write(fd, ACK_MSG, ACK_LEN);

    float vx, vy;
    if (sscanf(buf, "%f %f", &vx, &vy) == 2) {
        coords_virt_to_local(vx, vy, win_h, rx, ry); // Convert back to local
        return 1;
    }
    return 0;
}

/* ======================================================================================
 * SECTION: NETWORK SETUP
 * ====================================================================================== */
int init_server() {
    int s_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    int opt = 1;
    setsockopt(s_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_number);

    bind(s_fd, (struct sockaddr*)&addr, sizeof(addr));
    listen(s_fd, 1);
    
    logMessage(LOG_PATH_SC, "[SERVER] Waiting for client...");
    mvprintw(LINES/2, COLS/2 - 10, "WAITING FOR CLIENT..."); refresh();
    
    int c_fd = accept(s_fd, NULL, NULL);
    logMessage(LOG_PATH_SC, "[SERVER] Client connected.");
    return c_fd;
}

int init_client() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_number);

    // Usa IP inserito dall'utente
    if (inet_pton(AF_INET, server_address, &addr.sin_addr) <= 0) {
        logMessage(LOG_PATH_SC, "[CLIENT] Invalid IP address: %s", server_address);
        return -1;
    }

    logMessage(LOG_PATH_SC, "[CLIENT] Connecting to %s:%d ...",
               server_address, port_number);

    while (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        mvprintw(LINES/2, COLS/2 - 15, "CONNECTING TO %s...", server_address);
        refresh();
        sleep(1);
    }

    logMessage(LOG_PATH_SC, "[CLIENT] Connected.");
    return fd;
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

    // Draw Remote Entity (If Active)
    if (remote_x >= 0 && remote_y >= 0) {
        wattron(win, COLOR_PAIR(2)); // Red
        mvwprintw(win, (int)remote_y, (int)remote_x, current_mode == MODE_SERVER ? "C" : "S");
        wattroff(win, COLOR_PAIR(2));
    }

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();
}

// Master refresh function
void redraw_scene(WINDOW *win) {
    set_state(STATE_RENDERING); // Update State
    draw_background(win);
    if(current_mode == MODE_STANDALONE){

        draw_obstacles(win);
        draw_targets(win);
    }
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

void reposition_and_redraw(WINDOW **win_ptr) {
    if (is_term_resized(LINES, COLS)) {
        resize_term(0, 0);
    }
    int new_width  = COLS;
    int new_height = LINES - 1;
    int startx = 0;
    int starty = 1;

    if (*win_ptr != NULL) {
        if (wresize(*win_ptr, new_height, new_width) == ERR || mvwin(*win_ptr, starty, startx) == ERR) {
            destroy_window(*win_ptr);
            *win_ptr = create_window(new_height, new_width, starty, startx);
            wresize(status_win, 1, new_width);
            mvwin(status_win, 0, 0);
        }
    } else {
        *win_ptr = create_window(new_height, new_width, starty, startx);
        status_win = newwin(1, new_width, 0, 0);
    }

    werase(status_win);
    box(*win_ptr, 0, 0);
    redraw_scene(*win_ptr);
    logMessage(LOG_PATH, "[BB] Resized: %dx%d", new_width, new_height);
}

/* ======================================================================================
 * SECTION 7: IPC (BROADCASTING)
 * Helper functions to send window dimensions to other processes.
 * ====================================================================================== */

 ssize_t send_msg(int fd, const char *msg) {
    ssize_t n = write(fd, msg, strlen(msg));
    if (n < 0) {
        perror("write");
        exit(EXIT_FAILURE);
    }
    logMessage(LOG_PATH, "[BB] Sent: '%s'", msg);
    return n;
}

ssize_t recv_msg(int fd, char *buf, size_t bufsz) {
    bzero(buf, bufsz);
    ssize_t n = read(fd, buf, bufsz - 1);
    if (n < 0) {
        perror("read");
        exit(EXIT_FAILURE);
    }
    buf[n] = '\0';
    logMessage(LOG_PATH, "[BB] Received: '%s'", buf);
    return n;
}

void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ) {
    set_state(STATE_BROADCASTING); // Update State
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
    write(fd_obst,  &msg, sizeof(msg));
    write(fd_targ,  &msg, sizeof(msg));
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

// --- PROTOCOL HELPER FUNCTIONS ---

void send_str(int fd, const char *str) {
    // Inviamo buffer a dimensione fissa per semplicità e robustezza nel protocollo "string messages"
    char buf[64]; 
    memset(buf, 0, sizeof(buf));
    strncpy(buf, str, sizeof(buf)-1);
    if (write(fd, buf, sizeof(buf)) < 0) perror("Net write error");
}

int recv_str(int fd, char *buf) {
    memset(buf, 0, 64);
    // Leggiamo la dimensione fissa concordata
    if (read(fd, buf, 64) <= 0) return 0; // Connection closed or error
    return 1;
}

// Implementazione Handshake Iniziale (Client e Server)
void run_network_handshake(int mode, int fd, int *w, int *h) {
    char buf[64];
    char dim_str[64];

    if (mode == MODE_SERVER) {
        // --- SERVER SIDE ---
        // 1. Send OK, Recv OOK
        send_str(fd, "ok");
        recv_str(fd, buf); 
        if (strcmp(buf, "ook") != 0) logMessage(LOG_PATH, "[NET] Handshake Error: expected ook");

        // 2. Send Size, Recv SOK
        snprintf(dim_str, sizeof(dim_str), "%d %d", *w, *h);
        send_str(fd, dim_str);
        recv_str(fd, buf); // Expect "sok <size>"
        // (Opzionale: parsing di buf per verificare che contenga "sok")
        
    } else {
        // --- CLIENT SIDE ---
        // 1. Recv OK, Send OOK
        recv_str(fd, buf);
        if (strcmp(buf, "ok") != 0) logMessage(LOG_PATH, "[NET] Handshake Error: expected ok");
        send_str(fd, "ook");

        // 2. Recv Size, Send SOK
        recv_str(fd, dim_str);
        sscanf(dim_str, "%d %d", w, h);
        
        char reply[128];
        snprintf(reply, sizeof(reply), "sok %s", dim_str);
        send_str(fd, reply);
    }
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

    logMessage(LOG_PATH, "ip address: %s, port number: %d", server_address, port_number);

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
    
    // Inizializzazione Socket
    if (current_mode == MODE_SERVER) {
        sock_fd = init_server(); // Assumiamo ritorni un socket valido o esca
    } else if (current_mode == MODE_CLIENT) {
        sock_fd = init_client(); // Assumiamo ritorni un socket valido o esca
    }

    if (sock_fd >= 0) {
        // Esegue il protocollo: ok/ook -> size/sok
        run_network_handshake(current_mode, sock_fd, &win_w, &win_h);
        
        if (current_mode == MODE_CLIENT) {
            resizeterm(win_h + 1, win_w);
        }
    }

    // E. Initial Window Creation & IPC
    status_win = newwin(1, COLS, 0, 0);
    win = create_window(LINES - 1, COLS, 1, 0);
    
    reposition_and_redraw(&win);
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
    
    // --- MAIN LOOP ---
    while (1) {
        set_state(STATE_IDLE); // Mark as Idle before select()

        // 1. Handle UI Input (Exit / Resize)
        int ch = getch();
        if (ch != ERR) {
            set_state(STATE_PROCESSING_INPUT);
            if (ch == 'q') break;
            if (ch == KEY_RESIZE) {
                reposition_and_redraw(&win);
                send_resize(win, fd_drone_write);
            }
        }

        // 2. Periodic Obstacle Logic
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (num_obstacles > 0 && now.tv_sec - last_obst_change.tv_sec >= OBSTACLE_PERIOD_SEC) {
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
        if (fd_obst_read > max_fd) max_fd = fd_obst_read;
        if (fd_targ_read > max_fd) max_fd = fd_targ_read;
        max_fd += 1;

        //int max_fd = (sock_fd > fd_drone_read) ? sock_fd : fd_drone_read;
        //if (fd_input_read > max_fd) max_fd = fd_input_read;

        tv.tv_sec  = 0;
        tv.tv_usec = 50000; // 50ms Timeout

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

                    if (sock_fd >= 0) {
                        net_send_pos(sock_fd, current_x, current_y, win_h);
                    }

                    redraw_scene(win); // Sets STATE_RENDERING
                    
                    // Collision Logic: Drone vs Targets
                    int dx = (int)current_x;
                    int dy = (int)current_y;

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

        // ============================================================
        // SERVER LOGIC (ACTIVE): Invia dati periodicamente
        // ============================================================
        if (current_mode == MODE_SERVER && sock_fd >= 0) {
            // Nota: Per non bloccare tutto troppo spesso, potresti volerlo fare 
            // solo quando i dati cambiano o con un timer, ma qui seguiamo il protocollo sequenziale.
            
            char buf[64];
            char pos_str[64];

            // 1. SEND DRONE
            send_str(sock_fd, "drone");         // cmd: drone
            
            snprintf(pos_str, sizeof(pos_str), "%.2f %.2f", current_x, current_y);
            send_str(sock_fd, pos_str);         // data: x y
            
            recv_str(sock_fd, buf);             // ack: dok <drone> (Wait blocking)
            
            // 2. SEND OBSTACLE (Request remote pos)
            send_str(sock_fd, "obst");          // cmd: obst
            
            recv_str(sock_fd, pos_str);         // data: x y (from client)
            sscanf(pos_str, "%f %f", &remote_x, &remote_y); // Update remote representation
            
            send_str(sock_fd, "pok");           // ack: pok <obstacle>
            
            // Aggiorna l'ostacolo remoto per la fisica del Server
            // (Il server vede il drone client come un ostacolo)
             /* Point p; p.x = (int)remote_x; p.y = (int)remote_y; ... logica ostacolo ... */
        }

        // ============================================================
        // CLIENT LOGIC (REACTIVE): Risponde ai comandi del Server
        // ============================================================
        if (current_mode == MODE_CLIENT && sock_fd >= 0 && FD_ISSET(sock_fd, &readfds)) {
            char cmd[64];
            char buf[64];
            
            // 1. RCV COMMAND (x)
            if (recv_str(sock_fd, cmd) == 0) {
                // Connection closed
                close(sock_fd); sock_fd = -1;
                break; 
            }

            // 2. SWITCH X
            if (strcmp(cmd, "q") == 0) {
                send_str(sock_fd, "qok");
                break; // Exit loop
            }
            else if (strcmp(cmd, "drone") == 0) {
                // RCV X, Y
                recv_str(sock_fd, buf);
                sscanf(buf, "%f %f", &remote_x, &remote_y); // Update remote drone pos
                
                // SND DOK <DRONE>
                char ack[64];
                snprintf(ack, sizeof(ack), "dok %.2f %.2f", remote_x, remote_y);
                send_str(sock_fd, ack);
            }
            else if (strcmp(cmd, "obst") == 0) {
                // SND X, Y (Invio la mia posizione come ostacolo per il server)
                char my_pos[64];
                snprintf(my_pos, sizeof(my_pos), "%.2f %.2f", current_x, current_y);
                send_str(sock_fd, my_pos);
                
                // RCV POK <OBSTACLE>
                recv_str(sock_fd, buf);
            }
            
            redraw_scene(win); // Aggiorna grafica dopo scambio rete
        }
    }

    if (sock_fd >= 0) close(sock_fd);


    /* --- SUB-SECTION: CLEANUP --- */
    if (win) destroy_window(win);
    if (obstacles) free(obstacles);
    if (targets) free(targets);
    logMessage(LOG_PATH, "[BB] Terminated Successfully");
    endwin();
    return 0;
}