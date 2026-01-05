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

// System handles
static WINDOW *status_win = NULL;
static pid_t watchdog_pid = -1;

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
 * SECTION 1: CONNESSIONE AL SERVER
 * ====================================================================================== */

int connect_to_server(const char *host, int port) {
    int sockfd;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    while (1) {
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("[BB] socket");
            sleep(1);
            continue;
        }

        server = gethostbyname(host);
        if (!server) {
            logMessage(LOG_PATH_SC, "[BB] host not found, retrying...");
            printf("host not found, retrying...");
            close(sockfd);
            sleep(1);
            continue;
        }

        bzero(&serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        memcpy(&serv_addr.sin_addr.s_addr,
               server->h_addr,
               server->h_length);
        serv_addr.sin_port = htons(port);

        if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == 0) {
            logMessage(LOG_PATH_SC, "[BB->SERVER] CONNECTED to server");
            return sockfd;
        }

        logMessage(LOG_PATH_SC, "[BB->SERVER] connect failed (errno=%d), retrying...", errno);
        close(sockfd);
        sleep(1);
    }
}

int connect_to_client(const char *host, int port) {
    int sockfd;
    struct sockaddr_in cli_addr;
    struct hostent *server;

    while (1) {
        // 1. Creazione socket
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("[BB->CLIENT] socket");
            sleep(1);
            continue;
        }

        // 2. Risoluzione hostname
        server = gethostbyname(host);
        if (!server) {
            logMessage(LOG_PATH_SC, "[BB->CLIENT] host '%s' not found, retrying in 1s...", host);
            close(sockfd);
            sleep(1);
            continue;
        }

        // 3. Setup indirizzo client
        bzero(&cli_addr, sizeof(cli_addr));
        cli_addr.sin_family = AF_INET;
        memcpy(&cli_addr.sin_addr.s_addr, server->h_addr, server->h_length);
        cli_addr.sin_port = htons(port);

        // 4. Tentativo di connessione
        if (connect(sockfd, (struct sockaddr *)&cli_addr, sizeof(cli_addr)) == 0) {
            logMessage(LOG_PATH_SC, "[BB->CLIENT] Connected to client %s:%d", host, port);
            return sockfd;
        }

        // 5. Connessione fallita: chiudo e riprovo
        logMessage(LOG_PATH_SC, "[BB->CLIENT] connect to %s:%d failed (errno=%d, %s), retrying in 1s...",
                   host, port, errno, strerror(errno));
        close(sockfd);
        sleep(1);  // Retry lento per dare tempo al client di mettersi in ascolto
    }
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
    addr.sin_port = htons(NET_PORT);

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
    struct hostent *server = gethostbyname("localhost");

    addr.sin_family = AF_INET;
    memcpy(&addr.sin_addr.s_addr, server->h_addr, server->h_length);
    addr.sin_port = htons(NET_PORT);

    logMessage(LOG_PATH_SC, "[CLIENT] Connecting...");
    while (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        mvprintw(LINES/2, COLS/2 - 10, "CONNECTING..."); refresh();
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
            mvwprintw(win, ty, tx, "T");
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

/* ======================================================================================
 * SECTION 8: MAIN EXECUTION
 * Setup, Watchdog Synchronization, I/O Multiplexing Loop.
 * ====================================================================================== */

int main(int argc, char *argv[]) {
    // A. Parse Arguments (Pipe File Descriptors)
    if (argc < 10) {
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

    // --- NETWORK HANDSHAKE ---
    WINDOW *win = NULL;
    int win_h = LINES - 1, win_w = COLS;

    if (current_mode == MODE_SERVER) {
        sock_fd = init_server();
        // Send Dimensions
        char dim[64];
        snprintf(dim, sizeof(dim), "%d %d", win_w, win_h);
        write(sock_fd, dim, strlen(dim));
        char ack[2]; read(sock_fd, ack, 1); // Wait ACK
    } 
    else if (current_mode == MODE_CLIENT) {
        sock_fd = init_client();
        // Recv Dimensions
        char dim[64];
        read(sock_fd, dim, sizeof(dim));
        sscanf(dim, "%d %d", &win_w, &win_h);
        write(sock_fd, ACK_MSG, ACK_LEN); // Send ACK
        resizeterm(win_h + 1, win_w); // Resize local terminal
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
        if (sock_fd > max_fd) max_fd = sock_fd;
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
                            for (int j = i; j < num_targets - 1; j++) targets[j] = targets[j + 1];
                            num_targets--;
                            
                            // Notify Drone
                            set_state(STATE_BROADCASTING);
                            Message out_msg;
                            out_msg.type = MSG_TYPE_TARGETS;
                            snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                            write(fd_drone_write, &out_msg, sizeof(out_msg));
                            write(fd_drone_write, targets, sizeof(Point) * num_targets);

                            // Win Condition: Trigger Respawn
                            if (num_targets == 0) {
                                logMessage(LOG_PATH, "[BB] ALL TARGETS CLEARED");
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

        // 3. NETWORK RECEPTION
        if (sock_fd >= 0 && FD_ISSET(sock_fd, &readfds)) {
            float rx, ry;
            if (net_recv_pos(sock_fd, &rx, &ry, win_h)) {
                remote_x = rx; remote_y = ry;
                
                // IF SERVER: Remote drone is an OBSTACLE for local drone
                if (current_mode == MODE_SERVER) {
                     Point p; p.x = (int)remote_x; p.y = (int)remote_y;
                     Message obst_msg; 
                     obst_msg.type = MSG_TYPE_OBSTACLES;
                     snprintf(obst_msg.data, sizeof(obst_msg.data), "1"); // 1 Obstacle
                     write(fd_drone_write, &obst_msg, sizeof(obst_msg));
                     write(fd_drone_write, &p, sizeof(Point));
                }
            } else {
                // Connection closed
                break; 
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