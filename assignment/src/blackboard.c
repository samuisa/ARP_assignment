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
// ----------------------------------------

// Timers for periodic events (Obstacle movement)
static struct timespec last_obst_change = {0, 0};
#define OBSTACLE_PERIOD_SEC 5

// UI Optimization (Cache)
static char last_status[256] = "";

// Dynamic Game Entities
static float current_x = 1.0f;
static float current_y = 1.0f;
static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;

// System handles
static WINDOW *status_win = NULL;
static pid_t watchdog_pid = -1;

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

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();
}

// Master refresh function
void redraw_scene(WINDOW *win) {
    set_state(STATE_RENDERING); // Update State
    draw_background(win);
    draw_obstacles(win);
    draw_targets(win);
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

void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ, int server_fd) {
    set_state(STATE_BROADCASTING); // Update State
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
    write(fd_obst,  &msg, sizeof(msg));
    write(fd_targ,  &msg, sizeof(msg));

    char msg_server[64];
    snprintf(msg_server, sizeof(msg_server), "size %d %d\n", max_x, max_y);
    write(server_fd, msg_server, strlen(msg_server));
    
    logMessage(LOG_PATH, "[BB] SEND WINDOW %s", msg_server);
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
    if (argc < 9) {
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

    int server_fd = connect_to_server("localhost", 5000);
    logMessage(LOG_PATH_SC, "[BB] Connected to server");

    int client_fd = connect_to_client("localhost", 5001); // attenzione alla porta
    send_msg(client_fd, "BB\n"); // dici al client chi sei
    logMessage(LOG_PATH_SC, "[BB] Sent identity to client");


    send_msg(server_fd, "BB\n");
    logMessage(LOG_PATH_SC, "[BB] Sent identity to server");


    signal(SIGPIPE, SIG_IGN); 

    // B. Setup Watchdog Signal Handler
    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    // C. Watchdog Synchronization and PID Publishing (Secure)
    wait_for_watchdog_pid();

    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        logMessage(LOG_PATH, "[DRONE] Error opening PID file!");
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

    // E. Initial Window Creation & IPC
    status_win = newwin(1, COLS, 0, 0);
    WINDOW *win = create_window(LINES - 1, COLS, 1, 0);
    
    reposition_and_redraw(&win);
    send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write, server_fd);


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
        FD_SET(fd_obst_read, &readfds);
        FD_SET(fd_targ_read, &readfds); 
        FD_SET(server_fd, &readfds); 
        FD_SET(client_fd, &readfds); 

        
        int maxfd = fd_input_read;
        if (fd_drone_read > maxfd) maxfd = fd_drone_read;
        if (fd_obst_read > maxfd) maxfd = fd_obst_read;
        if (fd_targ_read > maxfd) maxfd = fd_targ_read;
        if (server_fd > maxfd) maxfd = server_fd;
        if (client_fd > maxfd) maxfd = client_fd;
        maxfd += 1;

        tv.tv_sec  = 0;
        tv.tv_usec = 50000; // 50ms Timeout

        int ret = select(maxfd, &readfds, NULL, NULL, &tv);
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

        if (FD_ISSET(server_fd, &readfds)) {
            char buf[BUFSZ];
            recv_msg(server_fd, buf, sizeof(buf));

            if (strncmp(buf, "drone", 5) == 0) {
                char drone_msg[64];
                snprintf(drone_msg, sizeof(drone_msg), "%d %d\n", (int)current_x, (int)current_y);
                send_msg(server_fd, drone_msg);
                logMessage(LOG_PATH_SC, "[BB] Drone position sent to server: %s", drone_msg);
            }
        }

        if (FD_ISSET(client_fd, &readfds)) {
            char buf[BUFSZ];
            recv_msg(client_fd, buf, sizeof(buf));
            
            if (strncmp(buf, "send_obst", 9) == 0) {
                // Costruisci stringa ostacoli
                char obst_msg[BUFSZ];
                int offset = 0;
                for (int i = 0; i < num_obstacles; i++) {
                    offset += snprintf(obst_msg + offset, sizeof(obst_msg) - offset,
                                    "%d %d;", obstacles[i].x, obstacles[i].y);
                }
                strncat(obst_msg, "\n", sizeof(obst_msg) - strlen(obst_msg) - 1);
                
                send_msg(client_fd, obst_msg);
                logMessage(LOG_PATH_SC, "[BB] Obstacles sent to client: %s", obst_msg);
            }
        }

    }

    close(server_fd);
    logMessage(LOG_PATH, "[BB] close serverfd");


    /* --- SUB-SECTION: CLEANUP --- */
    if (win) destroy_window(win);
    if (obstacles) free(obstacles);
    if (targets) free(targets);
    logMessage(LOG_PATH, "[BB] Terminated Successfully");
    endwin();
    return 0;
}