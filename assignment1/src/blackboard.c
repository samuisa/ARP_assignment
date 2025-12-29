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

#include "app_blackboard.h"
#include "app_common.h"
#include "process_pid.h"
#include "log.h"

/* * ======================================================================================
 * SECTION 1: GLOBAL STATE & CONFIGURATION
 * Stores the "World State" required for rendering and logic checks.
 * ======================================================================================
 */

// Timers for periodic game events (e.g., moving obstacles)
static struct timespec last_obst_change = {0, 0};
#define OBSTACLE_PERIOD_SEC 5

// UI State Caching (prevents unnecessary terminal refreshes)
static char last_status[256] = "";

// Dynamic Entity Coordinates (Updated via Pipe)
static float current_x = 1.0f;
static float current_y = 1.0f;

// Dynamic Arrays for Game Objects
static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;

// Ncurses Window Handles & Process Safety
static WINDOW *status_win = NULL;
static pid_t watchdog_pid = -1;

/* * ======================================================================================
 * SECTION 2: WATCHDOG & SAFETY MECHANISMS
 * Handles process registration and "Keep-Alive" signaling.
 * ======================================================================================
 */

// Writes this process ID to a shared file so the Watchdog can monitor it.
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

// Signal Handler for SIGUSR1: Responds to the Watchdog's heartbeat check.
void watchdog_ping_handler(int sig) {
    (void)sig; 
    if (watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

/* * ======================================================================================
 * SECTION 3: NCURSES WINDOW MANAGEMENT
 * Helpers for creating, destroying, and refreshing UI containers.
 * ======================================================================================
 */

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

/* * ======================================================================================
 * SECTION 4: RENDERING ENGINE
 * Functions responsible for drawing game entities on the screen.
 * ======================================================================================
 */

void draw_background(WINDOW *win) {
    werase(win);
    box(win, 0, 0);
}

// Renders obstacles as Red 'O' characters
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

// Renders targets as Green 'T' characters
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

// Renders the Drone as a Blue '+' character and handles map clamping
void draw_drone(WINDOW *win, float x, float y) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    int ix = (int)x;
    int iy = (int)y;

    // Boundary Checks: Ensure the drone stays inside the box
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

// Master refresh function: Clears screen and redraws all layers
void redraw_scene(WINDOW *win) {
    draw_background(win);
    draw_obstacles(win);
    draw_targets(win);
    draw_drone(win, current_x, current_y);

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();
}

/* * ======================================================================================
 * SECTION 5: INTERNAL GAME LOGIC
 * Helper functions for game rules (collision, generation).
 * ======================================================================================
 */

// Checks if a specific coordinate is already occupied by a target
int overlaps_target(int x, int y) {
    for (int i = 0; i < num_targets; i++) {
        if (targets[i].x == x && targets[i].y == y) return 1;
    }
    return 0;
}

// Moves a random obstacle to a new valid location (Periodic Event)
void generate_new_obstacle(int idx, int width, int height) {
    int valid;
    do {
        valid = 1;
        obstacles[idx].x = rand() % (width - 2) + 1;
        obstacles[idx].y = rand() % (height - 2) + 1;

        // Collision check: Obstacle vs Obstacle
        for (int i = 0; i < num_obstacles; i++) {
            if (i != idx && obstacles[idx].x == obstacles[i].x && obstacles[idx].y == obstacles[i].y) {
                valid = 0; break;
            }
        }
        // Collision check: Obstacle vs Target
        if (valid && overlaps_target(obstacles[idx].x, obstacles[idx].y)) {
            valid = 0;
        }
    } while (!valid);
}

/* * ======================================================================================
 * SECTION 6: UI UPDATES & RESIZING
 * Handles the top status bar and terminal window resizing events.
 * ======================================================================================
 */

// Updates the top status bar with real-time physics data from the drone
void update_dynamic(float x, float y,
                    float drn_Fx, float drn_Fy,
                    float obst_Fx, float obst_Fy,
                    float wall_Fx, float wall_Fy,
                    float targ_Fx, float targ_Fy)
{
    if (!status_win) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "x=%.4f y=%.4f | drn(%.4f %.4f) | obst(%.4f %.4f) | wall(%.4f %.4f) | targ(%.4f %.4f)",
        x, y,
        drn_Fx, drn_Fy,
        obst_Fx, obst_Fy,
        wall_Fx, wall_Fy,
        targ_Fx, targ_Fy
    );

    // Only redraw if the text content has actually changed
    if (strcmp(buffer, last_status) != 0) {
        strcpy(last_status, buffer);

        werase(status_win);
        mvwprintw(status_win, 0, 0, "%s", buffer);

        wnoutrefresh(status_win);
        doupdate();
    }
}

// Handles SIGWINCH-like events: Re-calculates dimensions and recreates windows
void reposition_and_redraw(WINDOW **win_ptr) {
    if (is_term_resized(LINES, COLS)) {
        resize_term(0, 0);
    }

    int new_width  = COLS;
    int new_height = LINES - 1;
    int startx = 0;
    int starty = 1;

    // Logic to either resize existing window or create a new one
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

/* * ======================================================================================
 * SECTION 7: IPC (INTER-PROCESS COMMUNICATION)
 * Functions to send data packets via Pipes to other processes.
 * ======================================================================================
 */

// Broadcasts map dimensions to Drone, Obstacle, and Target processes
void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ) {
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
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);
    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);
    write(fd_drone, &msg, sizeof(msg));
}

/* * ======================================================================================
 * SECTION 8: MAIN ENTRY POINT
 * Setup, Argument Parsing, and the Main Event Loop.
 * ======================================================================================
 */
int main(int argc, char *argv[]) {
    // 1. Argument Parsing (File Descriptors for Pipes)
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

    // 2. Signal Setup
    signal(SIGPIPE, SIG_IGN); 

    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    // ==========================================================
    // INIZIALIZZAZIONE SICURA CON LOCK/FLUSH
    // ==========================================================
    
    // PASSO A: Aspetta il watchdog (SENZA LOCK per evitare deadlock)
    wait_for_watchdog_pid();

    // PASSO B: Scrivi il proprio PID (CON LOCK come richiesto)
    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        logMessage(LOG_PATH, "[DRONE] Error opening PID file!");
        exit(1);
    }

    // 1. ACQUISISCI LOCK
    int fd_pid = fileno(fp_pid);
    flock(fd_pid, LOCK_EX); 

    // 2. CHIAMA FUNZIONE DI SCRITTURA
    publish_my_pid(fp_pid);

    // 3. FLUSH PER FORZARE SCRITTURA SU DISCO
    fflush(fp_pid);

    // 4. RILASCIA LOCK
    flock(fd_pid, LOCK_UN);

    // 5. CHIUDI
    fclose(fp_pid);

    // 4. Ncurses Initialization
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

    // 5. Window Creation & Initial IPC Handshake
    status_win = newwin(1, COLS, 0, 0);
    WINDOW *win = create_window(LINES - 1, COLS, 1, 0);
    
    reposition_and_redraw(&win);
    send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write);

    // Physics variables cache
    float drn_Fx = 0.0f, drn_Fy = 0.0f;
    float obst_Fx = 0.0f, obst_Fy = 0.0f;
    float wall_Fx = 0.0f, wall_Fy = 0.0f;
    float targ_Fx = 0.0f, targ_Fy = 0.0f;

    logMessage(LOG_PATH, "[BB] Ready and GUI started");

    // I/O Multiplexing Variables
    fd_set readfds;
    struct timeval tv;
    Message msg;
    
    /* * ======================================================================================
     * MAIN EVENT LOOP
     * Uses select() to listen for data from Input, Drone, Obstacles, and Targets simultaneously.
     * ======================================================================================
     */
    while (1) {
        // A. User Interface Input Handling
        int ch = getch();
        if (ch != ERR) {
            if (ch == 'q') break;
            if (ch == KEY_RESIZE) {
                reposition_and_redraw(&win);
                send_resize(win, fd_drone_write);
            }
        }

        // B. Periodic Obstacle Movement (Timer Logic)
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (num_obstacles > 0 && now.tv_sec - last_obst_change.tv_sec >= OBSTACLE_PERIOD_SEC) {
            last_obst_change = now;
            
            // Move one random obstacle
            int idx = rand() % num_obstacles;
            int max_y, max_x;
            getmaxyx(win, max_y, max_x);
            generate_new_obstacle(idx, max_x, max_y);
            
            redraw_scene(win);

            // Broadcast new obstacle positions to Drone
            Message m;
            m.type = MSG_TYPE_OBSTACLES;
            snprintf(m.data, sizeof(m.data), "%d", num_obstacles);
            write(fd_drone_write, &m, sizeof(m));
            write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
        }

        // C. Setup Select() for I/O Multiplexing
        FD_ZERO(&readfds);
        FD_SET(fd_input_read, &readfds);
        FD_SET(fd_drone_read, &readfds);
        FD_SET(fd_obst_read, &readfds);
        FD_SET(fd_targ_read, &readfds); 
        
        int maxfd = fd_input_read;
        if (fd_drone_read > maxfd) maxfd = fd_drone_read;
        if (fd_obst_read > maxfd) maxfd = fd_obst_read;
        if (fd_targ_read > maxfd) maxfd = fd_targ_read;
        maxfd += 1;

        tv.tv_sec  = 0;
        tv.tv_usec = 50000; // 50ms Timeout

        int ret = select(maxfd, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // D. Handle Data from INPUT Process (Keyboard)
        if (FD_ISSET(fd_input_read, &readfds)) {
            char buf[80];
            ssize_t n = read(fd_input_read, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                if (buf[0] == 'q'){
                    write(fd_wd_write, buf, sizeof(buf));
                    break;
                }
                // Forward keypress to the Drone Process
                msg.type = MSG_TYPE_INPUT;
                snprintf(msg.data, sizeof(msg.data), "%s", buf);
                write(fd_drone_write, &msg, sizeof(Message));
                
            }
        }

        // E. Handle Data from DRONE Process
        if (FD_ISSET(fd_drone_read, &readfds)) {
            if (read(fd_drone_read, &msg, sizeof(msg)) > 0) {
                switch (msg.type) {

                // Sub-case: Drone Position Update & Collision Logic
                case MSG_TYPE_POSITION:{
                    sscanf(msg.data, "%f %f", &current_x, &current_y);
                    redraw_scene(win);
                    
                    // Check Collision: Drone vs Targets
                    int dx = (int)current_x;
                    int dy = (int)current_y;
                    for (int i = 0; i < num_targets; i++) {
                        if (dx == (int)targets[i].x && dy == (int)targets[i].y) {
                            // Target Hit! Remove it from array.
                            for (int j = i; j < num_targets - 1; j++) targets[j] = targets[j + 1];
                            num_targets--;
                            
                            // Notify Drone about remaining targets
                            Message out_msg;
                            out_msg.type = MSG_TYPE_TARGETS;
                            snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                            write(fd_drone_write, &out_msg, sizeof(out_msg));
                            write(fd_drone_write, targets, sizeof(Point) * num_targets);

                            // Win Condition / Respawn Logic
                            if (num_targets == 0) {
                                logMessage(LOG_PATH, "[BB] ALL TARGETS CLEARED");
                                // Trigger Target Process to respawn based on obstacle layout
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

                // Sub-case: Physics/Force Data Update
                case MSG_TYPE_FORCE: {
                    if (sscanf(msg.data, "%f %f %f %f %f %f %f %f",
                            &drn_Fx, &drn_Fy,
                            &obst_Fx, &obst_Fy,
                            &wall_Fx, &wall_Fy,
                            &targ_Fx, &targ_Fy) == 8) {
                        update_dynamic(current_x, current_y,
                                    drn_Fx, drn_Fy,
                                    obst_Fx, obst_Fy,
                                    wall_Fx, wall_Fy,
                                    targ_Fx, targ_Fy);
                    }
                    break;
                }

                    default:
                    break;

                }
            }
        }

        // F. Handle Data from OBSTACLE Process
        if (FD_ISSET(fd_obst_read, &readfds)) {
            if (read(fd_obst_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_OBSTACLES) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(obstacles);
                    obstacles = malloc(sizeof(Point) * count);
                    read(fd_obst_read, obstacles, sizeof(Point) * count);
                    num_obstacles = count;
                    
                    logMessage(LOG_PATH, "[BB] received %d obstacles", num_obstacles);
                    // Log loop omitted for brevity
                    
                    // Forward new obstacles to Drone & Target processes
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

        // G. Handle Data from TARGET Process
        if (FD_ISSET(fd_targ_read, &readfds)) {
            if (read(fd_targ_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_TARGETS) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(targets);
                    targets = malloc(sizeof(Point) * count);
                    read(fd_targ_read, targets, sizeof(Point) * count);
                    num_targets = count;

                    // Forward new targets to Drone & Obstacle processes
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

    /* * ======================================================================================
     * SECTION 9: CLEANUP & EXIT
     * Frees resources and restores the terminal state.
     * ======================================================================================
     */
    if (win) destroy_window(win);
    if (obstacles) free(obstacles);
    if (targets) free(targets);
    // Close FDs...
    logMessage(LOG_PATH, "[BB] Terminated Successfully");
    endwin(); // CRITICAL: Restores terminal to normal mode
    return 0;
}