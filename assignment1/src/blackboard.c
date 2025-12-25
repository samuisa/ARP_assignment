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

#include "app_blackboard.h"
#include "app_common.h"
#include "process_pid.h"
#include "log.h"

// ================= GLOBALS =================
static struct timespec last_obst_change = {0, 0};
#define OBSTACLE_PERIOD_SEC 5

static char last_status[256] = "";
static float current_x = 1.0f;
static float current_y = 1.0f;

static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;

static WINDOW *status_win = NULL;
static pid_t watchdog_pid = -1;

// ================= PID & WATCHDOG =================
void publish_my_pid() {
    FILE *fp = fopen(PID_FILE_PATH, "a"); 
    if (!fp) exit(1);
    fprintf(fp, "%s %d\n", BB_PID_TAG, getpid());
    fclose(fp);
}

void wait_for_watchdog() {
    FILE *fp;
    char line[256], tag[128];
    int pid;
    bool found = false;
    // Usiamo printf qui perché ncurses non è ancora partito
    printf("[BLACKBOARD] In attesa del Watchdog...\n");
    fflush(stdout);

    while (!found) {
        fp = fopen(PID_FILE_PATH, "r");
        if (fp) {
            while (fgets(line, sizeof(line), fp)) {
                if (sscanf(line, "%s %d", tag, &pid) == 2) {
                    if (strcmp(tag, WD_PID_TAG) == 0) {
                        watchdog_pid = pid;
                        found = true;
                        break;
                    }
                }
            }
            fclose(fp);
        }
        if (!found) usleep(100000);
    }
    printf("[BLACKBOARD] Watchdog trovato: %d\n", watchdog_pid);
    fflush(stdout);
}

void watchdog_ping_handler(int sig) {
    (void)sig; 
    if (watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

// ================= WINDOW UTILS =================

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

/* ======================= DRAWING ======================= */

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
            mvwprintw(win, oy, ox, "O"); // Ho cambiato "0" in "O" per visibilità
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

    // Log ridotto per non intasare il disco
    // logMessage(LOG_PATH, "[BB] Drone drawn at (%f, %f)", x, y); 
}

void redraw_scene(WINDOW *win) {
    draw_background(win);
    draw_obstacles(win);
    draw_targets(win);
    draw_drone(win, current_x, current_y);

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();
}

int overlaps_target(int x, int y) {
    for (int i = 0; i < num_targets; i++) {
        if (targets[i].x == x && targets[i].y == y) return 1;
    }
    return 0;
}

void generate_new_obstacle(int idx, int width, int height) {
    int valid;
    do {
        valid = 1;
        obstacles[idx].x = rand() % (width - 2) + 1;
        obstacles[idx].y = rand() % (height - 2) + 1;

        for (int i = 0; i < num_obstacles; i++) {
            if (i != idx && obstacles[idx].x == obstacles[i].x && obstacles[idx].y == obstacles[i].y) {
                valid = 0; break;
            }
        }
        if (valid && overlaps_target(obstacles[idx].x, obstacles[idx].y)) {
            valid = 0;
        }
    } while (!valid);
}

/* ======================= STATUS BAR ======================= */

void update_dynamic(float x, float y,
                    float drn_Fx, float drn_Fy,
                    float obst_Fx, float obst_Fy,
                    float wall_Fx, float wall_Fy)
{
    if (!status_win) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "x=%.4f y=%.4f | drn(%.4f %.4f) | obst(%.4f %.4f) | wall(%.4f %.4f)",
        x, y,
        drn_Fx, drn_Fy,
        obst_Fx, obst_Fy,
        wall_Fx, wall_Fy
    );

    if (strcmp(buffer, last_status) != 0) {
        strcpy(last_status, buffer);

        werase(status_win);
        mvwprintw(status_win, 0, 0, "%s", buffer);

        wnoutrefresh(status_win);
        doupdate();

        logMessage(LOG_PATH,
            "[BB] Dynamic update: position=(%f,%f) drn=(%f,%f) obst=(%f,%f) wall=(%f,%f)",
            x, y, drn_Fx, drn_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy);
    }
}

/* ======================= RESIZE ======================= */

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

/* ======================= IPC ======================= */

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

// ================= MAIN =================
int main(int argc, char *argv[]) {
    // Controllo argomenti critico
    if (argc < 8) {
        fprintf(stderr, "[BB] Errore: Servono 7 file descriptors, ricevuti %d\n", argc-1);
        return 1;
    }

    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[2]);
    int fd_drone_write = atoi(argv[3]);
    int fd_obst_write  = atoi(argv[4]);
    int fd_obst_read   = atoi(argv[5]);
    int fd_targ_write  = atoi(argv[6]);
    int fd_targ_read   = atoi(argv[7]);

    signal(SIGPIPE, SIG_IGN); 

    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    // 1. Aspetta Watchdog (BLOCCANTE, stampa su stdout)
    wait_for_watchdog();

    // 2. Pubblica PID
    publish_my_pid();

    // 3. INIZIALIZZA NCURSES (MANCAVA QUESTO!)
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE); // <--- QUESTA RIGA MANCAVA! FIX CRUCIALE
    curs_set(0); // Nasconde il cursore
    start_color();
    use_default_colors();
    init_pair(1, COLOR_BLUE, -1);  // Drone
    init_pair(2, COLOR_RED, -1);   // Ostacoli
    init_pair(3, COLOR_GREEN, -1); // Target
    refresh();

    // 4. Crea finestre
    status_win = newwin(1, COLS, 0, 0);
    WINDOW *win = create_window(LINES - 1, COLS, 1, 0);
    
    // 5. Invia dimensioni iniziali
    reposition_and_redraw(&win);
    send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write);

    float drn_Fx = 0.0f, drn_Fy = 0.0f;
    float obst_Fx = 0.0f, obst_Fy = 0.0f;
    float wall_Fx = 0.0f, wall_Fy = 0.0f;

    logMessage(LOG_PATH, "[BB] Ready and GUI started");

    fd_set readfds;
    struct timeval tv;
    Message msg;
    
    while (1) {
        // Controllo input tastiera (ncurses)
        int ch = getch();
        if (ch != ERR) {
            if (ch == 'q') break;
            if (ch == KEY_RESIZE) {
                reposition_and_redraw(&win);
                send_resize(win, fd_drone_write);
            }
        }

        // Timer rigenerazione ostacoli
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (num_obstacles > 0 && now.tv_sec - last_obst_change.tv_sec >= OBSTACLE_PERIOD_SEC) {
            last_obst_change = now;
            int idx = rand() % num_obstacles;
            int max_y, max_x;
            getmaxyx(win, max_y, max_x);
            generate_new_obstacle(idx, max_x, max_y);
            
            redraw_scene(win);

            // Invia aggiornamento
            Message m;
            m.type = MSG_TYPE_OBSTACLES;
            snprintf(m.data, sizeof(m.data), "%d", num_obstacles);
            write(fd_drone_write, &m, sizeof(m));
            write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
        }

        // Setup Select
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
        tv.tv_usec = 50000; // 50ms refresh

        int ret = select(maxfd, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // Lettura PIPE: INPUT
        if (FD_ISSET(fd_input_read, &readfds)) {
            char buf[80];
            ssize_t n = read(fd_input_read, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                if (buf[0] == 'q') break;
                // Inoltra input al drone
                msg.type = MSG_TYPE_INPUT;
                snprintf(msg.data, sizeof(msg.data), "%s", buf);
                write(fd_drone_write, &msg, sizeof(Message));
            }
        }

        // Lettura PIPE: DRONE
        if (FD_ISSET(fd_drone_read, &readfds)) {
            if (read(fd_drone_read, &msg, sizeof(msg)) > 0) {
                switch (msg.type) {

                case MSG_TYPE_POSITION:{
                    sscanf(msg.data, "%f %f", &current_x, &current_y);
                    redraw_scene(win);
                    
                    // Gestione collisione Target (semplificata)
                    int dx = (int)current_x;
                    int dy = (int)current_y;
                    for (int i = 0; i < num_targets; i++) {
                        if (dx == (int)targets[i].x && dy == (int)targets[i].y) {
                            // Rimuovi target
                            for (int j = i; j < num_targets - 1; j++) targets[j] = targets[j + 1];
                            num_targets--;
                            
                            // Notifica rimozione
                            Message out_msg;
                            out_msg.type = MSG_TYPE_TARGETS;
                            snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                            write(fd_drone_write, &out_msg, sizeof(out_msg));
                            
                            // Rigenera ostacoli se finiti target
                            if (num_targets == 0) {
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
                    if (sscanf(msg.data, "%f %f %f %f %f %f",
                            &drn_Fx, &drn_Fy,
                            &obst_Fx, &obst_Fy,
                            &wall_Fx, &wall_Fy) == 6) {
                        update_dynamic(current_x, current_y,
                                    drn_Fx, drn_Fy,
                                    obst_Fx, obst_Fy,
                                    wall_Fx, wall_Fy);
                    }
                    break;
                }

                    default:
                    break;

                }
            }
        }

        // Lettura PIPE: OBSTACLES
        if (FD_ISSET(fd_obst_read, &readfds)) {
            if (read(fd_obst_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_OBSTACLES) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(obstacles);
                    obstacles = malloc(sizeof(Point) * count);
                    read(fd_obst_read, obstacles, sizeof(Point) * count);
                    num_obstacles = count;
                    
                    // Inoltra ostacoli agli altri processi
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

        // Lettura PIPE: TARGETS
        if (FD_ISSET(fd_targ_read, &readfds)) {
            if (read(fd_targ_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_TARGETS) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(targets);
                    targets = malloc(sizeof(Point) * count);
                    read(fd_targ_read, targets, sizeof(Point) * count);
                    num_targets = count;

                    // Inoltra targets
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

    // PULIZIA FINALE
    if (win) destroy_window(win);
    if (obstacles) free(obstacles);
    if (targets) free(targets);
    // Chiudi FD...
    logMessage(LOG_PATH, "[BB] Terminazione corretta");
    endwin(); // CRUCIALE: Ripristina il terminale
    return 0;
}