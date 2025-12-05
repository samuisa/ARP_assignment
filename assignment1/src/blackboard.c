#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include "app_blackboard.h"
#include "app_common.h"
#include "log.h"

static char last_status[256] = "";

static float current_x = 1.0f;
static float current_y = 1.0f;

static Point *obstacles = NULL;
static int num_obstacles = 0;

static Point *targets = NULL;
static int num_targets = 0;

static WINDOW *status_win = NULL;

/* ======================= WINDOW UTILS ======================= */

WINDOW* create_window(int height, int width, int starty, int startx)
{
    WINDOW *win = newwin(height, width, starty, startx);
    keypad(win, TRUE);
    box(win, 0, 0);
    wnoutrefresh(win);
    doupdate();
    return win;
}

void destroy_window(WINDOW *win)
{
    if (!win) return;
    werase(win);
    wnoutrefresh(win);
    doupdate();
    delwin(win);
}

/* ======================= DRAWING ======================= */

void draw_content(WINDOW *win)
{
    werase(win);
    box(win, 0, 0);

    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    for (int i = 0; i < num_obstacles; i++) {
        int ox = obstacles[i].x;
        int oy = obstacles[i].y;
        if (ox > 0 && ox < max_x - 1 &&
            oy > 0 && oy < max_y - 1) {
            mvwprintw(win, oy, ox, "0");
        }
    }

    for (int i = 0; i < num_targets; i++) {
        int tx = targets[i].x;
        int ty = targets[i].y;
        if (tx > 0 && tx < max_x - 1 &&
            ty > 0 && ty < max_y - 1) {
            mvwprintw(win, ty, tx, "T");
        }
    }
}

void draw_drone(WINDOW *win, float x, float y)
{
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    int ix = (int)x;
    int iy = (int)y;

    if (ix >= max_x - 1) ix = max_x - 2;
    if (iy >= max_y - 1) iy = max_y - 2;
    if (ix < 1) ix = 1;
    if (iy < 1) iy = 1;

    draw_content(win);
    mvwprintw(win, iy, ix, "+");

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();

    logMessage(LOG_PATH, "[BB] Drone drawn at position: (%f, %f)", x, y);
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

void reposition_and_redraw(WINDOW **win_ptr)
{
    if (is_term_resized(LINES, COLS)) {
        resize_term(0, 0);
    }

    int new_width  = COLS;
    int new_height = LINES - 1;

    int startx = 0;
    int starty = 1;

    if (*win_ptr != NULL) {
        if (wresize(*win_ptr, new_height, new_width) == ERR ||
            mvwin(*win_ptr, starty, startx) == ERR) {
            destroy_window(*win_ptr);
            *win_ptr = create_window(new_height, new_width, starty, startx);
            status_win = newwin(1, new_width, 0, 0);
        }
    } else {
        *win_ptr = create_window(new_height, new_width, starty, startx);
        status_win = newwin(1, new_width, 0, 0);
    }

    werase(status_win);
    box(*win_ptr, 0, 0);

    draw_drone(*win_ptr, current_x, current_y);

    logMessage(LOG_PATH, "[BB] Window resized/redrawn: width=%d height=%d", new_width, new_height);
}

/* ======================= IPC ======================= */

void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ)
{
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
    write(fd_obst,  &msg, sizeof(msg));
    write(fd_targ,  &msg, sizeof(msg));

    logMessage(LOG_PATH, "[BB] Window size sent: width=%d height=%d", max_x, max_y);
}

void send_resize(WINDOW *win, int fd_drone){
    Message msg;

    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));

    logMessage(LOG_PATH, "[BB] Resize message sent: width=%d height=%d", max_x, max_y);
}

/* ======================= MAIN ======================= */

int main(int argc, char *argv[]) {
    if (argc < 8) {
        fprintf(stderr, "Usage: %s <pipe_fd_input_read> <fd_drone_read> <fd_drone_write> <fd_obst_read> <fd_obst_write> <fd_targ_write> <fd_targ_read>\n", argv[0]);
        return 1;
    }

    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[2]);
    int fd_drone_write = atoi(argv[3]);
    int fd_obst_write  = atoi(argv[4]);
    int fd_obst_read   = atoi(argv[5]);
    int fd_targ_write  = atoi(argv[6]);
    int fd_targ_read   = atoi(argv[7]);

    float drn_Fx = 0.0f, drn_Fy = 0.0f;
    float obst_Fx = 0.0f, obst_Fy = 0.0f;
    float wall_Fx = 0.0f, wall_Fy = 0.0f;

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    timeout(0);

    logMessage(LOG_PATH, "[BB] main avviato");

    status_win = newwin(1, COLS, 0, 0);
    WINDOW *win = create_window(HEIGHT - 1, WIDTH, 1, 0);
    reposition_and_redraw(&win);
    send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write);

    fd_set readfds;
    struct timeval tv;
    Message msg;

    while (1) {
        int ch = getch();
        if (ch != ERR) {
            if (ch == 'q') break;
            if (ch == KEY_RESIZE) {
                reposition_and_redraw(&win);
                send_resize(win, fd_drone_write);
            }
        }

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
        tv.tv_usec = 50000;

        int ret = select(maxfd, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        // ---- PIPE INPUT ----
        if (FD_ISSET(fd_input_read, &readfds)) {
            char buf[80];
            ssize_t n = read(fd_input_read, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                logMessage(LOG_PATH, "[BB] Input pipe received: '%s'", buf);
                if (buf[0] == 'q') break;
                msg.type = MSG_TYPE_INPUT;
                snprintf(msg.data, sizeof(msg.data), "%s", buf);
                write(fd_drone_write, &msg, sizeof(Message));
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read fd_input");
                break;
            }
        }

        // ---- PIPE DRONE ----
        if (FD_ISSET(fd_drone_read, &readfds)) {
            ssize_t n = read(fd_drone_read, &msg, sizeof(msg));
            if (n > 0) {
                logMessage(LOG_PATH, "[BB] Drone message received: type=%d data='%s'", msg.type, msg.data);
                switch (msg.type) {
                    case MSG_TYPE_POSITION: {
                        if (sscanf(msg.data, "%f %f", &current_x, &current_y) == 2) {
                            draw_drone(win, current_x, current_y);
                        }
                        break;
                    }

                    case MSG_TYPE_FORCE: {
                        if (sscanf(msg.data, "%f %f %f %f %f %f",
                                &drn_Fx, &drn_Fy,
                                &obst_Fx, &obst_Fy,
                                &wall_Fx, &wall_Fy) == 6)
                        {
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
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read fd_drone_read");
                break;
            }
        }

        // ---- PIPE OBSTACLE ----
        if (FD_ISSET(fd_obst_read, &readfds)) {
            ssize_t n = read(fd_obst_read, &msg, sizeof(msg));
            if (n > 0) {
                logMessage(LOG_PATH, "[BB] Obstacles message received: type=%d data='%s'", msg.type, msg.data);
                switch (msg.type) {
                    case MSG_TYPE_OBSTACLES: {
                        int count = 0;
                        if (sscanf(msg.data, "%d", &count) == 1 && count > 0) {
                            if (obstacles != NULL) free(obstacles);
                            obstacles = malloc(sizeof(Point) * count);
                            if (!obstacles) {
                                fprintf(stderr, "Errore allocazione ostacoli\n");
                                break;
                            }
                            ssize_t n_read = read(fd_obst_read, obstacles, sizeof(Point) * count);
                            if ((size_t)n_read != sizeof(Point) * (size_t)count){
                                fprintf(stderr, "Errore lettura ostacoli (bytes letti %zd)\n", n_read);
                                free(obstacles);
                                obstacles = NULL;
                                num_obstacles = 0;
                            } else {
                                num_obstacles = count;
                                for(int i=0;i<num_obstacles;i++)
                                    logMessage(LOG_PATH, "[BB] Obstacle %d: (%f, %f)", i, obstacles[i].x, obstacles[i].y);

                                Message out_msg;
                                out_msg.type = MSG_TYPE_OBSTACLES;
                                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obstacles);

                                write(fd_drone_write, &out_msg, sizeof(out_msg));
                                write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
                                write(fd_targ_write, &out_msg, sizeof(out_msg));
                                write(fd_targ_write, obstacles, sizeof(Point) * num_obstacles);
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }
                draw_drone(win, current_x, current_y);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read fd_obst_read");
                break;
            }
        }

        // ----- PIPE TARGET ------
        if (FD_ISSET(fd_targ_read, &readfds)){
            ssize_t n = read(fd_targ_read, &msg, sizeof(msg));
            if (n > 0) {
                logMessage(LOG_PATH, "[BB] Targets message received: type=%d data='%s'", msg.type, msg.data);
                switch (msg.type) {
                    case MSG_TYPE_TARGETS: {
                        int count = 0;
                        if (sscanf(msg.data, "%d", &count) == 1 && count > 0) {
                            if (targets != NULL) free(targets);
                            targets = malloc(sizeof(Point) * count);
                            if (!targets) {
                                fprintf(stderr, "Errore allocazione targets\n");
                                break;
                            }
                            ssize_t n_read = read(fd_targ_read, targets, sizeof(Point) * count);
                            if ((size_t)n_read != sizeof(Point) * (size_t)count){
                                fprintf(stderr, "Errore lettura targets (bytes letti %zd)\n", n_read);
                                free(targets);
                                targets = NULL;
                                num_targets = 0;
                            } else {
                                num_targets = count;
                                for(int i=0;i<num_targets;i++)
                                    logMessage(LOG_PATH, "[BB] Target %d: (%f, %f)", i, targets[i].x, targets[i].y);

                                Message out_msg;
                                out_msg.type = MSG_TYPE_TARGETS;
                                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);

                                write(fd_drone_write, &out_msg, sizeof(out_msg));
                                write(fd_drone_write, targets, sizeof(Point) * num_targets);
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }
                draw_drone(win, current_x, current_y);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read fd_targ_read");
                break;
            }
        }
    }

    if (win) destroy_window(win);
    if (obstacles) free(obstacles);
    if (targets) free(targets);
    close(fd_input_read);
    close(fd_drone_read);
    close(fd_drone_write);
    close(fd_obst_read);
    close(fd_obst_write);
    close(fd_targ_read);
    close(fd_targ_write);

    logMessage(LOG_PATH, "[BB] main terminato");
    endwin();
    return 0;
}
