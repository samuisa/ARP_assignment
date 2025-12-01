#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include "app_window.h"
#include "app_common.h"
#include "log.h"

#define LOG_PATH "logs/system.log"

static int current_x = 1;
static int current_y = 1;
static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;

WINDOW* create_window(int height, int width, int starty, int startx) {
    WINDOW *win = newwin(height, width, starty, startx);
    keypad(win, TRUE);
    box(win, 0, 0);
    wrefresh(win);
    return win;
}

void destroy_window(WINDOW *win) {
    if (!win) return;
    wborder(win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
    wrefresh(win);
    delwin(win);
}

void draw_content(WINDOW *win, int x, int y) {
    werase(win);
    box(win, 0, 0);
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    for (int i = 0; i < num_obstacles; i++) {
        int ox = obstacles[i].x;
        int oy = obstacles[i].y;
        if (ox >= 0 && ox < max_x && oy >= 0 && oy < max_y) {
            mvwprintw(win, oy, ox, "0");
        }
    }

    for (int i = 0; i < num_targets; i++) {
        int tx = targets[i].x;
        int ty = targets[i].y;
        if (tx >= 0 && tx < max_x && ty >= 0 && ty < max_y) {
            mvwprintw(win, ty, tx, "T");
        }
    }

    if (x >= (max_x - 1)) x = max_x - 2;
    if (y >= (max_y - 1)) y = max_y - 2;
    if (x < 1) x = 1;
    if (y < 1) y = 1;
    mvwprintw(win, y, x, "+");
    wrefresh(win);
}

void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ) {
    Message msg;

    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
    write(fd_obst,  &msg, sizeof(msg));
    write(fd_targ, &msg, sizeof(msg));
}

void send_resize(WINDOW *win, int fd_drone){
    Message msg;

    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
}

void reposition_and_redraw(WINDOW **win_ptr) {
    if (is_term_resized(LINES, COLS)) {
        resize_term(0, 0);
    }

    int new_width = COLS;
    int new_height = LINES;

    int startx = (COLS - new_width) / 2;
    int starty = (LINES - new_height) / 2;

    if (*win_ptr != NULL) {
        if (wresize(*win_ptr, new_height, new_width) == ERR ||
            mvwin(*win_ptr, starty, startx) == ERR) {
            destroy_window(*win_ptr);
            *win_ptr = create_window(new_height, new_width, starty, startx);
        }
    } else {
        *win_ptr = create_window(new_height, new_width, starty, startx);
    }

    draw_content(*win_ptr, current_x, current_y);
}


int main(int argc, char *argv[]) {
    if (argc < 8) {
        fprintf(stderr, "Usage: %s <pipe_fd_input_read> <pipe_fd_input_write> <fd_drone_read> <fd_drone_write> <fd_obst_read> <fd_obst_write>\n", argv[0]);
        return 1;
    }
    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[3]);
    int fd_drone_write = atoi(argv[4]);
    int fd_obst_write  = atoi(argv[5]);
    int fd_obst_read   = atoi(argv[6]);
    int fd_targ_write  = atoi(argv[7]);
    int fd_targ_read   = atoi(argv[8]);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    timeout(0);

    logMessage(LOG_PATH, " [WIN] main avviato");
    WINDOW *win = create_window(HEIGHT, WIDTH, 0, 0);
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
                switch (msg.type) {
                    case MSG_TYPE_POSITION: {
                        if (sscanf(msg.data, "%d %d", &current_x, &current_y) == 2) {
                            draw_content(win, current_x, current_y);
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
                switch (msg.type) {
                    case MSG_TYPE_OBSTACLES: {
                        int count = 0;
                        if (sscanf(msg.data, "%d", &count) == 1 && count > 0) {
                            if (obstacles != NULL) {
                                free(obstacles);
                                obstacles = NULL;
                            }
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
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read fd_obst_read");
                break;
            }
        }

        // ----- PIPE TARGET ------
        if (FD_ISSET(fd_targ_read, &readfds)){
            ssize_t n = read(fd_targ_read, &msg, sizeof(msg));
            if (n > 0) {
                switch (msg.type) {
                    case MSG_TYPE_TARGETS: {
                        int count = 0;
                        if (sscanf(msg.data, "%d", &count) == 1 && count > 0) {
                            if (targets != NULL) {
                                free(targets);
                                targets = NULL;
                            }
                            targets = malloc(sizeof(Point) * count);
                            if (!targets) {
                                fprintf(stderr, "Errore allocazione ostacoli\n");
                                break;
                            }
                            ssize_t n_read = read(fd_targ_read, targets, sizeof(Point) * count);
                            if ((size_t)n_read != sizeof(Point) * (size_t)count){
                                fprintf(stderr, "Errore lettura ostacoli (bytes letti %zd)\n", n_read);
                                free(targets);
                                targets = NULL;
                                num_targets = 0;
                            } else {
                                num_targets = count;
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
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("read fd_obst_read");
                break;
            }
        }
    }

    // Pulizia finale
    if (win != NULL) destroy_window(win);
    endwin();
    if (obstacles != NULL) free(obstacles);
    close(fd_input_read);
    close(fd_drone_read);
    close(fd_drone_write);
    close(fd_obst_read);
    close(fd_obst_write);
    logMessage(LOG_PATH, " [WIN] main terminato");
    return 0;
}
