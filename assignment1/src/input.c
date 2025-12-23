#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include "app_common.h" // Definizione Message, MSG_TYPE_PID, ecc.
#include "log.h"        // logMessage

#define KEY_QUIT 'q'

void draw_legend() {
    int start_y = 6;
    int col_1 = 15, col_2 = 22, col_3 = 29;

    mvprintw(0, 0, "=== Drone Legend Control ===");
    mvprintw(2, 0, "Press '%c' to exit | Press the buttons below to control the drone", KEY_QUIT);

    mvprintw(4, 0, "------------------ LEGEND ------------------");

    mvprintw(start_y, col_1, "[ w ]");
    mvprintw(start_y, col_2, "[ e ]");
    mvprintw(start_y, col_3, "[ r ]");

    mvprintw(start_y + 2, col_1, "[ s ]");
    mvprintw(start_y + 2, col_2, "[ d ]");
    mvprintw(start_y + 2, col_3, "[ f ]");

    mvprintw(start_y + 4, col_1, "[ x ]");
    mvprintw(start_y + 4, col_2, "[ c ]");
    mvprintw(start_y + 4, col_3, "[ v ]");

    mvprintw(start_y + 6, 0, "--------------------------------------------");
    mvprintw(start_y + 8, 0, "Feedback: ");
    refresh();
}

void send_pid(int fd_wd_write){
    pid_t pid = getpid();
    Message msg;
    msg.type = MSG_TYPE_PID;
    snprintf(msg.data, sizeof(msg.data), "%d", pid);
    if(write(fd_wd_write, &msg, sizeof(msg)) < 0) {
        perror("[CTRL] write pid to watchdog");
        exit(1);
    }
    logMessage(LOG_PATH, "[CTRL] PID sent to watchdog: %d", pid);
}

pid_t receive_watchdog_pid(int fd_wd_read){
    Message msg;
    ssize_t n;
    pid_t pid_wd = -1;

    do {
        n = read(fd_wd_read, &msg, sizeof(msg));
    } while(n < 0 && errno == EINTR);

    if(n <= 0){
        perror("[DRONE] read watchdog PID");
        exit(1);
    }

    if(msg.type == MSG_TYPE_PID){
        sscanf(msg.data, "%d", &pid_wd);
        logMessage(LOG_PATH, "[DRONE] Watchdog PID received: %d", pid_wd);
    } else {
        logMessage(LOG_PATH, "[DRONE] Unexpected message type from watchdog: %d", msg.type);
    }

    return pid_wd;
}

int main(int argc, char *argv[]) {
    if(argc < 4) {
        fprintf(stderr, "Usage: %s <fd_out> <fd_watchdog_read> <fd_watchdog_write>\n", argv[0]);
        return 1;
    }

    int fd_out      = atoi(argv[1]);
    int fd_wd_read  = atoi(argv[2]);
    int fd_wd_write = atoi(argv[3]);

    send_pid(fd_wd_write);

    pid_t pid_watchdog = receive_watchdog_pid(fd_wd_read);
    
    int ch;
    char msg_buf[2];

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    logMessage(LOG_PATH, "[CTRL] Main started, fd_out=%d", fd_out);

    draw_legend();

    while(1) {
        ch = getch();

        if(ch == ERR) {
            usleep(10000);
            continue;
        }

        msg_buf[0] = (char)ch;
        msg_buf[1] = '\0';

        // invia al fd_out
        if(write(fd_out, msg_buf, 2) < 0) {
            perror("[CTRL] write to fd_out");
            break;
        }

        logMessage(LOG_PATH, "[CTRL] Input captured: '%c' (ASCII %d) sent to fd_out=%d",
                   ch, ch, fd_out);

        mvprintw(14, 0, "Feedback: '%c'  ", ch);
        refresh();

        if(ch == KEY_QUIT) {
            logMessage(LOG_PATH, "[CTRL] Quit command received");
            break;
        }
    }

    endwin();
    close(fd_out);
    close(fd_wd_read);
    close(fd_wd_write);

    logMessage(LOG_PATH, "[CTRL] Main terminated, pipes closed");
    return 0;
}
