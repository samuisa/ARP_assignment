#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include "process_pid.h"
#include "app_common.h" // Definizione Message, MSG_TYPE_PID, ecc.
#include "log.h"        // logMessage

#define KEY_QUIT 'q'

static volatile pid_t watchdog_pid = -1;

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

void publish_my_pid() {
    FILE *fp = fopen(PID_FILE_PATH, "a");
    if (!fp) exit(1);
    fprintf(fp, "%s %d\n", INPUT_PID_TAG, getpid());
    fclose(fp);
}

void wait_for_watchdog() {
    FILE *fp;
    char line[256], tag[128];
    int pid;
    bool found = false;
    
    printf("[INPUT] In attesa del Watchdog...\n");
    fflush(stdout);

    while(!found) {
        fp = fopen(PID_FILE_PATH, "r");
        if(fp) {
            while(fgets(line, sizeof(line), fp)) {
                if(sscanf(line, "%s %d", tag, &pid) == 2) {
                    if(strcmp(tag, WD_PID_TAG) == 0) {
                        watchdog_pid = pid; 
                        found = true; 
                        break;
                    }
                }
            }
            fclose(fp);
        }
        if(!found) usleep(100000);
    }
    printf("[INPUT] Watchdog trovato! PID: %d\n", watchdog_pid);
    printf("[INPUT] Controlli attivi (premi i tasti, non vedrai nulla a schermo)\n");
    fflush(stdout);
}

void watchdog_ping_handler(int signo) {
    (void)signo; // Zittisce il warning del compilatore
    if(watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

int main(int argc, char *argv[]) {
    if(argc < 2) {
        fprintf(stderr, "Usage: %s <fd_out> <fd_watchdog_read> <fd_watchdog_write>\n", argv[0]);
        return 1;
    }

    int fd_out = atoi(argv[1]);

    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    wait_for_watchdog();
    publish_my_pid();
    
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
            perror("[INPUT] write to fd_out");
            break;
        }

        logMessage(LOG_PATH, "[INPUT] Input captured: '%c' (ASCII %d) sent to fd_out=%d",
                   ch, ch, fd_out);

        mvprintw(14, 0, "Feedback: '%c'  ", ch);
        refresh();

        if(ch == KEY_QUIT) {
            logMessage(LOG_PATH, "[INPUT] Quit command received");
            break;
        }
    }

    endwin();
    close(fd_out);

    logMessage(LOG_PATH, "[CTRL] Main terminated, pipes closed");
    return 0;
}