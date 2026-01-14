/* ======================================================================================
 * FILE: input.c
 * ====================================================================================== */
#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/file.h>

#include "process_pid.h"
#include "app_common.h"
#include "log.h"       

#define KEY_QUIT 'q'

static volatile pid_t watchdog_pid = -1;

void draw_legend() {
    int start_y = 6;
    int col_1 = 15, col_2 = 22, col_3 = 29;
    mvprintw(0, 0, "=== Drone Legend Control ===");
    mvprintw(2, 0, "Press '%c' to exit | Press the buttons below to control the drone", KEY_QUIT);
    mvprintw(4, 0, "------------------ LEGEND ------------------");
    mvprintw(start_y, col_1, "[ w ]"); mvprintw(start_y, col_2, "[ e ]"); mvprintw(start_y, col_3, "[ r ]");
    mvprintw(start_y + 2, col_1, "[ s ]"); mvprintw(start_y + 2, col_2, "[ d ]"); mvprintw(start_y + 2, col_3, "[ f ]");
    mvprintw(start_y + 4, col_1, "[ x ]"); mvprintw(start_y + 4, col_2, "[ c ]"); mvprintw(start_y + 4, col_3, "[ v ]");
    mvprintw(start_y + 6, 0, "--------------------------------------------");
    mvprintw(start_y + 8, 0, "Feedback: ");
    refresh();
}

void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", INPUT_PID_TAG, getpid());
}

void wait_for_watchdog_pid() {
    FILE *fp;
    char line[256], tag[128];
    int pid_temp;
    bool wd_found = false;
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
}

void watchdog_ping_handler(int signo) {
    (void)signo; 
    if(watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

int main(int argc, char *argv[]) {
    if(argc < 3) return 1;

    int fd_out = atoi(argv[1]);
    int mode = atoi(argv[2]);

    if(mode == MODE_STANDALONE){
        // 1. PUBBLICA IL PID SUBITO
        FILE *fp_pid = fopen(PID_FILE_PATH, "a");
        if (!fp_pid) {
            perror("fopen PID file");
            exit(1);
        }
        int fd_pid = fileno(fp_pid);
        flock(fd_pid, LOCK_EX); 
        publish_my_pid(fp_pid);
        fflush(fp_pid);
        flock(fd_pid, LOCK_UN);
        fclose(fp_pid);

        // 2. SETUP SEGNALI
        struct sigaction sa;
        sa.sa_handler = watchdog_ping_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART;
        sigaction(SIGUSR1, &sa, NULL);

        // 3. ASPETTA IL WATCHDOG
        wait_for_watchdog_pid();
    }
    
    int ch;
    char msg_buf[2];

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    draw_legend();

    while(1) {
        ch = getch();

        if(ch == ERR) {
            usleep(10000); // 10ms sleep to save CPU
            continue;
        }

        msg_buf[0] = (char)ch;
        msg_buf[1] = '\0';

        if(write(fd_out, msg_buf, 2) < 0) break;
        mvprintw(14, 0, "Feedback: '%c'  ", ch);
        refresh();

        if(ch == KEY_QUIT) break;
    }

    endwin();
    close(fd_out);
    return 0;
}