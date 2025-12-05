#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_common.h" // Assumiamo che questo file esista
#include "log.h"        // Include per il logging

// Definizione della struttura dei comandi per una migliore leggibilità
#define KEY_UP_LEFT 'w'
#define KEY_UP_CENTER 'e'
#define KEY_UP_RIGHT 'r'
#define KEY_MID_LEFT 's'
#define KEY_MID_CENTER 'd'
#define KEY_MID_RIGHT 'f'
#define KEY_DOWN_LEFT 'x'
#define KEY_DOWN_CENTER 'c'
#define KEY_DOWN_RIGHT 'v'
#define KEY_QUIT 'q'

void draw_legend() {
    int start_y = 6;
    int col_1 = 15;
    int col_2 = 22;
    int col_3 = 29;

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

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <fd_out>\n", argv[0]);
        return 1;
    }

    int fd_out = atoi(argv[1]);
    int ch;
    char msg[2];

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    logMessage(LOG_PATH, "[CTRL] Main started, fd_out=%d", fd_out);

    draw_legend();

    while (1) {
        ch = getch();

        if (ch == ERR) {
            usleep(10000);
            continue;
        }

        msg[0] = (char)ch;
        msg[1] = '\0';

        write(fd_out, msg, 2);
        logMessage(LOG_PATH, "[CTRL] Input captured: '%c' (ASCII %d), sent to fd_out=%d", ch, ch, fd_out);

        mvprintw(14, 0, "Feedback: '%c'  ", ch);
        refresh();

        if (ch == KEY_QUIT) {
            logMessage(LOG_PATH, "[CTRL] Quit command received, exiting main loop");
            break;
        }
    }

    endwin();
    close(fd_out);

    logMessage(LOG_PATH, "[CTRL] Main terminated, fd_out closed");
    return 0;
}
