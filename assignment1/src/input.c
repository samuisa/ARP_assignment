#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_common.h"

int main(int argc, char *argv[]) {
    if(argc < 2){
        fprintf(stderr, "Usage: %s <fd_out> <fd_in>\n", argv[0]);
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

    printw("Controllo drone - Premi q per uscire\n");
    refresh();

    while (1) {
        ch = getch();

        if (ch == ERR) {
            usleep(10000);
            continue;
        }

        msg[0] = (char)ch;
        msg[1] = '\0';

        write(fd_out, msg, 2);

        if (ch == 'q')
            break;
    }
    endwin();
    close(fd_out);
    return 0;
}
