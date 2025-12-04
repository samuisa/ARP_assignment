#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_common.h" // Assumiamo che questo file esista

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
    // Coordinate di partenza per la griglia
    int start_y = 6;
    int col_1 = 15;
    int col_2 = 22;
    int col_3 = 29;

    // --- Intestazione ---
    mvprintw(0, 0, "=== Drone Legend Control ===");
    mvprintw(2, 0, "Press '%c' to exit | Press the bottoms below to control the drone", KEY_QUIT);

    // --- Legenda Grafica ---
    
    // Titolo
    mvprintw(4, 0, "------------------ LEGEND ------------------");

    // Riga ALTI (High) - W, E, R
    mvprintw(start_y, col_1, "[ w ]");
    mvprintw(start_y, col_2, "[ e ]");
    mvprintw(start_y, col_3, "[ r ]");

    // Riga ORIZZONTALI / STOP - S, D, F
    mvprintw(start_y + 2, col_1, "[ s ]");
    mvprintw(start_y + 2, col_2, "[ d ]");
    mvprintw(start_y + 2, col_3, "[ f ]");

    // Riga BASSI (Low) - X, C, V
    mvprintw(start_y + 4, col_1, "[ x ]");
    mvprintw(start_y + 4, col_2, "[ c ]");
    mvprintw(start_y + 4, col_3, "[ v ]");

    mvprintw(start_y + 6, 0, "--------------------------------------------");
    
    // Area per il feedback in tempo reale
    mvprintw(start_y + 8, 0, "Feedback: ");
}


int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <fd_out> <fd_in>\n", argv[0]);
        return 1;
    }

    int fd_out = atoi(argv[1]);
    int ch;
    char msg[2];

    // Inizializzazione Ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    // Disegna la legenda iniziale
    draw_legend();
    refresh();

    // Loop principale di lettura input
    while (1) {
        ch = getch();

        if (ch == ERR) {
            usleep(10000); // 10ms di attesa
            continue;
        }

        // Prepara il messaggio da inviare
        msg[0] = (char)ch;
        msg[1] = '\0';
        
        // Scrive il carattere sul file descriptor di output
        write(fd_out, msg, 2); 

        // Aggiorna il feedback sullo schermo
        mvprintw(14, 0, "Feedback: '%c'", ch);
        refresh();

        if (ch == KEY_QUIT)
            break; // Esci se 'q' è premuto
    }
    
    // Pulizia e chiusura
    endwin();
    close(fd_out);
    return 0;
}