#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/file.h>

#include "app_common.h"
#include "log.h"
#include "process_pid.h"

// Globals
static Point *obstacles = NULL;
static int num_obstacles = 0;
static volatile pid_t watchdog_pid = -1; // Volatile perché usata nel signal handler

typedef enum { STATE_INIT, STATE_WAITING, STATE_GENERATING } ProcessState;
static volatile sig_atomic_t current_state = STATE_INIT;

// Funzione per dire al mondo che esistiamo
void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", TARGET_PID_TAG, getpid());
    logMessage(LOG_PATH, "[TARG] PID published securely");
}

void wait_for_watchdog_pid() {
    FILE *fp;
    char line[256], tag[128];
    int pid_temp;
    bool wd_found = false;

    logMessage(LOG_PATH, "[TARG] Waiting for Watchdog...");

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
    logMessage(LOG_PATH, "[TARG] Watchdog found (PID %d)", watchdog_pid);
}

// Handler per il segnale SIGUSR1 inviato dal Watchdog
void watchdog_ping_handler(int signo) {
    (void)signo;
    if(watchdog_pid > 0) {
        kill(watchdog_pid, SIGUSR2);
    }
}

Point* generate_targets(int width, int height, Point* obstacles, int num_obstacles, int* num_out) {
    int total_cells = (width - 2) * (height - 2);
    // Calcola quanti target generare
    int count = (int) round(PERC_TARG * total_cells);
    if (count < 1) count = 1;

    Point* arr = malloc(sizeof(Point) * count);
    if (!arr) exit(1);
    
    // Nota: srand rimosso da qui e messo nel main

    for (int i = 0; i < count; i++) {
        int valid;
        do {
            valid = 1;
            // Genera coordinate (evitando i bordi)
            arr[i].x = rand() % (width - 2) + 1;
            arr[i].y = rand() % (height - 2) + 1;
            for (int j = 0; j < i; j++) {
                if (arr[i].x == arr[j].x && arr[i].y == arr[j].y) { valid = 0; break; }
            }
            for (int j = 0; j < num_obstacles && valid; j++) {
                if (arr[i].x == obstacles[j].x && arr[i].y == obstacles[j].y) { valid = 0; break; }
            }
        } while (!valid);
    }
    logMessage(LOG_PATH, "[TARG] Generated %d targets", count);
    for(int i=0; i<count; i++){
        logMessage(LOG_PATH, "[TARG] targets %d position: %d %d", i, arr[i].x, arr[i].y);
    }
    *num_out = count;
    return arr;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Uso: %s <fd_in> <fd_out>\n", argv[0]);
        return 1;
    }

    // Inizializza il generatore di numeri casuali UNA SOLA VOLTA
    srand(time(NULL));

    int fd_in  = atoi(argv[1]);
    int fd_out = atoi(argv[2]);
    int win_width = 0, win_height = 0;

    logMessage(LOG_PATH, "[TARG] Started with PID: %d", getpid());

    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    // PASSO A: Aspetta il watchdog (SENZA LOCK per evitare deadlock)
    wait_for_watchdog_pid();

    // PASSO B: Scrivi il proprio PID (CON LOCK come richiesto)
    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        logMessage(LOG_PATH, "[TARG] Error opening PID file!");
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

    while (1) {
        current_state = STATE_WAITING;
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_in, &set);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        int ret = select(fd_in + 1, &set, NULL, NULL, &tv);
        
        // Gestione errori select
        if (ret < 0) {
            if (errno == EINTR) {
                continue; 
            }
            logMessage(LOG_PATH, "[TARG] ERROR select(): %s", strerror(errno));
            break; 
        }

        // Se c'è qualcosa da leggere
        if (FD_ISSET(fd_in, &set)) {
            Message msg;
            ssize_t n = read(fd_in, &msg, sizeof(msg));
            if (n <= 0) {
                logMessage(LOG_PATH, "[TARG] Pipe closed, exiting.");
                break;
            }

            if (msg.type == MSG_TYPE_SIZE) {
                sscanf(msg.data, "%d %d", &win_width, &win_height);
            }
            else if (msg.type == MSG_TYPE_OBSTACLES) {
                current_state = STATE_GENERATING;
                int count = 0;
                sscanf(msg.data, "%d", &count);
                
                // Pulizia vecchi ostacoli
                free(obstacles);
                obstacles = NULL;
                num_obstacles = 0;

                // Lettura nuovi ostacoli (se ce ne sono)
                if (count > 0) {
                    obstacles = malloc(sizeof(Point) * count);
                    if (obstacles) {
                        read(fd_in, obstacles, sizeof(Point) * count);
                        num_obstacles = count;
                    }
                }

                // Generazione Target
                if (win_width > 0 && win_height > 0) {
                    int num_targ = 0;
                    Point* arr = generate_targets(win_width, win_height, obstacles, num_obstacles, &num_targ);
                    
                    // Invio numero target
                    Message out_msg;
                    out_msg.type = MSG_TYPE_TARGETS;
                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targ);
                    write(fd_out, &out_msg, sizeof(out_msg));
                    
                    // Invio array target
                    write(fd_out, arr, sizeof(Point) * num_targ);
                    free(arr);
                }
            }
        }
    }

    free(obstacles);
    close(fd_in);
    close(fd_out);
    return 0;
}