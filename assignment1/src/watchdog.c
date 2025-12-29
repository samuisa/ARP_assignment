#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/file.h>
#include <stdarg.h> // <--- NECESSARIO PER GESTIRE I MESSAGGI VARIABILI

#include "process_pid.h" 
#include "log.h" 

#define LOG_PATH "logs/watchdog.log"
#define TIMEOUT_US 200000
#define CYCLE_DELAY 2

typedef struct {
    pid_t pid;
    char name[32];
    volatile int alive;          
} ProcessInfo;

#define MAX_PROCESSES 20
static ProcessInfo process_map[MAX_PROCESSES];
static int process_count = 0;

// =============================================================
// FUNZIONE WRAPPER LOCALE PER LOGGARE SU FILE E CONSOLE
// =============================================================
void w_log(const char *format, ...) {
    va_list args;
    char buffer[1024]; // Buffer per contenere il messaggio formattato

    // 1. Formatta la stringa nel buffer
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 2. Stampa su CONSOLE (stdout) con timestamp semplice opzionale
    // Nota: logMessage aggiunge il timestamp nel file, qui lo aggiungiamo a video
    // se lo vuoi pulito togli la parte del time_t
    time_t t = time(NULL);
    struct tm tm_info;
    localtime_r(&t, &tm_info);
    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%H:%M:%S", &tm_info);
    
    printf("[%s] %s\n", timebuf, buffer);
    fflush(stdout); // Importante per vedere subito il messaggio

    // 3. Stampa su FILE (chiama la vecchia funzione)
    // Passiamo "%s", buffer per evitare problemi se il messaggio contiene %
    logMessage(LOG_PATH, "%s", buffer);
}

// =============================================================

void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n",WD_PID_TAG, getpid());
    w_log("[WATCHDOG] PID published securely");
}

void refresh_process_registry() {
    FILE *fp = fopen(PID_FILE_PATH, "r");
    if (!fp) return;
    
    char line[256];
    char tag[128];
    int pid_val;
    
    process_count = 0; 

    while (fgets(line, sizeof(line), fp)) {
        if (sscanf(line, "%s %d", tag, &pid_val) == 2) {
            if (strcmp(tag, WD_PID_TAG) == 0) continue;

            if (process_count < MAX_PROCESSES) {
                process_map[process_count].pid = (pid_t)pid_val;
                process_map[process_count].alive = 0; 
                
                if (strcmp(tag, DRONE_PID_TAG) == 0) strcpy(process_map[process_count].name, "DRONE");
                else if (strcmp(tag, BB_PID_TAG) == 0) strcpy(process_map[process_count].name, "BLACKBOARD");
                else if (strcmp(tag, TARGET_PID_TAG) == 0) strcpy(process_map[process_count].name, "TARGET");
                else if (strcmp(tag, OBSTACLE_PID_TAG) == 0) strcpy(process_map[process_count].name, "OBSTACLE");
                else if (strcmp(tag, INPUT_PID_TAG) == 0) strcpy(process_map[process_count].name, "INPUT");
                else strcpy(process_map[process_count].name, "UNKNOWN");
                
                process_count++;
            }
        }
    }
    fclose(fp);
}

// Handler rapido: segna solo che il processo è vivo
void pong_handler(int sig, siginfo_t *info, void *context) {
    (void)context;
    if (sig == SIGUSR2) {
        pid_t sender_pid = info->si_pid;
        for (int i = 0; i < process_count; i++) {
            if (process_map[i].pid == sender_pid) {
                process_map[i].alive = 1; 
                return;
            }
        }
    }
}

int main(int argc, char *argv[]) {

    if (argc < 2) {
        fprintf(stderr, "[BB] Error: Needed 1 file descriptors, received %d\n", argc-1);
        return 1;
    }

    int fd_bb_read  = atoi(argv[1]);


    signal(SIGUSR1, SIG_IGN); 
    remove(PID_FILE_PATH);

    // Sostituito logMessage con w_log
    w_log("[WATCHDOG] Starting... PID: %d", getpid());

    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        w_log("[WATCHDOG] Error opening PID file!");
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
    
    struct sigaction sa_pong = {0};
    sa_pong.sa_sigaction = pong_handler;
    sigemptyset(&sa_pong.sa_mask);
    sa_pong.sa_flags = SA_RESTART | SA_SIGINFO; 
    sigaction(SIGUSR2, &sa_pong, NULL);

    w_log("[WATCHDOG] Warm-up phase (4 seconds)...");
    sleep(4); 
    w_log("[WATCHDOG] Warm-up complete. Monitoring started.");

    while (1) {
        char buf[80];
        ssize_t n = read(fd_bb_read, buf, sizeof(buf)-1);

        if(n>0){
            break;
        }
        refresh_process_registry();

        if (process_count == 0) {
            sleep(1);
            continue;
        }

        // --- CICLO DI CONTROLLO SEQUENZIALE ---
        for(int i=0; i<process_count; i++) {
            
            // 1. Reset flag
            process_map[i].alive = 0;

            // 2. Invia PING
            kill(process_map[i].pid, SIGUSR1);

            // 3. Attesa attiva della risposta (Polling veloce)
            int elapsed = 0;
            int step = 5000; // Controlla ogni 5ms
            
            while (process_map[i].alive == 0 && elapsed < TIMEOUT_US) {
                usleep(step);
                elapsed += step;
            }

            // 4. Verifica immediata
            if (process_map[i].alive == 1) {
                // Successo
            } else {
                // Fallimento: Timeout
                w_log("[WATCHDOG] ALERT! Process %s [PID %d] timed out after %d ms!", 
                           process_map[i].name, process_map[i].pid, elapsed/1000);
                
                // Nota: w_log scrive già su console, quindi non serve write(STDERR...)
                w_log("[WATCHDOG] Killing system due to unresponsive process.");
                kill(0, SIGKILL); 
                exit(1);
            }
        }
        
        // Log riassuntivo
        w_log("[WATCHDOG] All %d processes checked and healthy.", process_count);
        
        sleep(CYCLE_DELAY);
    }
    logMessage(LOG_PATH, "[WD] Terminated Successfully");
    return 0;
}