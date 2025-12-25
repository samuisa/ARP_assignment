#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include "process_pid.h" 
#include "log.h" 

#define LOG_PATH "logs/watchdog.log"
#define TIMEOUT_US 200000 // 200ms di attesa massima per OGNI processo
#define CYCLE_DELAY 2     // Pausa tra un ciclo di controllo completo e l'altro

typedef struct {
    pid_t pid;
    char name[32];
    // 'volatile' è essenziale perché viene modificato dal signal handler
    volatile int alive;          
} ProcessInfo;

#define MAX_PROCESSES 20
static ProcessInfo process_map[MAX_PROCESSES];
static int process_count = 0;

void publish_watchdog_pid() {
    FILE *f = fopen(PID_FILE_PATH, "w"); 
    if (!f) { perror("fopen"); exit(1); }
    fprintf(f, "%s %d\n", WD_PID_TAG, getpid());
    fclose(f);
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

int main() {
    signal(SIGUSR1, SIG_IGN); 
    remove(PID_FILE_PATH);

    logMessage(LOG_PATH, "[WATCHDOG] Starting... PID: %d", getpid());
    publish_watchdog_pid();

    struct sigaction sa_pong = {0};
    sa_pong.sa_sigaction = pong_handler;
    sigemptyset(&sa_pong.sa_mask);
    sa_pong.sa_flags = SA_RESTART | SA_SIGINFO; 
    sigaction(SIGUSR2, &sa_pong, NULL);

    logMessage(LOG_PATH, "[WATCHDOG] Warm-up phase (4 seconds)...");
    sleep(4); 
    logMessage(LOG_PATH, "[WATCHDOG] Warm-up complete. Monitoring started.");

    while (1) {
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
            // Aspettiamo fino a TIMEOUT_US microsecondi (es. 200ms)
            int elapsed = 0;
            int step = 5000; // Controlla ogni 5ms
            
            while (process_map[i].alive == 0 && elapsed < TIMEOUT_US) {
                usleep(step);
                elapsed += step;
            }

            // 4. Verifica immediata
            if (process_map[i].alive == 1) {
                // Successo: il processo ha risposto. Passiamo subito al prossimo!
                // (Opzionale: logga solo se serve debug intenso, altrimenti intasa)
                // logMessage(LOG_PATH, "[OK] %s is alive", process_map[i].name);
            } else {
                // Fallimento: Timeout scaduto per questo specifico processo
                logMessage(LOG_PATH, "[WATCHDOG] ALERT! Process %s [PID %d] timed out after %d ms!", 
                           process_map[i].name, process_map[i].pid, elapsed/1000);
                
                const char *msg = "[WATCHDOG] Killing system due to unresponsive process.\n";
                write(STDERR_FILENO, msg, strlen(msg));
                kill(0, SIGKILL); 
                exit(1);
            }
        }
        
        // Se siamo qui, tutti i processi sono vivi.
        // Log riassuntivo (più pulito)
        logMessage(LOG_PATH, "[WATCHDOG] All %d processes checked and healthy.", process_count);

        // Attesa prima del prossimo giro di controlli
        sleep(CYCLE_DELAY);
    }
}