#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "log.h"
#include "app_common.h"
#include "process_pid.h"

/* ========================================================= */

void ensureLogsDir(void) {
    struct stat st;
    if (stat("logs", &st) == -1) {
        if (mkdir("logs", 0700) == -1) {
            perror("mkdir logs");
            exit(1);
        }
    }
    FILE *logf = fopen(LOG_PATH, "w");
    if (!logf) {
        perror("fopen log file");
        exit(1);
    }
    fclose(logf);
}

/* ========================================================= */

int main(void) {

    ensureLogsDir();
    logMessage(LOG_PATH, "[MAIN] PROGRAM STARTED");

    /* ================= PIPE APPLICATIVE ================= */

    int pipe_input_blackboard[2];
    int pipe_blackboard_drone[2];
    int pipe_drone_blackboard[2];
    int pipe_blackboard_obstacle[2];
    int pipe_obstacle_blackboard[2];
    int pipe_blackboard_target[2];
    int pipe_target_blackboard[2];

    /* NOTA: Le pipe del Watchdog sono state rimosse.
       La comunicazione avverrà tramite segnali e file PID.
    */

    if (pipe(pipe_input_blackboard) == -1 ||
        pipe(pipe_blackboard_drone) == -1 ||
        pipe(pipe_drone_blackboard) == -1 ||
        pipe(pipe_blackboard_obstacle) == -1 ||
        pipe(pipe_obstacle_blackboard) == -1 ||
        pipe(pipe_blackboard_target) == -1 ||
        pipe(pipe_target_blackboard) == -1) {

        perror("pipe");
        logMessage(LOG_PATH, "[MAIN] ERROR creating application pipes");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Application pipes created successfully");

    FILE *fp = fopen(PID_FILE_PATH, "w");
    if (fp) {
        fclose(fp);
    }

    /* ================= PROCESSO INPUT ================= */

    pid_t pid_input = fork();
    if (pid_input == 0) {
        // Chiudo lati inutilizzati delle pipe applicative
        close(pipe_input_blackboard[0]); // Input scrive solo qui
        
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_out[16];
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_input_blackboard[1]);

        // Lancio input senza pipe watchdog
        execlp("konsole", "konsole", "-e",
               "./exec/input", fd_out, NULL);

        perror("exec input");
        exit(1);
    }

    /* ================= PROCESSO OBSTACLE ================= */

    pid_t pid_obst = fork();
    if (pid_obst == 0) {
        // Setup pipe applicative
        close(pipe_blackboard_obstacle[1]); // Legge da BB
        close(pipe_obstacle_blackboard[0]); // Scrive a BB

        // Chiudo altre pipe
        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_in[16], fd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_obstacle[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_obstacle_blackboard[1]);

        // Lancio obstacle senza pipe watchdog
        execlp("./exec/obstacle", "./exec/obstacle",
               fd_in, fd_out, NULL);

        perror("exec obstacle");
        exit(1);
    }

    /* ================= PROCESSO TARGET ================= */

    pid_t pid_target = fork();
    if (pid_target == 0) {
        // Setup pipe applicative
        close(pipe_blackboard_target[1]); // Legge da BB
        close(pipe_target_blackboard[0]); // Scrive a BB

        // Chiudo altre pipe
        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);

        char fd_in[16], fd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_target[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_target_blackboard[1]);

        // Lancio target senza pipe watchdog
        execlp("./exec/target", "./exec/target",
               fd_in, fd_out, NULL);

        perror("exec target");
        exit(1);
    }

    /* ================= PROCESSO BLACKBOARD ================= */

    pid_t pid_blackboard = fork();
    if (pid_blackboard == 0) {
        // Setup pipe applicative
        close(pipe_input_blackboard[1]);
        close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_drone[0]);
        close(pipe_blackboard_obstacle[0]);
        close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]);
        close(pipe_target_blackboard[1]);

        char fd_in_input[16], fd_in_drone[16];
        char fd_out_drone[16], fd_out_obst[16];
        char fd_in_obst[16], fd_out_target[16], fd_in_target[16];

        snprintf(fd_in_input,  sizeof(fd_in_input),  "%d", pipe_input_blackboard[0]);
        snprintf(fd_in_drone,  sizeof(fd_in_drone),  "%d", pipe_drone_blackboard[0]);
        snprintf(fd_out_drone, sizeof(fd_out_drone), "%d", pipe_blackboard_drone[1]);
        snprintf(fd_out_obst,  sizeof(fd_out_obst),  "%d", pipe_blackboard_obstacle[1]);
        snprintf(fd_in_obst,   sizeof(fd_in_obst),   "%d", pipe_obstacle_blackboard[0]);
        snprintf(fd_out_target,sizeof(fd_out_target),"%d", pipe_blackboard_target[1]);
        snprintf(fd_in_target, sizeof(fd_in_target), "%d", pipe_target_blackboard[0]);

        // Lancio blackboard senza pipe watchdog
        execlp("konsole", "konsole", "-e",
               "./exec/blackboard",
               fd_in_input, fd_in_drone,
               fd_out_drone, fd_out_obst,
               fd_in_obst, fd_out_target,
               fd_in_target, NULL);

        perror("exec blackboard");
        exit(1);
    }

    /* ================= PROCESSO DRONE ================= */

    pid_t pid_drone = fork();
    if (pid_drone == 0) {
        // Setup pipe applicative
        close(pipe_blackboard_drone[1]); // Legge da BB
        close(pipe_drone_blackboard[0]); // Scrive a BB

        // Chiudo altre pipe
        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_in[16], fd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_drone[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_drone_blackboard[1]);

        // Lancio drone senza pipe watchdog
        execlp("./exec/drone", "./exec/drone",
               fd_in, fd_out, NULL);

        perror("exec drone");
        exit(1);
    }

    /* ================= PROCESSO WATCHDOG ================= */

    pid_t pid_watchdog = fork();
    if (pid_watchdog == 0) {
        
        // Il Watchdog non usa nessuna pipe applicativa
        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        // Lancio watchdog senza argomenti (si sincronizza via file PID)
        execlp("konsole", "konsole", "-e",
               "./exec/watchdog", NULL);

        perror("exec watchdog");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Watchdog started (pid=%d)", pid_watchdog);

    /* ================= PARENT ================= */

    // Chiudo tutte le pipe nel processo padre
    close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
    close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
    close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
    close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
    close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
    close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
    close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

    logMessage(LOG_PATH,
        "[MAIN] All processes running (input=%d drone=%d bb=%d obst=%d targ=%d)",
        pid_input, pid_drone, pid_blackboard, pid_obst, pid_target);

    while (wait(NULL) > 0);

    logMessage(LOG_PATH, "[MAIN] PROGRAM EXIT");
    return 0;
}