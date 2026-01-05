/* ======================================================================================
 * SECTION 1: INCLUDES AND UTILITIES
 * Standard headers and helper functions for directory management.
 * ====================================================================================== */
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

/* ======================================================================================
 * SECTION 2: MAIN PROCESS ORCHESTRATION
 * Creates pipes and forks all child processes (Input, Obstacle, Target, BB, Drone, Watchdog).
 * ====================================================================================== */
int main(void) {

    ensureLogsDir();
    logMessage(LOG_PATH, "[MAIN] PROGRAM STARTED");

    // --- 1. SELEZIONE MODALITA' ---
    int mode = MODE_STANDALONE;
    printf("======================================\n");
    printf(" SELECT MODE:\n");
    printf(" 0: MODE_STANDALONE (Local)\n");
    printf(" 1: Server\n");
    printf(" 2: Client\n");
    printf("======================================\n");
    printf("> ");
    if (scanf("%d", &mode) != 1) mode = 0;

    logMessage(LOG_PATH, "[MAIN] Starting in MODE: %d", mode);

    char arg_mode[4];
    snprintf(arg_mode, sizeof(arg_mode), "%d", mode);

    /* --- SUB-SECTION: PIPE CREATION --- */
    /* Establishing communication channels between processes */

    int pipe_input_blackboard[2];
    int pipe_blackboard_drone[2];
    int pipe_drone_blackboard[2];
    int pipe_blackboard_obstacle[2];
    int pipe_obstacle_blackboard[2];
    int pipe_blackboard_target[2];
    int pipe_target_blackboard[2];
    int pipe_blackboard_watchdog[2];

    /* Note: Watchdog uses signals/PID files, not application pipes (except for BB trigger if needed) */

    if (pipe(pipe_input_blackboard) == -1 ||
        pipe(pipe_blackboard_drone) == -1 ||
        pipe(pipe_drone_blackboard) == -1 ||
        pipe(pipe_blackboard_obstacle) == -1 ||
        pipe(pipe_obstacle_blackboard) == -1 ||
        pipe(pipe_blackboard_target) == -1 ||
        pipe(pipe_target_blackboard) == -1 || 
        pipe(pipe_blackboard_watchdog) == -1) {

        perror("pipe");
        logMessage(LOG_PATH, "[MAIN] ERROR creating application pipes");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Application pipes created successfully");

    // Reset PID file
    FILE *fp = fopen(PID_FILE_PATH, "w");
    if (fp) {
        fclose(fp);
    }

    /* --- SUB-SECTION: FORKING PROCESSES --- */

    /* 1. INPUT PROCESS (Runs in Konsole) */
    pid_t pid_input = fork();
    if (pid_input == 0) {
        // Close unused pipe ends
        close(pipe_input_blackboard[0]); 
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);
        close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);

        char fd_out[16];
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_input_blackboard[1]);

        execlp("konsole", "konsole", "-e", "./exec/input", fd_out, arg_mode, NULL);
        perror("exec input");
        exit(1);
    }

        /* 4. BLACKBOARD PROCESS (Runs in Konsole) */
    pid_t pid_blackboard = fork();
    if (pid_blackboard == 0) {
        close(pipe_input_blackboard[1]);
        close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_drone[0]);
        close(pipe_blackboard_obstacle[0]);
        close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]);
        close(pipe_target_blackboard[1]);
        close(pipe_blackboard_watchdog[0]);

        char fd_in_input[16], fd_in_drone[16];
        char fd_out_drone[16], fd_out_obst[16];
        char fd_in_obst[16], fd_out_target[16], fd_in_target[16], fd_out_wd[16];

        snprintf(fd_in_input,  sizeof(fd_in_input),  "%d", pipe_input_blackboard[0]);
        snprintf(fd_in_drone,  sizeof(fd_in_drone),  "%d", pipe_drone_blackboard[0]);
        snprintf(fd_out_drone, sizeof(fd_out_drone), "%d", pipe_blackboard_drone[1]);
        snprintf(fd_out_obst,  sizeof(fd_out_obst),  "%d", pipe_blackboard_obstacle[1]);
        snprintf(fd_in_obst,   sizeof(fd_in_obst),   "%d", pipe_obstacle_blackboard[0]);
        snprintf(fd_out_target,sizeof(fd_out_target),"%d", pipe_blackboard_target[1]);
        snprintf(fd_in_target, sizeof(fd_in_target), "%d", pipe_target_blackboard[0]);
        snprintf(fd_out_wd, sizeof(fd_out_wd), "%d", pipe_blackboard_watchdog[1]);

        execlp("konsole", "konsole", "-e",
               "./exec/blackboard",
               fd_in_input, fd_in_drone,
               fd_out_drone, fd_out_obst,
               fd_in_obst, fd_out_target,
               fd_in_target, fd_out_wd,
               arg_mode, NULL);

        perror("exec blackboard");
        exit(1);
    }

    /* 5. DRONE PROCESS */
    pid_t pid_drone = fork();
    if (pid_drone == 0) {
        close(pipe_blackboard_drone[1]); 
        close(pipe_drone_blackboard[0]); 

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);
        close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);

        char fd_in[16], fd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_drone[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_drone_blackboard[1]);

        execlp("./exec/drone", "./exec/drone", fd_in, fd_out, arg_mode, NULL);
        perror("exec drone");
        exit(1);
    }

    pid_t pid_obst = -1, pid_target = -1, pid_watchdog = -1;

    if (mode == MODE_STANDALONE){

        /* 2. OBSTACLE PROCESS */
        pid_obst = fork();
        if (pid_obst == 0) {
            close(pipe_blackboard_obstacle[1]); 
            close(pipe_obstacle_blackboard[0]); 

            close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
            close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
            close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
            close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
            close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);
            close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);

            char fd_in[16], fd_out[16];
            snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_obstacle[0]);
            snprintf(fd_out, sizeof(fd_out), "%d", pipe_obstacle_blackboard[1]);

            execlp("./exec/obstacle", "./exec/obstacle", fd_in, fd_out, NULL);
            perror("exec obstacle");
            exit(1);
        }

        /* 3. TARGET PROCESS */
        pid_target = fork();
        if (pid_target == 0) {
            close(pipe_blackboard_target[1]); 
            close(pipe_target_blackboard[0]); 

            close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
            close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
            close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
            close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
            close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
            close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);

            char fd_in[16], fd_out[16];
            snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_target[0]);
            snprintf(fd_out, sizeof(fd_out), "%d", pipe_target_blackboard[1]);

            execlp("./exec/target", "./exec/target", fd_in, fd_out, NULL);
            perror("exec target");
            exit(1);
        }

        /* 6. WATCHDOG PROCESS (Runs in Konsole) */
        pid_watchdog = fork();
        if (pid_watchdog == 0) {
            // Close all application pipes except the one reading from BB (if used)
            close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
            close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
            close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
            close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
            close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
            close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
            close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);
            close(pipe_blackboard_watchdog[1]);

            char fd_in_bb[16];
            snprintf(fd_in_bb, sizeof(fd_in_bb), "%d", pipe_blackboard_watchdog[0]);

            execlp("konsole", "konsole", "-e", "./exec/watchdog", fd_in_bb, NULL);
            perror("exec watchdog");
            exit(1);
        }

        logMessage(LOG_PATH, "[MAIN] Watchdog started (pid=%d)", pid_watchdog);
    }

    /* --- SUB-SECTION: PARENT CLEANUP AND WAIT --- */
    
    // Close all pipes in the parent process (critical to avoid zombie pipes)
    close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
    close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
    close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
    close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
    close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
    close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
    close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);
    close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);

    logMessage(LOG_PATH,
        "[MAIN] All processes running (input=%d drone=%d bb=%d obst=%d targ=%d)",
        pid_input, pid_drone, pid_blackboard, pid_obst, pid_target);

    // Wait for children to exit
    while (wait(NULL) > 0);

    logMessage(LOG_PATH, "[MAIN] PROGRAM EXIT");
    return 0;
}