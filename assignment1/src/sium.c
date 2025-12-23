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

    /* ================= PIPE WATCHDOG ================= */

    int pipe_input_watchdog[2];
    int pipe_drone_watchdog[2];
    int pipe_blackboard_watchdog[2];
    int pipe_obstacle_watchdog[2];
    int pipe_target_watchdog[2];

    int pipe_watchdog_input[2];
    int pipe_watchdog_drone[2];
    int pipe_watchdog_blackboard[2];
    int pipe_watchdog_obstacle[2];
    int pipe_watchdog_target[2];

    if (pipe(pipe_input_blackboard) == -1 ||
        pipe(pipe_blackboard_drone) == -1 ||
        pipe(pipe_drone_blackboard) == -1 ||
        pipe(pipe_blackboard_obstacle) == -1 ||
        pipe(pipe_obstacle_blackboard) == -1 ||
        pipe(pipe_blackboard_target) == -1 ||
        pipe(pipe_target_blackboard) == -1 ||

        pipe(pipe_input_watchdog) == -1 ||
        pipe(pipe_drone_watchdog) == -1 ||
        pipe(pipe_blackboard_watchdog) == -1 ||
        pipe(pipe_obstacle_watchdog) == -1 ||
        pipe(pipe_target_watchdog) == -1 ||
        
        pipe(pipe_watchdog_input) == -1 ||
        pipe(pipe_watchdog_drone) == -1 ||
        pipe(pipe_watchdog_blackboard) == -1 ||
        pipe(pipe_watchdog_obstacle) == -1 ||
        pipe(pipe_watchdog_target) == -1) {

        perror("pipe");
        logMessage(LOG_PATH, "[MAIN] ERROR creating pipes");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Pipes created successfully");

    /* ================= PROCESSO INPUT ================= */

    pid_t pid_input = fork();
    if (pid_input == 0) {

        close(pipe_input_blackboard[0]);
        close(pipe_input_watchdog[0]);
        close(pipe_watchdog_input[1]);


        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        close(pipe_drone_watchdog[0]); close(pipe_drone_watchdog[1]);
        close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);
        close(pipe_obstacle_watchdog[0]); close(pipe_obstacle_watchdog[1]);
        close(pipe_target_watchdog[0]); close(pipe_target_watchdog[1]);

        close(pipe_watchdog_drone[0]); close(pipe_watchdog_drone[1]);
        close(pipe_watchdog_blackboard[0]); close(pipe_watchdog_blackboard[1]);
        close(pipe_watchdog_obstacle[0]); close(pipe_watchdog_obstacle[1]);
        close(pipe_watchdog_target[0]); close(pipe_watchdog_target[1]);

        char fd_out[16], fd_wd_out[16], fd_wd_in[16];
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_input_blackboard[1]);
        snprintf(fd_wd_in,  sizeof(fd_wd_in),  "%d", pipe_watchdog_input[0]);
        snprintf(fd_wd_out,  sizeof(fd_wd_out),  "%d", pipe_input_watchdog[1]);

        execlp("konsole", "konsole", "-e",
               "./exec/input", fd_out, fd_wd_in, fd_wd_out, NULL);

        perror("exec input");
        exit(1);
    }

    /* ================= PROCESSO OBSTACLE ================= */

    pid_t pid_obst = fork();
    if (pid_obst == 0) {

        close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]);
        close(pipe_obstacle_watchdog[0]);
        close(pipe_watchdog_obstacle[1]);

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        close(pipe_input_watchdog[0]); close(pipe_input_watchdog[1]);
        close(pipe_drone_watchdog[0]); close(pipe_drone_watchdog[1]);
        close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);
        close(pipe_target_watchdog[0]); close(pipe_target_watchdog[1]);

        close(pipe_watchdog_drone[0]); close(pipe_watchdog_drone[1]);
        close(pipe_watchdog_blackboard[0]); close(pipe_watchdog_blackboard[1]);
        close(pipe_watchdog_target[0]); close(pipe_watchdog_target[1]);
        close(pipe_watchdog_input[0]);  close(pipe_watchdog_input[1]);



        char fd_in[16], fd_out[16], fd_wd_out[16], fd_wd_in[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_obstacle[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_obstacle_blackboard[1]);
        snprintf(fd_wd_in,  sizeof(fd_wd_in),  "%d", pipe_watchdog_obstacle[0]);
        snprintf(fd_wd_out,  sizeof(fd_wd_out),  "%d", pipe_obstacle_watchdog[1]);

        execlp("./exec/obstacle", "./exec/obstacle",
               fd_in, fd_out, fd_wd_in, fd_wd_out, NULL);

        perror("exec obstacle");
        exit(1);
    }

    /* ================= PROCESSO TARGET ================= */

    pid_t pid_target = fork();
    if (pid_target == 0) {

        close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]);
        close(pipe_target_watchdog[0]);
        close(pipe_watchdog_target[1]);

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);

        close(pipe_input_watchdog[0]); close(pipe_input_watchdog[1]);
        close(pipe_drone_watchdog[0]); close(pipe_drone_watchdog[1]);
        close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);
        close(pipe_obstacle_watchdog[0]); close(pipe_obstacle_watchdog[1]);

        close(pipe_watchdog_drone[0]); close(pipe_watchdog_drone[1]);
        close(pipe_watchdog_blackboard[0]); close(pipe_watchdog_blackboard[1]);
        close(pipe_watchdog_obstacle[0]); close(pipe_watchdog_obstacle[1]);
        close(pipe_watchdog_input[0]);  close(pipe_watchdog_input[1]);

        char fd_in[16], fd_out[16], fd_wd_in[16], fd_wd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_target[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_target_blackboard[1]);
        snprintf(fd_wd_in,  sizeof(fd_wd_in),  "%d", pipe_watchdog_target[0]);
        snprintf(fd_wd_out,  sizeof(fd_wd_out),  "%d", pipe_target_watchdog[1]);


        execlp("./exec/target", "./exec/target",
               fd_in, fd_out, fd_wd_in, fd_wd_out, NULL);

        perror("exec target");
        exit(1);
    }

    /* ================= PROCESSO BLACKBOARD ================= */

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
        close(pipe_watchdog_blackboard[1]);

        close(pipe_input_watchdog[0]); close(pipe_input_watchdog[1]);
        close(pipe_drone_watchdog[0]); close(pipe_drone_watchdog[1]);
        close(pipe_obstacle_watchdog[0]); close(pipe_obstacle_watchdog[1]);
        close(pipe_target_watchdog[0]); close(pipe_target_watchdog[1]);

        close(pipe_watchdog_drone[0]); close(pipe_watchdog_drone[1]);
        close(pipe_watchdog_target[0]); close(pipe_watchdog_target[1]);
        close(pipe_watchdog_obstacle[0]); close(pipe_watchdog_obstacle[1]);
        close(pipe_watchdog_input[0]);  close(pipe_watchdog_input[1]);

        char fd_in_input[16], fd_in_drone[16];
        char fd_out_drone[16], fd_out_obst[16];
        char fd_in_obst[16], fd_out_target[16], fd_in_target[16];
        char fd_wd_in[16], fd_wd_out[16];

        snprintf(fd_in_input,  sizeof(fd_in_input),  "%d", pipe_input_blackboard[0]);
        snprintf(fd_in_drone,  sizeof(fd_in_drone),  "%d", pipe_drone_blackboard[0]);
        snprintf(fd_out_drone, sizeof(fd_out_drone), "%d", pipe_blackboard_drone[1]);
        snprintf(fd_out_obst,  sizeof(fd_out_obst),  "%d", pipe_blackboard_obstacle[1]);
        snprintf(fd_in_obst,   sizeof(fd_in_obst),   "%d", pipe_obstacle_blackboard[0]);
        snprintf(fd_out_target,sizeof(fd_out_target),"%d", pipe_blackboard_target[1]);
        snprintf(fd_in_target, sizeof(fd_in_target), "%d", pipe_target_blackboard[0]);
        snprintf(fd_wd_in,     sizeof(fd_wd_in),     "%d", pipe_watchdog_blackboard[0]);
        snprintf(fd_wd_out,    sizeof(fd_wd_out),    "%d", pipe_blackboard_watchdog[1]);

        execlp("konsole", "konsole", "-e",
               "./exec/blackboard",
               fd_in_input, fd_in_drone,
               fd_out_drone, fd_out_obst,
               fd_in_obst, fd_out_target,
               fd_in_target, fd_wd_in, fd_wd_out, NULL);

        perror("exec blackboard");
        exit(1);
    }

    /* ================= PROCESSO DRONE ================= */

    pid_t pid_drone = fork();
    if (pid_drone == 0) {

        close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]);
        close(pipe_drone_watchdog[0]);
        close(pipe_watchdog_drone[1]);

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        close(pipe_input_watchdog[0]); close(pipe_input_watchdog[1]);
        close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);
        close(pipe_obstacle_watchdog[0]); close(pipe_obstacle_watchdog[1]);
        close(pipe_target_watchdog[0]); close(pipe_target_watchdog[1]);

        close(pipe_watchdog_blackboard[0]); close(pipe_watchdog_blackboard[1]);
        close(pipe_watchdog_target[0]); close(pipe_watchdog_target[1]);
        close(pipe_watchdog_obstacle[0]); close(pipe_watchdog_obstacle[1]);
        close(pipe_watchdog_input[0]);  close(pipe_watchdog_input[1]);

        char fd_in[16], fd_out[16], fd_wd_in[16], fd_wd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_blackboard_drone[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_drone_blackboard[1]);
        snprintf(fd_wd_in,  sizeof(fd_wd_in),  "%d", pipe_watchdog_drone[0]);
        snprintf(fd_wd_out,  sizeof(fd_wd_out),  "%d", pipe_drone_watchdog[1]);

        execlp("./exec/drone", "./exec/drone",
               fd_in, fd_out, fd_wd_in, fd_wd_out, NULL);

        perror("exec drone");
        exit(1);
    }

    /* ================= PROCESSO WATCHDOG ================= */

    pid_t pid_watchdog = fork();
    if (pid_watchdog == 0) {

        close(pipe_input_watchdog[1]);
        close(pipe_drone_watchdog[1]);
        close(pipe_blackboard_watchdog[1]);
        close(pipe_obstacle_watchdog[1]);
        close(pipe_target_watchdog[1]);



        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        close(pipe_watchdog_blackboard[0]);
        close(pipe_watchdog_drone[0]);
        close(pipe_watchdog_target[0]);
        close(pipe_watchdog_obstacle[0]);
        close(pipe_watchdog_input[0]);

        char fd_input_in[16], fd_drone_in[16], fd_bb_in[16], fd_obst_in[16], fd_targ_in[16];
        char fd_input_out[16], fd_drone_out[16], fd_bb_out[16], fd_obst_out[16], fd_targ_out[16];

        snprintf(fd_input_in, sizeof(fd_input_in), "%d", pipe_input_watchdog[0]);
        snprintf(fd_drone_in, sizeof(fd_drone_in), "%d", pipe_drone_watchdog[0]);
        snprintf(fd_bb_in,    sizeof(fd_bb_in),    "%d", pipe_blackboard_watchdog[0]);
        snprintf(fd_obst_in,  sizeof(fd_obst_in),  "%d", pipe_obstacle_watchdog[0]);
        snprintf(fd_targ_in,  sizeof(fd_targ_in),  "%d", pipe_target_watchdog[0]);

        snprintf(fd_input_out, sizeof(fd_input_out), "%d", pipe_watchdog_input[1]);
        snprintf(fd_drone_out, sizeof(fd_drone_out), "%d", pipe_watchdog_drone[1]);
        snprintf(fd_bb_out,    sizeof(fd_bb_out),    "%d", pipe_watchdog_blackboard[1]);
        snprintf(fd_obst_out,  sizeof(fd_obst_out),  "%d", pipe_watchdog_obstacle[1]);
        snprintf(fd_targ_out,  sizeof(fd_targ_out),  "%d", pipe_watchdog_target[1]);

        execlp("./exec/watchdog", "./exec/watchdog",
               fd_input_in, fd_drone_in, fd_bb_in, fd_obst_in, fd_targ_in,
               fd_input_out, fd_drone_out, fd_bb_out, fd_obst_out, fd_targ_out, NULL);

        perror("exec watchdog");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Watchdog started (pid=%d)", pid_watchdog);

    /* ================= PARENT ================= */

    close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
    close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
    close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
    close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
    close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
    close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
    close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

    close(pipe_input_watchdog[0]); close(pipe_input_watchdog[1]);
    close(pipe_drone_watchdog[0]); close(pipe_drone_watchdog[1]);
    close(pipe_blackboard_watchdog[0]); close(pipe_blackboard_watchdog[1]);
    close(pipe_obstacle_watchdog[0]); close(pipe_obstacle_watchdog[1]);
    close(pipe_target_watchdog[0]); close(pipe_target_watchdog[1]);

    close(pipe_watchdog_input[0]); close(pipe_watchdog_input[1]);
    close(pipe_watchdog_drone[0]); close(pipe_watchdog_drone[1]);
    close(pipe_watchdog_blackboard[0]); close(pipe_watchdog_blackboard[1]);
    close(pipe_watchdog_obstacle[0]); close(pipe_watchdog_obstacle[1]);
    close(pipe_watchdog_target[0]); close(pipe_watchdog_target[1]);

    logMessage(LOG_PATH,
        "[MAIN] All processes running (input=%d drone=%d bb=%d obst=%d targ=%d)",
        pid_input, pid_drone, pid_blackboard, pid_obst, pid_target);

    while (wait(NULL) > 0);

    logMessage(LOG_PATH, "[MAIN] PROGRAM EXIT");
    return 0;
}
