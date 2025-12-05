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

void ensureLogsDir() {
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

int main() {
    ensureLogsDir();
    logMessage(LOG_PATH, "[MAIN] PROGRAM STARTED");

    // -- PIPE --
    int pipe_input_blackboard[2];
    int pipe_blackboard_drone[2];
    int pipe_drone_blackboard[2];
    int pipe_blackboard_obstacle[2];
    int pipe_obstacle_blackboard[2];
    int pipe_blackboard_target[2];
    int pipe_target_blackboard[2];

    if (pipe(pipe_input_blackboard) == -1 ||
        pipe(pipe_blackboard_drone) == -1 ||
        pipe(pipe_drone_blackboard) == -1 ||
        pipe(pipe_blackboard_obstacle) == -1 ||
        pipe(pipe_obstacle_blackboard) == -1 ||
        pipe(pipe_blackboard_target) == -1 ||
        pipe(pipe_target_blackboard) == -1) {
        perror("pipe");
        logMessage(LOG_PATH, "[MAIN] ERROR creating pipes");
        exit(1);
    }
    logMessage(LOG_PATH, "[MAIN] Pipes created successfully");

    // -------- PROCESSO INPUT --------
    pid_t pid_input = fork();
    if (pid_input == 0) {
        logMessage(LOG_PATH, "[MAIN] Input process started (pid=%d)", getpid());

        close(pipe_input_blackboard[0]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_write_str[16];
        snprintf(fd_write_str, sizeof(fd_write_str), "%d", pipe_input_blackboard[1]);

        execlp("konsole", "konsole", "-e", "./exec/input", fd_write_str, NULL);
        perror("exec input");
        logMessage(LOG_PATH, "[MAIN] ERROR executing input process");
        exit(1);
    }

    // -------- PROCESSO OBSTACLE --------
    pid_t pid_obst = fork();
    if (pid_obst == 0) {
        logMessage(LOG_PATH, "[MAIN] Obstacle process started (pid=%d)", getpid());

        close(pipe_blackboard_obstacle[1]);
        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        int devnull = open("/dev/null", O_RDWR);
        dup2(devnull, STDIN_FILENO);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);

        char fd_in_str[16], fd_out_str[16];
        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_blackboard_obstacle[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_obstacle_blackboard[1]);

        execlp("./exec/obstacle", "./exec/obstacle", fd_in_str, fd_out_str, NULL);
        perror("exec obstacle");
        logMessage(LOG_PATH, "[MAIN] ERROR executing obstacle process");
        exit(1);
    }

    // -------- PROCESSO TARGET --------
    pid_t pid_target = fork();
    if(pid_target == 0){
        logMessage(LOG_PATH, "[MAIN] Target process started (pid=%d)", getpid());

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]);

        char fd_in_wind_str[16]; char fd_out_wind_str[16];
        snprintf(fd_in_wind_str, sizeof(fd_in_wind_str), "%d", pipe_blackboard_target[0]);
        snprintf(fd_out_wind_str, sizeof(fd_out_wind_str), "%d", pipe_target_blackboard[1]);

        execlp("./exec/target", "./exec/target", fd_in_wind_str, fd_out_wind_str, NULL);
        perror("exec target");
        logMessage(LOG_PATH, "[MAIN] ERROR executing target process");
        exit(1);
    }

    // -------- PROCESSO BLACKBOARD --------
    pid_t pid_blackboard = fork();
    if (pid_blackboard == 0) {
        logMessage(LOG_PATH, "[MAIN] Blackboard process started (pid=%d)", getpid());

        close(pipe_input_blackboard[1]);
        close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_drone[0]);
        close(pipe_blackboard_obstacle[0]);
        close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]);
        close(pipe_target_blackboard[1]);

        char fd_in_input_str[16], fd_in_drone_str[16], fd_out_drone_str[16];
        char fd_out_obst_str[16], fd_in_obst_str[16], fd_in_targ_str[16], fd_out_targ_str[16];

        snprintf(fd_in_input_str, sizeof(fd_in_input_str), "%d", pipe_input_blackboard[0]);
        snprintf(fd_in_drone_str, sizeof(fd_in_drone_str), "%d", pipe_drone_blackboard[0]);
        snprintf(fd_out_drone_str, sizeof(fd_out_drone_str), "%d", pipe_blackboard_drone[1]);
        snprintf(fd_out_obst_str, sizeof(fd_out_obst_str), "%d", pipe_blackboard_obstacle[1]);
        snprintf(fd_in_obst_str, sizeof(fd_in_obst_str), "%d", pipe_obstacle_blackboard[0]);
        snprintf(fd_out_targ_str, sizeof(fd_out_targ_str), "%d", pipe_blackboard_target[1]);
        snprintf(fd_in_targ_str, sizeof(fd_in_targ_str), "%d", pipe_target_blackboard[0]);

        execlp("konsole", "konsole", "-e",
               "./exec/blackboard",
               fd_in_input_str, fd_in_drone_str,
               fd_out_drone_str, fd_out_obst_str,
               fd_in_obst_str, fd_out_targ_str,
               fd_in_targ_str,
               NULL);

        perror("exec blackboard");
        logMessage(LOG_PATH, "[MAIN] ERROR executing blackboard process");
        exit(1);
    }

    // -------- PROCESSO DRONE --------
    pid_t pid_drone = fork();
    if (pid_drone == 0) {
        logMessage(LOG_PATH, "[MAIN] Drone process started (pid=%d)", getpid());

        close(pipe_blackboard_drone[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_drone_blackboard[0]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        int devnull = open("/dev/null", O_RDWR);
        dup2(devnull, STDIN_FILENO);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);

        char fd_in_str[16], fd_out_str[16];
        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_blackboard_drone[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_drone_blackboard[1]);

        execlp("./exec/drone", "./exec/drone", fd_in_str, fd_out_str, NULL);
        perror("exec drone");
        logMessage(LOG_PATH, "[MAIN] ERROR executing drone process");
        exit(1);
    }

    // -------- PARENT --------
    close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
    close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
    close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
    close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
    close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
    close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
    close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

    printf("[MAIN] Main program running (Input, Drone, Blackboard, Obstacle, Target started)\n");
    logMessage(LOG_PATH, "[MAIN] Main program running");

    int status;
    pid_t wpid;
    while ((wpid = wait(&status)) > 0) {
        logMessage(LOG_PATH, "[MAIN] Child process pid=%d exited with status=%d", wpid, status);
    }

    logMessage(LOG_PATH, "[MAIN] PROGRAM EXIT");
    return 0;
}
