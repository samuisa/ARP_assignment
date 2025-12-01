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

// Crea la cartella logs se non esiste
void ensureLogsDir() {
    struct stat st;

    // Create logs directory if it doesn't exist
    if (stat("logs", &st) == -1) {
        if (mkdir("logs", 0700) == -1) {
            perror("mkdir logs");
            exit(1);
        }
    }

    // Clear the log file at startup
    FILE *logf = fopen(LOG_PATH, "w");  // "w" truncates the file
    if (!logf) {
        perror("fopen log file");
        exit(1);
    }
    fclose(logf);  // Close immediately, we'll reopen later in main
}

int main() {
    ensureLogsDir();
    logMessage(LOG_PATH, " [MAIN] PROGRAM STARTED");

    // -- PIPE --
    int pipe_input_window[2];
    //int pipe_window_input[2];
    int pipe_window_drone[2];
    int pipe_drone_window[2];
    int pipe_window_obstacle[2];
    int pipe_obstacle_window[2];
    int pipe_window_target[2];
    int pipe_target_window[2];

    if (pipe(pipe_input_window) == -1 ||
        pipe(pipe_window_drone) == -1 ||
        pipe(pipe_drone_window) == -1 ||
        pipe(pipe_window_obstacle) == -1 ||
        pipe(pipe_obstacle_window) == -1 ||
        pipe(pipe_window_target) == -1 ||
        pipe(pipe_target_window) == -1) {
        perror("pipe");
        exit(1);
    }

    // -------- PROCESSO INPUT --------
    pid_t pid_input = fork();
    if (pid_input == 0) {
        // chiudo pipe non usate
        close(pipe_input_window[0]);
        close(pipe_window_drone[0]); close(pipe_window_drone[1]);
        close(pipe_drone_window[0]); close(pipe_drone_window[1]);
        close(pipe_window_obstacle[0]); close(pipe_window_obstacle[1]);
        close(pipe_obstacle_window[0]); close(pipe_obstacle_window[1]);
        close(pipe_window_target[0]); close(pipe_window_target[1]);
        close(pipe_target_window[0]); close(pipe_target_window[1]);

        char fd_write_str[16];
        //char fd_read_str[16];
        snprintf(fd_write_str, sizeof(fd_write_str), "%d", pipe_input_window[1]);
        //snprintf(fd_read_str, sizeof(fd_read_str), "%d", pipe_window_input[0]);

        execlp("konsole", "konsole", "-e", "./eseguibili/input", fd_write_str, NULL);
        perror("exec input");
        exit(1);
    }

    // -------- PROCESSO OBSTACLE --------
    pid_t pid_obst = fork();
    if (pid_obst == 0) {
        close(pipe_window_obstacle[1]);
        close(pipe_input_window[0]); close(pipe_input_window[1]);
        close(pipe_window_drone[0]); close(pipe_window_drone[1]);
        close(pipe_drone_window[0]); close(pipe_drone_window[1]);
        close(pipe_window_target[0]); close(pipe_window_target[1]);
        close(pipe_target_window[0]); close(pipe_target_window[1]);

        int devnull = open("/dev/null", O_RDWR);
        dup2(devnull, STDIN_FILENO);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);

        char fd_in_str[16], fd_out_str[16];
        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_window_obstacle[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_obstacle_window[1]);

        execlp("./eseguibili/obstacle", "./eseguibili/obstacle", fd_in_str, fd_out_str, NULL);
        perror("exec obstacle");
        exit(1);
    }

    // -------- PROCESSO TARGET --------


    pid_t pid_target = fork();
    if(pid_target == 0){
        // chiudo pipe non usate
        close(pipe_input_window[0]); close(pipe_input_window[0]);
        close(pipe_window_drone[0]); close(pipe_window_drone[1]);
        close(pipe_drone_window[0]); close(pipe_drone_window[1]);
        close(pipe_window_obstacle[0]); close(pipe_window_obstacle[1]);
        close(pipe_obstacle_window[0]); close(pipe_obstacle_window[1]);
        close(pipe_window_target[1]);
        close(pipe_target_window[0]);

        char fd_in_wind_str[16]; char fd_out_wind_str[16];

        snprintf(fd_in_wind_str, sizeof(fd_in_wind_str), "%d", pipe_window_target[0]);
        snprintf(fd_out_wind_str, sizeof(fd_out_wind_str), "%d", pipe_target_window[1]);

        execlp("./eseguibili/target", "./eseguibili/target", fd_in_wind_str, fd_out_wind_str, NULL);
        perror("exec target");
        exit(1);
    }

    // -------- PROCESSO WINDOW --------
    pid_t pid_window = fork();
    if (pid_window == 0) {
        close(pipe_input_window[1]);
        close(pipe_drone_window[1]);
        close(pipe_window_drone[0]);
        close(pipe_window_obstacle[0]);
        close(pipe_obstacle_window[1]);
        close(pipe_window_target[0]);
        close(pipe_target_window[1]);

        char fd_in_input_str[16], fd_in_drone_str[16], fd_out_drone_str[16];
        char fd_out_obst_str[16], fd_in_obst_str[16], fd_in_targ_str[16], fd_out_targ_str[16];
        //char fd_out_input_str[16];

        snprintf(fd_in_input_str, sizeof(fd_in_input_str), "%d", pipe_input_window[0]);
        snprintf(fd_in_drone_str, sizeof(fd_in_drone_str), "%d", pipe_drone_window[0]);
        snprintf(fd_out_drone_str, sizeof(fd_out_drone_str), "%d", pipe_window_drone[1]);
        snprintf(fd_out_obst_str, sizeof(fd_out_obst_str), "%d", pipe_window_obstacle[1]);
        snprintf(fd_in_obst_str, sizeof(fd_in_obst_str), "%d", pipe_obstacle_window[0]);
        snprintf(fd_out_targ_str, sizeof(fd_out_targ_str), "%d", pipe_window_target[1]);
        snprintf(fd_in_targ_str, sizeof(fd_in_targ_str), "%d", pipe_target_window[0]);

        execlp("konsole", "konsole", "-e",
               "./eseguibili/window",
               fd_in_input_str, fd_in_drone_str,
               fd_out_drone_str, fd_out_obst_str,
               fd_in_obst_str, fd_out_targ_str,
               fd_in_targ_str,
               NULL);

        perror("exec window");
        exit(1);
    }

    // -------- PROCESSO DRONE --------
    pid_t pid_drone = fork();
    if (pid_drone == 0) {
        close(pipe_window_drone[1]);
        close(pipe_window_obstacle[0]); close(pipe_window_obstacle[1]);
        close(pipe_input_window[0]); close(pipe_input_window[1]);
        close(pipe_drone_window[0]);  // lo useremo per scrivere
        close(pipe_obstacle_window[0]); close(pipe_obstacle_window[1]);
        close(pipe_window_target[0]); close(pipe_window_target[1]);
        close(pipe_target_window[0]); close(pipe_target_window[1]);

        int devnull = open("/dev/null", O_RDWR);
        dup2(devnull, STDIN_FILENO);
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        close(devnull);

        char fd_in_str[16], fd_out_str[16];
        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_window_drone[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_drone_window[1]);

        execlp("./eseguibili/drone", "./eseguibili/drone", fd_in_str, fd_out_str, NULL);
        perror("exec drone");
        exit(1);
    }

    // -------- PARENT --------
    close(pipe_input_window[0]); close(pipe_input_window[1]);
    close(pipe_window_drone[0]); close(pipe_window_drone[1]);
    close(pipe_drone_window[0]); close(pipe_drone_window[1]);
    close(pipe_window_obstacle[0]); close(pipe_window_obstacle[1]);
    close(pipe_obstacle_window[0]); close(pipe_obstacle_window[1]);
    close(pipe_window_target[0]); close(pipe_window_target[1]);
    close(pipe_target_window[0]); close(pipe_target_window[1]);

    printf(" [MAIN] Main program running (Input, Drone, Window, Obstacle, Target started)\n");
    logMessage(LOG_PATH, " [MAIN] Main program running");

    // Aspetta tutti i figli
    int status;
    pid_t wpid;
    while ((wpid = wait(&status)) > 0);

    logMessage(LOG_PATH, " [MAIN] PROGRAM EXIT");
    return 0;
}
