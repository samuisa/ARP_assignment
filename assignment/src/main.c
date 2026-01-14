/* ======================================================================================
 * MAIN.C - PROCESS ORCHESTRATION
 * Forks all child processes (Input, Drone, Obstacle, Target, Blackboard, Watchdog, Network)
 * and sets up IPC pipes.
 * ====================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "log.h"
#include "app_common.h"
#include "process_pid.h"

/* --------------------------------------------------------------------------------------
 * SECTION 1: LOG DIRECTORY CREATION
 * ------------------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------------------
 * SECTION 2: MAIN
 * ------------------------------------------------------------------------------------- */
int main(void) {

    ensureLogsDir();
    logMessage(LOG_PATH, "[MAIN] PROGRAM STARTED");

    // --- MODE AND ROLE ---
    memset(server_address, 0, sizeof(server_address));
    port_number = 0;
    int mode = MODE_STANDALONE;
    int role = 0;

    printf(" Select mode:\n 1: standalone\n 2: networked\n> ");
    if (scanf("%d", &mode) != 1) mode = MODE_STANDALONE;

    if (mode == MODE_NETWORKED) {
        printf(" Select role:\n 1: server\n 2: client\n> ");
        if (scanf("%d", &role) != 1) role = MODE_SERVER;
        if (role != MODE_SERVER && role != MODE_CLIENT) role = MODE_SERVER;

        if (role == MODE_CLIENT) {
            printf(" Insert IP address: ");
            scanf("%63s", server_address);
        }

        printf(" Insert port number: ");
        scanf("%d", &port_number);
    }

    logMessage(LOG_PATH, "[MAIN] Starting in MODE: %d", mode);
    if(mode == MODE_NETWORKED) logMessage(LOG_PATH_SC, "[MAIN] Network role: %d", role);

    char arg_mode[4], arg_role[4];
    snprintf(arg_mode, sizeof(arg_mode), "%d", mode);
    snprintf(arg_role, sizeof(arg_role), "%d", role);

    /* --- PIPE CREATION --- */
    int pipe_input_bb[2], pipe_bb_drone[2], pipe_drone_bb[2];
    int pipe_bb_obst[2], pipe_obst_bb[2];
    int pipe_bb_target[2], pipe_target_bb[2];
    int pipe_bb_wd[2], pipe_bb_network[2], pipe_network_bb[2];

    if (pipe(pipe_input_bb) == -1 || pipe(pipe_bb_drone) == -1 || pipe(pipe_drone_bb) == -1 ||
        pipe(pipe_bb_obst) == -1 || pipe(pipe_obst_bb) == -1 || pipe(pipe_bb_target) == -1 ||
        pipe(pipe_target_bb) == -1 || pipe(pipe_bb_wd) == -1 || pipe(pipe_bb_network) == -1 ||
        pipe(pipe_network_bb) == -1) {
        perror("pipe");
        logMessage(LOG_PATH, "[MAIN] ERROR creating pipes");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Pipes created successfully");

    /* --- RESET PID FILE --- */
    FILE *fp = fopen(PID_FILE_PATH, "w");
    if (fp) fclose(fp);

    /* --- FORK INPUT PROCESS --- */
    pid_t pid_input = fork();
    if (pid_input == 0) {
        // Close unused ends
        close(pipe_input_bb[0]);
        close(pipe_bb_drone[0]); close(pipe_bb_drone[1]);
        close(pipe_drone_bb[0]); close(pipe_drone_bb[1]);
        close(pipe_bb_obst[0]); close(pipe_bb_obst[1]);
        close(pipe_obst_bb[0]); close(pipe_obst_bb[1]);
        close(pipe_bb_target[0]); close(pipe_bb_target[1]);
        close(pipe_target_bb[0]); close(pipe_target_bb[1]);
        close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);
        close(pipe_bb_network[0]); close(pipe_bb_network[1]);
        close(pipe_network_bb[0]); close(pipe_network_bb[1]);

        char fd_out[16]; snprintf(fd_out, sizeof(fd_out), "%d", pipe_input_bb[1]);
        execlp("konsole", "konsole", "-e", "./exec/input", fd_out, arg_mode, NULL);
        perror("exec input");
        exit(1);
    }

    /* --- FORK BLACKBOARD PROCESS --- */
    pid_t pid_bb = fork();
    if (pid_bb == 0) {
        // Close unused ends
        close(pipe_input_bb[1]);
        close(pipe_bb_drone[0]); close(pipe_drone_bb[1]);
        close(pipe_bb_obst[0]); close(pipe_obst_bb[1]);
        close(pipe_bb_target[0]); close(pipe_target_bb[1]);
        close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);
        close(pipe_bb_network[0]); close(pipe_network_bb[1]);

        char fd_in_input[16], fd_in_drone[16], fd_out_drone[16];
        char fd_out_obst[16], fd_in_obst[16];
        char fd_out_target[16], fd_in_target[16];
        char fd_out_wd[16], fd_out_network[16], fd_in_network[16], arg_port[16];

        snprintf(fd_in_input,   sizeof(fd_in_input),   "%d", pipe_input_bb[0]);
        snprintf(fd_in_drone,   sizeof(fd_in_drone),   "%d", pipe_drone_bb[0]);
        snprintf(fd_out_drone,  sizeof(fd_out_drone),  "%d", pipe_bb_drone[1]);
        snprintf(fd_out_obst,   sizeof(fd_out_obst),   "%d", pipe_bb_obst[1]);
        snprintf(fd_in_obst,    sizeof(fd_in_obst),    "%d", pipe_obst_bb[0]);
        snprintf(fd_out_target, sizeof(fd_out_target), "%d", pipe_bb_target[1]);
        snprintf(fd_in_target,  sizeof(fd_in_target),  "%d", pipe_target_bb[0]);
        snprintf(fd_out_wd,     sizeof(fd_out_wd),     "%d", pipe_bb_wd[1]);
        snprintf(arg_port,      sizeof(arg_port),      "%d", port_number);
        snprintf(fd_out_network,sizeof(fd_out_network),"%d", pipe_bb_network[1]);
        snprintf(fd_in_network, sizeof(fd_in_network), "%d", pipe_network_bb[0]);

        if (strlen(server_address) == 0) strcpy(server_address, "0.0.0.0");

        execlp("konsole", "konsole", "-e",
            "./exec/blackboard",
            fd_in_input, fd_in_drone,
            fd_out_drone, fd_out_obst,
            fd_in_obst, fd_out_target,
            fd_in_target, fd_out_wd,
            arg_mode, server_address,
            fd_out_network, fd_in_network,
            arg_role, NULL);

        perror("exec blackboard");
        exit(1);
    }

    /* --- FORK DRONE PROCESS --- */
    pid_t pid_drone = fork();
    if (pid_drone == 0) {
        // Close unused
        close(pipe_bb_drone[1]); close(pipe_drone_bb[0]);
        close(pipe_input_bb[0]); close(pipe_input_bb[1]);
        close(pipe_bb_obst[0]); close(pipe_bb_obst[1]);
        close(pipe_obst_bb[0]); close(pipe_obst_bb[1]);
        close(pipe_bb_target[0]); close(pipe_bb_target[1]);
        close(pipe_target_bb[0]); close(pipe_target_bb[1]);
        close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);
        close(pipe_bb_network[0]); close(pipe_bb_network[1]);
        close(pipe_network_bb[0]); close(pipe_network_bb[1]);

        char fd_in[16], fd_out[16];
        snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_bb_drone[0]);
        snprintf(fd_out, sizeof(fd_out), "%d", pipe_drone_bb[1]);

        execlp("./exec/drone", "./exec/drone", fd_in, fd_out, arg_mode, arg_role, NULL);
        perror("exec drone");
        exit(1);
    }

    pid_t pid_obst = -1, pid_target = -1, pid_watchdog = -1, pid_network = -1;

    if(mode == MODE_STANDALONE){
        /* --- FORK OBSTACLE PROCESS --- */

        pid_obst = fork();
        if(pid_obst < 0) {
            perror("fork obstacle");
            logMessage(LOG_PATH, "[MAIN] fork obstacle failed: %d", errno);
            exit(1);
        }
        if(pid_obst == 0){
            close(pipe_bb_obst[1]); close(pipe_obst_bb[0]);
            close(pipe_input_bb[0]); close(pipe_input_bb[1]);
            close(pipe_bb_drone[0]); close(pipe_bb_drone[1]);
            close(pipe_drone_bb[0]); close(pipe_drone_bb[1]);
            close(pipe_bb_target[0]); close(pipe_bb_target[1]);
            close(pipe_target_bb[0]); close(pipe_target_bb[1]);
            close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);
            close(pipe_bb_network[0]); close(pipe_bb_network[1]);
            close(pipe_network_bb[0]); close(pipe_network_bb[1]);

            char fd_in[16], fd_out[16];
            snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_bb_obst[0]);
            snprintf(fd_out, sizeof(fd_out), "%d", pipe_obst_bb[1]);

            execlp("./exec/obstacle", "./exec/obstacle", fd_in, fd_out, NULL);
            perror("exec obstacle");
            exit(1);
        }

        /* --- FORK TARGET PROCESS --- */
        pid_target = fork();
        if(pid_target < 0) {
            perror("fork target");
            logMessage(LOG_PATH, "[MAIN] fork target failed: %d", errno);
            exit(1);
        }
        if(pid_target == 0){
            close(pipe_bb_target[1]); close(pipe_target_bb[0]);
            close(pipe_input_bb[0]); close(pipe_input_bb[1]);
            close(pipe_bb_drone[0]); close(pipe_bb_drone[1]);
            close(pipe_drone_bb[0]); close(pipe_drone_bb[1]);
            close(pipe_bb_obst[0]); close(pipe_bb_obst[1]);
            close(pipe_obst_bb[0]); close(pipe_obst_bb[1]);
            close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);
            close(pipe_bb_network[0]); close(pipe_bb_network[1]);
            close(pipe_network_bb[0]); close(pipe_network_bb[1]);

            char fd_in[16], fd_out[16];
            snprintf(fd_in,  sizeof(fd_in),  "%d", pipe_bb_target[0]);
            snprintf(fd_out, sizeof(fd_out), "%d", pipe_target_bb[1]);

            execlp("./exec/target", "./exec/target", fd_in, fd_out, NULL);
            perror("exec target");
            exit(1);
        }

        /* --- FORK WATCHDOG --- */
        pid_watchdog = fork();
        if(pid_watchdog < 0) {
            perror("fork watchdog");
            logMessage(LOG_PATH, "[MAIN] fork watchdog failed: %d", errno);
            exit(1);
        }
        if(pid_watchdog == 0){
            close(pipe_bb_wd[1]);
            close(pipe_input_bb[0]); close(pipe_input_bb[1]);
            close(pipe_bb_drone[0]); close(pipe_bb_drone[1]);
            close(pipe_drone_bb[0]); close(pipe_drone_bb[1]);
            close(pipe_bb_obst[0]); close(pipe_bb_obst[1]);
            close(pipe_obst_bb[0]); close(pipe_obst_bb[1]);
            close(pipe_bb_target[0]); close(pipe_bb_target[1]);
            close(pipe_target_bb[0]); close(pipe_target_bb[1]);
            close(pipe_bb_network[0]); close(pipe_bb_network[1]);
            close(pipe_network_bb[0]); close(pipe_network_bb[1]);

            char fd_in_bb[16]; snprintf(fd_in_bb, sizeof(fd_in_bb), "%d", pipe_bb_wd[0]);
            execlp("konsole", "konsole", "-e", "./exec/watchdog", fd_in_bb, NULL);
            perror("exec watchdog");
            exit(1);
        }

    } else if(mode == MODE_NETWORKED){
        /* --- FORK NETWORK PROCESS --- */
        pid_network = fork();
        if(pid_network == 0){
            close(pipe_network_bb[0]); close(pipe_bb_network[1]);
            close(pipe_input_bb[0]); close(pipe_input_bb[1]);
            close(pipe_bb_drone[0]); close(pipe_bb_drone[1]);
            close(pipe_drone_bb[0]); close(pipe_drone_bb[1]);
            close(pipe_bb_obst[0]); close(pipe_bb_obst[1]);
            close(pipe_obst_bb[0]); close(pipe_obst_bb[1]);
            close(pipe_bb_target[0]); close(pipe_bb_target[1]);
            close(pipe_target_bb[0]); close(pipe_target_bb[1]);
            close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);

            char fd_out_bb[16], fd_in_bb[16], arg_port[16];
            snprintf(fd_out_bb, sizeof(fd_out_bb), "%d", pipe_network_bb[1]);
            snprintf(fd_in_bb,  sizeof(fd_in_bb),  "%d", pipe_bb_network[0]);
            snprintf(arg_port, sizeof(arg_port), "%d", port_number);

            if (strlen(server_address) == 0) strcpy(server_address, "127.0.0.1");

            logMessage(LOG_PATH_SC, "[MAIN] Network running in mode: %d", role);

            execl("./exec/network", "./exec/network", fd_in_bb, fd_out_bb, arg_role,
                  server_address, arg_port, NULL);
            perror("exec network");
            exit(1);
        }
    }

    /* --- CLOSE ALL PARENT PIPE ENDS --- */
    close(pipe_input_bb[0]); close(pipe_input_bb[1]);
    close(pipe_bb_drone[0]); close(pipe_bb_drone[1]);
    close(pipe_drone_bb[0]); close(pipe_drone_bb[1]);
    close(pipe_bb_obst[0]); close(pipe_bb_obst[1]);
    close(pipe_obst_bb[0]); close(pipe_obst_bb[1]);
    close(pipe_bb_target[0]); close(pipe_bb_target[1]);
    close(pipe_target_bb[0]); close(pipe_target_bb[1]);
    close(pipe_bb_wd[0]); close(pipe_bb_wd[1]);
    close(pipe_bb_network[0]); close(pipe_bb_network[1]);
    close(pipe_network_bb[0]); close(pipe_network_bb[1]);

    logMessage(LOG_PATH, "[MAIN] All processes started (input=%d drone=%d bb=%d obst=%d targ=%d watchdog=%d network=%d)",
        pid_input, pid_drone, pid_bb, pid_obst, pid_target, pid_watchdog, pid_network);

    /* --- WAIT FOR CHILDREN --- */
    while (wait(NULL) > 0);
    logMessage(LOG_PATH, "[MAIN] PROGRAM EXIT");

    return 0;
}
