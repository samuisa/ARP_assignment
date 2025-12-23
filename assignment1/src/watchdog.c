#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "log.h"
#include "app_common.h"

#define LOG_PATH_WD "logs/watchdog.log"
/*typedef struct {
    pid_t pid;
    char name[32];
} watched_proc_t;

static watched_proc_t procs[MAX_PROC];
static int num_procs = 0;*/

pid_t receive_pid(int fd_read){
    Message msg;
    ssize_t n;
    pid_t pid = -1;

    do {
        n = read(fd_read, &msg, sizeof(msg));
    } while(n < 0 && errno == EINTR);

    if(n <= 0){
        perror("[WD] read watchdog PID");
        exit(1);
    }

    if(msg.type == MSG_TYPE_PID){
        sscanf(msg.data, "%d", &pid);
        //logMessage(LOG_PATH, "[WD] Watchdog PID received: %d", pid_wd);
    } else {
        //logMessage(LOG_PATH, "[WD] Unexpected message type from watchdog: %d", msg.type);
    }

    return pid;
}

void send_pid(int fd_write){
    pid_t pid = getpid();
    Message msg;
    msg.type = MSG_TYPE_PID;
    snprintf(msg.data, sizeof(msg.data), "%d", pid);
    write(fd_write, &msg, sizeof(msg));
}

/* ================= MAIN WATCHDOG ================= */
int main(int argc, char *argv[]) {

    if(argc < 10) {
        fprintf(stderr, "Usage: %s <fd_proc_read> [<fd_proc_read> ...]\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[2]);
    int fd_bb_read     = atoi(argv[3]);
    int fd_obst_read   = atoi(argv[4]);
    int fd_targ_read   = atoi(argv[5]);

    int fd_input_write = atoi(argv[6]);
    int fd_drone_write = atoi(argv[7]);
    int fd_bb_write    = atoi(argv[8]);
    int fd_obst_write  = atoi(argv[9]);
    int fd_targ_write  = atoi(argv[10]);

    pid_t pid_input = receive_pid(fd_input_read);
        logMessage(LOG_PATH_WD, "[WD] Input process pid received: %d", pid_input);

    pid_t pid_drone = receive_pid(fd_drone_read);
        logMessage(LOG_PATH_WD, "[WD] Drone process pid received: %d", pid_drone);

    pid_t pid_bb    = receive_pid(fd_bb_read);
        logMessage(LOG_PATH_WD, "[WD] Blackboard process pid received: %d", pid_bb);

    pid_t pid_obst  = receive_pid(fd_obst_read);
        logMessage(LOG_PATH_WD, "[WD] Obstacle process pid received: %d", pid_obst);

    pid_t pid_targ  = receive_pid(fd_targ_read);
        logMessage(LOG_PATH_WD, "[WD] Target process pid received: %d", pid_targ);


    send_pid(fd_input_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to input process: %d", getpid());

    send_pid(fd_drone_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to drone process: %d", getpid());

    send_pid(fd_bb_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to blackboard process: %d", getpid());

    send_pid(fd_obst_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to obstacle process: %d", getpid());

    send_pid(fd_targ_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to target process: %d", getpid());


    return 0;
}
