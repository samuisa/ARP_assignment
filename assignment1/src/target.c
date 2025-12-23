#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sys/select.h>
#include <time.h>

#include "app_common.h"
#include "log.h"

/*====================================================================
  GLOBAL VARIABLES
======================================================================*/
static Point *obstacles = NULL;
static int num_obstacles = 0;

/*====================================================================
  GENERATE TARGETS
  Generates a number of targets proportional to grid size, avoiding obstacles
======================================================================*/
Point* generate_targets(int width, int height,
                        Point* obstacles, int num_obstacles,
                        int* num_out) {

    int total_cells = (width - 2) * (height - 2);
    int count = (int) round(PERC_TARG * total_cells);
    if (count < 1) count = 1;

    Point* arr = malloc(sizeof(Point) * count);
    if (!arr) {
        logMessage(LOG_PATH, "[TARG] ERROR malloc target: %s", strerror(errno));
        exit(1);
    }

    srand(time(NULL));

    for (int i = 0; i < count; i++) {
        int valid;
        do {
            valid = 1;
            arr[i].x = rand() % (width - 2) + 1;
            arr[i].y = rand() % (height - 2) + 1;

            /* Avoid duplicates in targets */
            for (int j = 0; j < i; j++) {
                if (arr[i].x == arr[j].x && arr[i].y == arr[j].y) {
                    valid = 0;
                    break;
                }
            }

            /* Avoid overlap with obstacles */
            for (int j = 0; j < num_obstacles && valid; j++) {
                if (arr[i].x == obstacles[j].x &&
                    arr[i].y == obstacles[j].y) {
                    valid = 0;
                    break;
                }
            }

        } while (!valid);
    }

    logMessage(LOG_PATH, "[TARG] Generated %d targets for grid %dx%d",
               count, width, height);

    *num_out = count;
    return arr;
}

void send_pid(int fd_wd_write){
    pid_t pid = getpid();
    Message msg;
    msg.type = MSG_TYPE_PID;
    snprintf(msg.data, sizeof(msg.data), "%d", pid);
    if(write(fd_wd_write, &msg, sizeof(msg)) < 0){
        perror("[TARG] write PID to watchdog");
        exit(1);
    }
    logMessage(LOG_PATH, "[TARG] PID sent to watchdog: %d", pid);
}

pid_t receive_watchdog_pid(int fd_wd_read){
    Message msg;
    ssize_t n;
    pid_t pid_wd = -1;

    do {
        n = read(fd_wd_read, &msg, sizeof(msg));
    } while(n < 0 && errno == EINTR);

    if(n <= 0){
        perror("[TARG] read watchdog PID");
        exit(1);
    }

    if(msg.type == MSG_TYPE_PID){
        sscanf(msg.data, "%d", &pid_wd);
        logMessage(LOG_PATH, "[TARG] Watchdog PID received: %d", pid_wd);
    } else {
        logMessage(LOG_PATH, "[TARG] Unexpected message type from watchdog: %d", msg.type);
    }

    return pid_wd;
}

/*====================================================================
  MAIN TARGET PROCESS
======================================================================*/
int main(int argc, char *argv[]) {

    if (argc < 5) {
        fprintf(stderr, "Usage: %s <pipe_read_fd> <pipe_write_fd> <pipe_watchdog_write_fd> <pipe_watchdog_read_fd>\n", argv[0]);
        return 1;
    }

    int fd_in  = atoi(argv[1]);
    int fd_out = atoi(argv[2]);
    int fd_wd_read     = atoi(argv[3]);
    int fd_wd_write    = atoi(argv[4]);
    //pid_t watchdog_pid = atoi(argv[3]);


    logMessage(LOG_PATH, "[TARG] Started (fd_in=%d, fd_out=%d)", fd_in, fd_out);

    int win_width = 0;
    int win_height = 0;

    send_pid(fd_wd_write);

    pid_t pid_watchdog = receive_watchdog_pid(fd_wd_read);

    /*================== MAIN LOOP ==================*/
    while (1) {

        /*--------------------------
          Setup select() to read pipe
        ---------------------------*/
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_in, &set);

        int ret = select(fd_in + 1, &set, NULL, NULL, NULL);
        if (ret < 0) {
            logMessage(LOG_PATH, "[TARG] ERROR select(): %s", strerror(errno));
            continue;
        }

        /*--------------------------
          Check if data is ready
        ---------------------------*/
        if (FD_ISSET(fd_in, &set)) {

            Message msg;
            ssize_t n = read(fd_in, &msg, sizeof(msg));

            if (n == 0) {
                logMessage(LOG_PATH, "[TARG] Pipe closed by blackboard, exiting.");
                break;
            }

            if (n < 0) {
                logMessage(LOG_PATH, "[TARG] ERROR read(): %s", strerror(errno));
                continue;
            }

            if (n != sizeof(msg)) {
                logMessage(LOG_PATH,
                           "[TARG] WARNING partial read (%zd bytes instead of %zu)",
                           n, sizeof(msg));
                continue;
            }

            /*================================================================
              RECEIVE WINDOW SIZE
            =================================================================*/
            if (msg.type == MSG_TYPE_SIZE) {

                if (sscanf(msg.data, "%d %d", &win_width, &win_height) != 2) {
                    logMessage(LOG_PATH, "[TARG] ERROR: invalid window size '%s'", msg.data);
                    continue;
                }

                logMessage(LOG_PATH, "[TARG] Received window size %dx%d", win_width, win_height);
            }

            /*================================================================
              RECEIVE OBSTACLES
            =================================================================*/
            if (msg.type == MSG_TYPE_OBSTACLES){

                int count = 0;
                if (sscanf(msg.data, "%d", &count) == 1 && count > 0) {

                    if (obstacles != NULL) {
                        free(obstacles);
                        obstacles = NULL;
                    }

                    obstacles = malloc(sizeof(Point) * count);
                    if (!obstacles) {
                        logMessage(LOG_PATH, "[TARG] ERROR allocating obstacles");
                        break;
                    }

                    ssize_t n_read = read(fd_in, obstacles, sizeof(Point) * count);
                    if ((size_t)n_read != sizeof(Point) * (size_t)count) {
                        logMessage(LOG_PATH, "[TARG] ERROR reading obstacles (%zd bytes)", n_read);
                        free(obstacles);
                        obstacles = NULL;
                        num_obstacles = 0;
                        continue;
                    }

                    num_obstacles = count;
                    logMessage(LOG_PATH, "[TARG] Received %d obstacles", num_obstacles);

                    /*--------------------------
                      GENERATE TARGETS
                      Only if window size has been received
                    ---------------------------*/
                    if (win_width > 0 && win_height > 0) {

                        int num_targ = 0;
                        Point* arr = generate_targets(
                            win_width, win_height,
                            obstacles, num_obstacles,
                            &num_targ
                        );

                        /*--------------------------
                          SEND TARGETS TO BLACKBOARD
                        ---------------------------*/
                        Message out_msg;
                        out_msg.type = MSG_TYPE_TARGETS;
                        snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targ);

                        write(fd_out, &out_msg, sizeof(out_msg));
                        write(fd_out, arr, sizeof(Point) * num_targ);

                        logMessage(LOG_PATH, "[TARG] Sent %d targets to blackboard", num_targ);

                        free(arr);
                    } else {
                        logMessage(LOG_PATH, "[TARG] WARNING: window size not yet received");
                    }
                }
            }
        }
    }

    /*================== CLEANUP ==================*/
    close(fd_in);
    close(fd_out);

    logMessage(LOG_PATH, "[TARG] Terminated successfully.");
    return 0;
}
