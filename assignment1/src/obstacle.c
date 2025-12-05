#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/select.h>

#include "app_common.h"
#include "log.h"

/*====================================================================
  GENERATE OBSTACLES
  Generates a number of obstacles proportional to the grid size
======================================================================*/
Point* generate_obstacles(int width, int height, int* num_out) {

    int total_cells = (width - 2) * (height - 2);
    int count = (int) round(PERC_OBST * total_cells);
    if (count < 1) count = 1;

    Point* arr = malloc(sizeof(Point) * count);
    if (!arr) {
        logMessage(LOG_PATH, "[OBST] ERROR malloc: %s", strerror(errno));
        exit(1);
    }

    srand(time(NULL));

    for (int i = 0; i < count; i++) {
        int valid;
        do {
            valid = 1;
            arr[i].x = rand() % (width - 2) + 1;
            arr[i].y = rand() % (height - 2) + 1;
            /* Avoid duplicates */
            for (int j = 0; j < i; j++) {
                if (arr[i].x == arr[j].x && arr[i].y == arr[j].y) {
                    valid = 0;
                    break;
                }
            }
        } while (!valid);
    }

    logMessage(LOG_PATH, "[OBST] Generated %d obstacles for grid %dx%d", count, width, height);

    *num_out = count;
    return arr;
}

/*====================================================================
  MAIN OBSTACLE PROCESS
======================================================================*/
int main(int argc, char *argv[]) {

    if (argc < 3) {
        fprintf(stderr, "Usage: %s <pipe_read_fd> <pipe_write_fd>\n", argv[0]);
        return 1;
    }

    int fd_in  = atoi(argv[1]);
    int fd_out = atoi(argv[2]);

    logMessage(LOG_PATH, "[OBST] Started (fd_in=%d, fd_out=%d)", fd_in, fd_out);

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
            logMessage(LOG_PATH, "[OBST] ERROR select(): %s", strerror(errno));
            continue;
        }

        /*--------------------------
          Check if data is ready
        ---------------------------*/
        if (FD_ISSET(fd_in, &set)) {

            Message msg;
            ssize_t n = read(fd_in, &msg, sizeof(msg));

            if (n == 0) {
                logMessage(LOG_PATH, "[OBST] Pipe closed by blackboard, exiting.");
                break;
            }

            if (n < 0) {
                logMessage(LOG_PATH, "[OBST] ERROR read(): %s", strerror(errno));
                continue;
            }

            if (n != sizeof(msg)) {
                logMessage(LOG_PATH,
                           "[OBST] WARNING partial read (%zd bytes instead of %zu)",
                           n, sizeof(msg));
                continue;
            }

            /*================================================================
              RECEIVE WINDOW SIZE
            =================================================================*/
            if (msg.type == MSG_TYPE_SIZE) {

                int width, height;
                if (sscanf(msg.data, "%d %d", &width, &height) != 2) {
                    logMessage(LOG_PATH,
                               "[OBST] ERROR: invalid window size: '%s'",
                               msg.data);
                    continue;
                }

                /* Generate obstacles based on window size */
                int num_obst = 0;
                Point* arr = generate_obstacles(width, height, &num_obst);

                /*================================================================
                  SEND DATA BACK:
                  1) Message with number of obstacles
                  2) Raw array of Point structures
                =================================================================*/
                Message out_msg;
                out_msg.type = MSG_TYPE_OBSTACLES;
                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obst);

                ssize_t w1 = write(fd_out, &out_msg, sizeof(out_msg));
                ssize_t w2 = write(fd_out, arr, sizeof(Point) * num_obst);

                logMessage(LOG_PATH,
                           "[OBST] Sent %d obstacles to blackboard (width=%d height=%d) "
                           "write_msg=%zd write_array=%zd",
                           num_obst, width, height, w1, w2);

                free(arr);
            }
        }
    }

    /*================== CLEANUP ==================*/
    close(fd_in);
    close(fd_out);

    logMessage(LOG_PATH, "[OBST] Terminated successfully.");
    return 0;
}
