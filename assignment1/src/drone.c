#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>

#include "app_common.h"
#include "log.h"

#define EPSILON 0.001f

static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;

/* ======================= SEND FUNCTIONS ======================= */
void send_position(Message msg, float x, float y, int fd_out){
    msg.type = MSG_TYPE_POSITION;
    snprintf(msg.data, sizeof(msg.data), "%f %f", x, y);
    logMessage(LOG_PATH, "[DRONE] Sending position: (%f, %f)", x, y);
    write(fd_out, &msg, sizeof(msg));
}

void send_forces(Message msg, int fd_out, float drone_Fx, float drone_Fy,
                 float obst_Fx, float obst_Fy, float wall_Fx, float wall_Fy){
    msg.type = MSG_TYPE_FORCE;
    snprintf(msg.data, sizeof(msg.data), "%f %f %f %f %f %f",
             drone_Fx, drone_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy);
    logMessage(LOG_PATH, "[DRONE] Sending forces: drone(%f,%f) obst(%f,%f) wall(%f,%f)",
               drone_Fx, drone_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy);
    write(fd_out, &msg, sizeof(msg));
}

/* ======================= WATCHDOG PID ======================= */
void send_pid(int fd_wd_write){
    pid_t pid = getpid();
    Message msg;
    msg.type = MSG_TYPE_PID;
    snprintf(msg.data, sizeof(msg.data), "%d", pid);
    if(write(fd_wd_write, &msg, sizeof(msg)) < 0){
        perror("[DRONE] write PID to watchdog");
        exit(1);
    }
    logMessage(LOG_PATH, "[DRONE] PID sent to watchdog: %d", pid);
}

pid_t receive_watchdog_pid(int fd_wd_read){
    Message msg;
    ssize_t n;
    pid_t pid_wd = -1;

    do {
        n = read(fd_wd_read, &msg, sizeof(msg));
    } while(n < 0 && errno == EINTR);

    if(n <= 0){
        perror("[DRONE] read watchdog PID");
        exit(1);
    }

    if(msg.type == MSG_TYPE_PID){
        sscanf(msg.data, "%d", &pid_wd);
        logMessage(LOG_PATH, "[DRONE] Watchdog PID received: %d", pid_wd);
    } else {
        logMessage(LOG_PATH, "[DRONE] Unexpected message type from watchdog: %d", msg.type);
    }

    return pid_wd;
}

/* ======================= MAIN ======================= */
int main(int argc, char *argv[]){
    if(argc < 5){
        fprintf(stderr, "Usage: %s <fd_in> <fd_out> <fd_watchdog_read> <fd_watchdog_write>\n", argv[0]);
        return 1;
    }

    int fd_in       = atoi(argv[1]);
    int fd_out      = atoi(argv[2]);
    int fd_wd_read  = atoi(argv[3]);
    int fd_wd_write = atoi(argv[4]);

    fcntl(fd_in, F_SETFL, O_NONBLOCK);

    Drone drn = {0};
    int win_width = 0, win_height = 0;
    Message msg;
    bool spawned = false;

    logMessage(LOG_PATH, "[DRONE] Process started");

    send_pid(fd_wd_write);

    pid_t pid_watchdog = receive_watchdog_pid(fd_wd_read);

    /* ================= LOOP PRINCIPALE ================= */
    while(1){
        ssize_t n = read(fd_in, &msg, sizeof(Message));
        if(n > 0){
            logMessage(LOG_PATH, "[DRONE] Message received: type=%d, data='%s'", msg.type, msg.data);

            switch(msg.type){
                case MSG_TYPE_SIZE: {
                    sscanf(msg.data, "%d %d", &win_width, &win_height);
                    logMessage(LOG_PATH, "[DRONE] Window size: width=%d height=%d", win_width, win_height);

                    if(!spawned){
                        drn.x = win_width / 2.0;
                        drn.y = win_height / 2.0;
                        drn.x_1 = drn.x_2 = drn.x;
                        drn.y_1 = drn.y_2 = drn.y;
                        spawned = true;
                        send_position(msg, drn.x, drn.y, fd_out);
                    }
                    break;
                }

                case MSG_TYPE_INPUT: {
                    char ch = msg.data[0];
                    logMessage(LOG_PATH, "[DRONE] Input received: '%c'", ch);

                    if(ch == 'q') goto quit;

                    bool isValid = true;
                    switch(ch){
                        case 'e':  drn.Fy -= 1.0f; break;
                        case 'r':  drn.Fx += 1.0f; drn.Fy -= 1.0f; break;
                        case 'f':  drn.Fx += 1.0f; break;
                        case 'v':  drn.Fx += 1.0f; drn.Fy += 1.0f; break;
                        case 'c':  drn.Fy += 1.0f; break;
                        case 'x':  drn.Fx -= 1.0f; drn.Fy += 1.0f; break;
                        case 's':  drn.Fx -= 1.0f; break;
                        case 'w':  drn.Fx -= 1.0f; drn.Fy -= 1.0f; break;
                        case 'd':
                            drn.Fx *= 0.5f;
                            drn.Fy *= 0.5f;
                            if(fabs(drn.Fx) <= 0.5f) drn.Fx = 0.0f;
                            if(fabs(drn.Fy) <= 0.5f) drn.Fy = 0.0f;
                            break;
                        default: isValid = false; break;
                    }

                    if(isValid)
                        logMessage(LOG_PATH, "[DRONE] Forces updated: Fx=%f Fy=%f", drn.Fx, drn.Fy);
                    else
                        logMessage(LOG_PATH, "[DRONE] Unknown input: '%c'", ch);

                    break;
                }

                case MSG_TYPE_OBSTACLES: { 
                    logMessage(LOG_PATH, "[DRONE] Obstacles metadata received: %s", msg.data); 
                    int count; 
                    sscanf(msg.data, "%d", &count); 
                    logMessage(LOG_PATH, "[DRONE] Number of obstacles = %d", count); 
                    free(obstacles); 
                    obstacles = count ? malloc(sizeof(Point)*count) : NULL; 
                    num_obstacles = 0; 
                    if (obstacles) { 
                        if (read(fd_in, obstacles, sizeof(Point)*count) > 0) { 
                            num_obstacles = count; 
                            for(int i=0; i<count; i++){
                                logMessage(LOG_PATH, "[DRONE] Obstacle %d at (%f, %f)", i, obstacles[i].x, obstacles[i].y); 
                            }
                        } 
                    } 
                    break; 
                }

                case MSG_TYPE_TARGETS: { 
                    logMessage(LOG_PATH, "[DRONE] Targets metadata received: %s", msg.data); 
                    int count; 
                    sscanf(msg.data, "%d", &count); 
                    logMessage(LOG_PATH, "[DRONE] Number of targets = %d", count); 
                    free(targets); targets = count ? malloc(sizeof(Point)*count) : NULL; 
                    num_targets = 0; 
                    if (targets) { 
                        if (read(fd_in, targets, sizeof(Point)*count) > 0) { 
                            num_targets = count; 
                            for(int i=0; i<count; i++){ 
                                logMessage(LOG_PATH, "[DRONE] Target %d at (%f, %f)", i, targets[i].x, targets[i].y);
                            } 
                        } 
                    } 
                    break; 
                }

                default: break;
            }
        }

        /* ================= CALCOLO DELLE FORZE ================= */
        float repFx=0.0f, repFy=0.0f, repWallFx=0.0f, repWallFy=0.0f;

        // Ostacoli
        for(int i=0; i<num_obstacles; i++){
            float dx = drn.x - (obstacles[i].x + 0.5);
            float dy = drn.y - (obstacles[i].y + 0.5);
            float d = sqrt(dx*dx + dy*dy) - 0.5f;
            if(d < rho && d > 0.1f){
                float F = eta * (1.0f/d - 1.0f/rho) / (d*d);
                repFx += F * dx/d;
                repFy += F * dy/d;
            }
        }
        if(fabs(repFx) < EPSILON) repFx = 0.0f;
        if(fabs(repFy) < EPSILON) repFy = 0.0f;

        // Muri
        float dR = (win_width-1) - drn.x;
        float dL = drn.x - 1;
        float dT = drn.y - 1;
        float dB = (win_height-1) - drn.y;

        if(dR < rho) repWallFx -= eta * (1.0f/dR - 1.0f/rho)/(dR*dR);
        if(dL < rho) repWallFx += eta * (1.0f/dL - 1.0f/rho)/(dL*dL);
        if(dT < rho) repWallFy += eta * (1.0f/dT - 1.0f/rho)/(dT*dT);
        if(dB < rho) repWallFy -= eta * (1.0f/dB - 1.0f/rho)/(dB*dB);

        // Somma forze e limitazione
        float totFx = drn.Fx + repFx + repWallFx;
        float totFy = drn.Fy + repFy + repWallFy;
        float forceMag = sqrt(totFx*totFx + totFy*totFy);
        if(forceMag > MAX_FORCE){
            totFx = totFx/forceMag*MAX_FORCE;
            totFy = totFy/forceMag*MAX_FORCE;
        }

        // Aggiornamento posizione
        drn.x_2 = drn.x_1; drn.x_1 = drn.x;
        drn.y_2 = drn.y_1; drn.y_1 = drn.y;
        drn.x = (DT*DT*totFx - drn.x_2 + (2+K*DT)*drn.x_1)/(1+K*DT);
        drn.y = (DT*DT*totFy - drn.y_2 + (2+K*DT)*drn.y_1)/(1+K*DT);

        // Controllo collisioni con ostacoli
        for(int i=0; i<num_obstacles; i++){
            float dx = drn.x - obstacles[i].x;
            float dy = drn.y - obstacles[i].y;
            float d = sqrt(dx*dx + dy*dy);
            if(d <= 0.1f){
                drn.x = drn.x_1;
                drn.y = drn.y_1;
                break;
            }
        }

        // Invio aggiornamenti
        send_position(msg, drn.x, drn.y, fd_out);
        send_forces(msg, fd_out, drn.Fx, drn.Fy, repFx, repFy, repWallFx, repWallFy);

        usleep(1000);
    }

quit:
    logMessage(LOG_PATH, "[DRONE] Shutdown requested");

    free(obstacles);
    free(targets);
    close(fd_in);
    close(fd_out);
    close(fd_wd_read);
    close(fd_wd_write);

    return 0;
}
