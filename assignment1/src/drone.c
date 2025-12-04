#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>

#include "app_common.h"
#include "log.h"

static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;


void send_position(Message msg, float x, float y, int fd_out){
    msg.type = MSG_TYPE_POSITION;
    snprintf(msg.data, sizeof(msg.data), "%f %f", x, y);
    write(fd_out, &msg, sizeof(msg));
}

void send_forces(Message msg,  int fd_out, float drone_Fx, float drone_Fy, float obst_Fx, float obst_Fy, float wall_Fx, float wall_Fy){
    msg.type = MSG_TYPE_FORCE;
    snprintf(msg.data, sizeof(msg.data), "%f %f %f %f %f %f", drone_Fx, drone_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy);
    write(fd_out, &msg, sizeof(msg));
}

int main(int argc, char *argv[]) {
    if (argc < 3) return 1;

    int fd_in  = atoi(argv[1]);
    int fd_out = atoi(argv[2]);

    fcntl(fd_in, F_SETFL, O_NONBLOCK);

    Drone drn = {0};
    int win_width = 0, win_height = 0;
    Message msg;
    bool spawned = false;

    while (1) {
        ssize_t n = read(fd_in, &msg, sizeof(Message));
        if (n > 0) {
            switch (msg.type) {

                case MSG_TYPE_SIZE: {
                    sscanf(msg.data, "%d %d", &win_width, &win_height);
                    if(!spawned){
                        drn.x = win_width  / 2.0;
                        drn.y = win_height / 2.0;
                        logMessage(LOG_PATH, "[DRONE] Initial drone position: (x = %f, y = %f)", drn.x, drn.y);
                        drn.x_1 = drn.x_2 = drn.x;
                        drn.y_1 = drn.y_2 = drn.y;
                        spawned = true;
                        send_position(msg, drn.x, drn.y, fd_out);
                    }
                    break;
                }

                case MSG_TYPE_INPUT: {
                    char ch = msg.data[0];
                    bool isValid = true;

                    if (ch == 'q') goto quit;

                    switch (ch) {
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
                            
                            if(fabs(drn.Fx) <= 0.5f){
                                drn.Fx = 0.0f;
                            }
                            if(fabs(drn.Fy) <= 0.5f){
                                drn.Fy = 0.0f;
                            }
                            break;

                        default:
                            isValid = false;
                            break;
                    }
                    if(isValid){
                        logMessage(LOG_PATH, "[DRONE] Input received, force updated, command: %c, Fx = %f, Fy = %f", ch, drn.Fx, drn.Fy);
                    }
                    else{
                        logMessage(LOG_PATH, "[DRONE] Input not recognized");
                    }
                    
                    break;
                }

                case MSG_TYPE_OBSTACLES: {
                    int count;
                    sscanf(msg.data, "%d", &count);
                    free(obstacles);
                    obstacles = count ? malloc(sizeof(Point)*count) : NULL;
                    num_obstacles = 0;
                    if (obstacles)
                        if (read(fd_in, obstacles, sizeof(Point)*count) > 0)
                            num_obstacles = count;
                    break;
                }

                case MSG_TYPE_TARGETS: {
                    int count;
                    sscanf(msg.data, "%d", &count);
                    free(targets);
                    targets = count ? malloc(sizeof(Point)*count) : NULL;
                    num_targets = 0;
                    if (targets)
                        if (read(fd_in, targets, sizeof(Point)*count) > 0)
                            num_targets = count;
                    break;
                }

                default:{
                    break;
                }
            }
        }

        // -------------------------------
        // OBSTACLE POTENTIAL FIELD
        // -------------------------------
        float repFx = 0.0f, repFy = 0.0f;

        for (int i = 0; i < num_obstacles; i++) {
            float dx = drn.x - obstacles[i].x;
            float dy = drn.y - obstacles[i].y;

            if (fabs(dx) < EPSILON) dx = 0.0f;
            if (fabs(dy) < EPSILON) dy = 0.0f;   

            float d = sqrt(dx*dx + dy*dy);

            if (d < rho && d > 0.1) {
                float F = eta * (1.0f/d - 1.0f/rho) / (d*d);
                repFx += F * dx/d;
                repFy += F * dy/d;
            }
        }

        if (fabs(repFx) < EPSILON) repFx = 0.0;
        if (fabs(repFy) < EPSILON) repFy = 0.0;

        // -------------------------------
        // WALL POTENTIAL FIELD
        // -------------------------------
        float repWallFx = 0.0f, repWallFy = 0.0f;
        float dR = (win_width-1) - drn.x;
        float dL = drn.x - 1;
        float dT = drn.y - 1;
        float dB = (win_height-1) - drn.y;

        if(dR < rho){
            repWallFx -= eta * (1.0f/dR - 1.0f/rho) / (dR*dR);
        }
        if(dL < rho){
            repWallFx += eta * (1.0f/dL - 1.0f/rho) / (dL*dL);
        }
        if(dT < rho){
            repWallFy += eta * (1.0f/dT - 1.0f/rho) / (dT*dT);
        }
        if(dB < rho){
            repWallFy -= eta * (1.0f/dB - 1.0f/rho) / (dB*dB);
        }

        // -------------------------------
        // TOTAL FORCE WITH MAX LIMIT
        // -------------------------------

        float totFx = drn.Fx + repFx + repWallFx;
        float totFy = drn.Fy + repFy + repWallFy;

        float forceMag = sqrt(totFx*totFx + totFy*totFy);
        if (forceMag > MAX_FORCE) {
            totFx = totFx / forceMag * MAX_FORCE;
            totFy = totFy / forceMag * MAX_FORCE;
        }

        // -------------------------------
        // DRONE MOVEMENT UPDATE
        // -------------------------------
        drn.x_2 = drn.x_1;
        drn.x_1 = drn.x;
        drn.y_2 = drn.y_1;
        drn.y_1 = drn.y;

        drn.x = (DT*DT*totFx - drn.x_2 + (2 + K*DT)*drn.x_1) / (1 + K*DT);
        drn.y = (DT*DT*totFy - drn.y_2 + (2 + K*DT)*drn.y_1) / (1 + K*DT);

        //drn.Fx = totFx;
        //drn.Fy = totFy;
        
        // Dopo aver calcolato drn.x e drn.y
        for (int i = 0; i < num_obstacles; i++) {
            if ((int)drn.x == obstacles[i].x && (int)drn.y == obstacles[i].y) {
                // collisione: annullo il movimento
                drn.x = drn.x_1;
                drn.y = drn.y_1;
                break;
            }
        }

        // -------------------------------
        // SEND UPDATED POSITION
        // -------------------------------
        send_position(msg, drn.x, drn.y, fd_out);
        send_forces(msg, fd_out, drn.Fx, drn.Fy, repFx, repFy, repWallFx, repWallFy);

        usleep(1000);
    }

quit:
    free(obstacles);
    free(targets);
    close(fd_in);
    close(fd_out);
    return 0;
}
