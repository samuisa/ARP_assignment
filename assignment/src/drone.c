/* ======================================================================================
 * SECTION 1: INCLUDES AND CONFIGURATION
 * Headers and Physics constants (EPSILON).
 * ====================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>

#include "app_common.h"
#include "log.h"
#include "process_pid.h"
#undef EPSILON
#define EPSILON 0.001f

typedef enum {
    STATE_INIT, STATE_WAITING_INPUT, STATE_PROCESSING_INPUT,
    STATE_CALCULATING_PHYSICS, STATE_SENDING_OUTPUT, STATE_IDLE
} ProcessState;

// Global Game Objects
static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;

static volatile pid_t watchdog_pid = -1; 
static volatile sig_atomic_t current_state = STATE_INIT;

/* ======================================================================================
 * SECTION 2: WATCHDOG & IPC HELPERS
 * Signal handlers, PID registration, and Pipe sending functions.
 * ====================================================================================== */

void watchdog_ping_handler(int sig) {
    (void)sig; 
    if (watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

void wait_for_watchdog_pid() {
    FILE *fp;
    char line[256], tag[128];
    int pid_temp;
    bool wd_found = false;

    //logMessage(LOG_PATH, "[DRONE] Waiting for Watchdog...");

    while (!wd_found) {
        fp = fopen(PID_FILE_PATH, "r");
        if (fp) {
            while (fgets(line, sizeof(line), fp)) {
                if (sscanf(line, "%s %d", tag, &pid_temp) == 2) {
                    if (strcmp(tag, WD_PID_TAG) == 0) {
                        watchdog_pid = (pid_t)pid_temp;
                        wd_found = true;
                        break;
                    }
                }
            }
            fclose(fp);
        }
        if (!wd_found) usleep(200000);
    }
    //logMessage(LOG_PATH, "[DRONE] Watchdog found (PID %d)", watchdog_pid);
}

void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", DRONE_PID_TAG, getpid());
    //logMessage(LOG_PATH, "[DRONE] PID published securely");
}

void send_position(Message msg, float x, float y, int fd_out){
    msg.type = MSG_TYPE_POSITION;
    snprintf(msg.data, sizeof(msg.data), "%f %f", x, y);
    //logMessage(LOG_PATH, "[DRONE] Sending position: (%f, %f)", x, y);
    write(fd_out, &msg, sizeof(msg));
}

void send_forces(Message msg, int fd_out, float drone_Fx, float drone_Fy,
                 float obst_Fx, float obst_Fy, float wall_Fx, float wall_Fy,
                 float abtrFx, float abtrFy){
    msg.type = MSG_TYPE_FORCE;
    snprintf(msg.data, sizeof(msg.data), "%f %f %f %f %f %f %f %f",
             drone_Fx, drone_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, abtrFx, abtrFy);
    //logMessage(LOG_PATH, "[DRONE] Sending forces: drone(%f,%f) obst(%f,%f) wall(%f,%f) targ(%f,%f)", drone_Fx, drone_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, abtrFx, abtrFy);
    write(fd_out, &msg, sizeof(msg));
}

/* ======================================================================================
 * SECTION 3: MAIN EXECUTION & PHYSICS ENGINE
 * ====================================================================================== */
int main(int argc, char *argv[]) {
    if (argc < 4) return 1;

    int fd_in   = atoi(argv[1]);
    int fd_out  = atoi(argv[2]);
    int mode    = atoi(argv[3]);

    signal(SIGPIPE, SIG_IGN); 
    fcntl(fd_in, F_SETFL, O_NONBLOCK);

    Drone drn = {0};
    Message msg;
    int win_width = 0, win_height = 0;
    bool spawned = false;

    //logMessage(LOG_PATH, "[DRONE] Process started");

    if(mode == MODE_STANDALONE){

        struct sigaction sa;
        sa.sa_handler = watchdog_ping_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART;
        sigaction(SIGUSR1, &sa, NULL);

        // --- WATCHDOG SYNCHRONIZATION ---
        wait_for_watchdog_pid();

        FILE *fp_pid = fopen(PID_FILE_PATH, "a");
        if (!fp_pid) {
            //logMessage(LOG_PATH, "[DRONE] Error opening PID file!");
            exit(1);
        }
        int fd_pid = fileno(fp_pid);
        flock(fd_pid, LOCK_EX); 
        publish_my_pid(fp_pid);
        fflush(fp_pid);
        flock(fd_pid, LOCK_UN);
        fclose(fp_pid);
    }

    // --- MAIN LOOP ---
    while (1) {
        current_state = STATE_WAITING_INPUT;
        
        // 1. NON-BLOCKING READ for Inputs/Updates
        ssize_t n = read(fd_in, &msg, sizeof(Message));
        
        if (n < 0 && errno == EINTR) {
            continue;
        }

        if (n > 0) {
            current_state = STATE_PROCESSING_INPUT;
            switch (msg.type) {
                case MSG_TYPE_SIZE: {
                    sscanf(msg.data, "%d %d", &win_width, &win_height);
                    if (!spawned) {
                        // Center spawn
                        drn.x = win_width / 2.0f;
                        drn.y = win_height / 2.0f;
                        drn.x_1 = drn.x_2 = drn.x;
                        drn.y_1 = drn.y_2 = drn.y;
                        spawned = true;
                        current_state = STATE_SENDING_OUTPUT;
                        send_position(msg, drn.x, drn.y, fd_out);
                    }
                    break;
                }
                case MSG_TYPE_INPUT: {
                    // Force Control Logic (Keys to Force Vector)
                    char ch = msg.data[0];
                    //logMessage(LOG_PATH, "[DRONE] Input received: '%c'", ch);
                    if(ch == 'q') {
                        goto quit;
                        break;
                    }

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
                        case 'd': // Brake
                            drn.Fx *= 0.5f;
                            drn.Fy *= 0.5f;
                            if(fabs(drn.Fx) <= 0.5f) drn.Fx = 0.0f;
                            if(fabs(drn.Fy) <= 0.5f) drn.Fy = 0.0f;
                            break;
                        default: isValid = false; break;
                    }
                    if(isValid){
                        //logMessage(LOG_PATH, "[DRONE] Forces updated: Fx=%f Fy=%f", drn.Fx, drn.Fy);
                    }
                    break;
                }

                case MSG_TYPE_OBSTACLES: { 
                    // Deserialize Obstacles
                    //logMessage(LOG_PATH, "[DRONE] Obstacles metadata received: %s", msg.data); 
                    int count; 
                    sscanf(msg.data, "%d", &count); 
                    free(obstacles); 
                    obstacles = count ? malloc(sizeof(Point)*count) : NULL; 
                    num_obstacles = 0; 
                    if (obstacles) { 
                        if (read(fd_in, obstacles, sizeof(Point)*count) > 0) { 
                            num_obstacles = count; 
                        } 
                    } 
                    break; 
                }

                case MSG_TYPE_TARGETS: { 
                    // Deserialize Targets
                    //logMessage(LOG_PATH, "[DRONE] Targets metadata received: %s", msg.data); 
                    int count; 
                    sscanf(msg.data, "%d", &count); 
                    free(targets); targets = count ? malloc(sizeof(Point)*count) : NULL; 
                    num_targets = 0; 
                    if (targets) { 
                        if (read(fd_in, targets, sizeof(Point)*count) > 0) { 
                            num_targets = count; 
                        } 
                    } 
                    break; 
                }
                default: break;
            }
        }

        /* --- SUB-SECTION: PHYSICS CALCULATIONS --- */
        current_state = STATE_CALCULATING_PHYSICS;
        float repFx=0.0f, repFy=0.0f, repWallFx=0.0f, repWallFy=0.0f, abtrFx = 0.0f, abtrFy = 0.0f;        
        
        // A. Attractive Forces (Targets)
        for(int i=0; i<num_targets; i++){
            float dx = drn.x - ((float)targets[i].x + 0.5);
            float dy = drn.y - ((float)targets[i].y + 0.5);
            float d = sqrt(dx*dx + dy*dy) - 0.5f;

            //logMessage(LOG_PATH, "distanza da target %d d = %f", i, d);
            if(d < rho && d > 0.1f){
                float F = eta * (1.0f/d - 1.0f/rho) / (d*d);
                abtrFx += F * dx/d;
                abtrFy += F * dy/d;
            }
        }
        if(fabs(abtrFx) < EPSILON) abtrFx = 0.0f;
        if(fabs(abtrFy) < EPSILON) abtrFy = 0.0f; 

        // B. Repulsive Forces (Obstacles)
        for(int i=0; i<num_obstacles; i++){
            float dx = drn.x - ((float)obstacles[i].x + 0.5);
            float dy = drn.y - ((float)obstacles[i].y + 0.5);
            float d = sqrt(dx*dx + dy*dy) - 0.5f;
            if(d < rho && d > 0.1f){
                float F = eta * (1.0f/d - 1.0f/rho) / (d*d);
                repFx += F * dx/d;
                repFy += F * dy/d;
            }
        }
        if(fabs(repFx) < EPSILON) repFx = 0.0f;
        if(fabs(repFy) < EPSILON) repFy = 0.0f;

        // C. Repulsive Forces (Walls)
        float dR = (win_width-1) - drn.x;
        float dL = drn.x - 1;
        float dT = drn.y - 1;
        float dB = (win_height-1) - drn.y;

        if(dR < rho) repWallFx -= eta * (1.0f/dR - 1.0f/rho)/(dR*dR);
        if(dL < rho) repWallFx += eta * (1.0f/dL - 1.0f/rho)/(dL*dL);
        if(dT < rho) repWallFy += eta * (1.0f/dT - 1.0f/rho)/(dT*dT);
        if(dB < rho) repWallFy -= eta * (1.0f/dB - 1.0f/rho)/(dB*dB);

        // D. Force Summation and Clamping
        float totFx = drn.Fx + repFx + repWallFx - abtrFx;
        float totFy = drn.Fy + repFy + repWallFy - abtrFy;
        float forceMag = sqrt(totFx*totFx + totFy*totFy);
        if(forceMag > MAX_FORCE){
            totFx = totFx/forceMag*MAX_FORCE;
            totFy = totFy/forceMag*MAX_FORCE;
        }

        // E. Euler Integration (Position Update)
        drn.x_2 = drn.x_1; drn.x_1 = drn.x;
        drn.y_2 = drn.y_1; drn.y_1 = drn.y;
        drn.x = (DT*DT*totFx - drn.x_2 + (2+K*DT)*drn.x_1)/(1+K*DT);
        drn.y = (DT*DT*totFy - drn.y_2 + (2+K*DT)*drn.y_1)/(1+K*DT);

        // F. Hard Collision Check (Obstacles)
        for(int i=0; i<num_obstacles; i++){
            float dx = drn.x - (float)obstacles[i].x;
            float dy = drn.y - (float)obstacles[i].y;
            float d = sqrt(dx*dx + dy*dy);
            if(d <= 0.1f){
                drn.x = drn.x_1;
                drn.y = drn.y_1;
                break;
            }
        }

        // 3. SEND OUTPUT (Position and Forces)
        current_state = STATE_SENDING_OUTPUT;
        send_position(msg, drn.x, drn.y, fd_out);
        send_forces(msg, fd_out, drn.Fx, drn.Fy, repFx, repFy, repWallFx, repWallFy, abtrFx, abtrFy);

        current_state = STATE_IDLE;
        usleep(1000); 
    }
quit:
    free(obstacles);
    free(targets);
    close(fd_in);
    close(fd_out);
    return 0;
}