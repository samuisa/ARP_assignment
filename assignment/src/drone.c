/* ======================================================================================
 * FILE: drone.c
 * Logic: 
 * 1. Flush Input Pipe (Handle all pending keys/obstacles)
 * 2. Calculate Physics (High Frequency ~1000Hz)
 * 3. Send Output to Blackboard (Throttled ~30Hz to avoid pipe flooding)
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

// Configurazione Frequenza
#define PHYSICS_DT_SEC 0.001f    // 1ms physics step
#define RENDER_FPS 30            // 30 invii al secondo alla blackboard
#define RENDER_DT_NS (1000000000L / RENDER_FPS)

typedef enum {
    STATE_INIT, STATE_WAITING_INPUT, STATE_PROCESSING_INPUT,
    STATE_CALCULATING_PHYSICS, STATE_SENDING_OUTPUT, STATE_IDLE
} ProcessState;

static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;
static volatile pid_t watchdog_pid = -1; 
static volatile sig_atomic_t current_state = STATE_INIT;

// --- HELPERS ---
void watchdog_ping_handler(int sig) {
    (void)sig; 
    if (watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}

void wait_for_watchdog_pid() {
    FILE *fp;
    char line[256], tag[128];
    int pid_temp;
    bool wd_found = false;
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
}

void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", DRONE_PID_TAG, getpid());
}

void send_position(Message msg, float x, float y, int fd_out){
    msg.type = MSG_TYPE_POSITION;
    snprintf(msg.data, sizeof(msg.data), "%f %f", x, y);
    write(fd_out, &msg, sizeof(msg));
}

void send_forces(Message msg, int fd_out, float drone_Fx, float drone_Fy,
                 float obst_Fx, float obst_Fy, float wall_Fx, float wall_Fy,
                 float abtrFx, float abtrFy){
    msg.type = MSG_TYPE_FORCE;
    snprintf(msg.data, sizeof(msg.data), "%f %f %f %f %f %f %f %f",
             drone_Fx, drone_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, abtrFx, abtrFy);
    write(fd_out, &msg, sizeof(msg));
}

long get_time_diff_ns(struct timespec t1, struct timespec t2) {
    return (t2.tv_sec - t1.tv_sec) * 1000000000L + (t2.tv_nsec - t1.tv_nsec);
}

// --- MAIN ---
int main(int argc, char *argv[]) {
    if (argc < 5) return 1;

    int fd_in   = atoi(argv[1]);
    int fd_out  = atoi(argv[2]);
    int mode    = atoi(argv[3]);
    int role    = atoi(argv[4]);

    signal(SIGPIPE, SIG_IGN); 
    // Fondamentale: Pipe Non-Blocking per svuotarla velocemente
    fcntl(fd_in, F_SETFL, O_NONBLOCK);

    Drone drn = {0};
    Message msg;
    int win_width = 0, win_height = 0;
    bool spawned = false;

    // Watchdog Setup
    if(mode == MODE_STANDALONE){
        // 1. PRIMA PUBBLICA IL PID (Così il watchdog sa che esistiamo)
        FILE *fp_pid = fopen(PID_FILE_PATH, "a");
        if (!fp_pid) {
            perror("fopen PID file");
            exit(1);
        }
        int fd_pid = fileno(fp_pid);
        flock(fd_pid, LOCK_EX); 
        publish_my_pid(fp_pid);
        fflush(fp_pid);
        flock(fd_pid, LOCK_UN);
        fclose(fp_pid);

        // 2. POI SETUP SEGNALI
        struct sigaction sa;
        sa.sa_handler = watchdog_ping_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART;
        sigaction(SIGUSR1, &sa, NULL);

        // 3. INFINE ASPETTA IL PID DEL WATCHDOG
        // Se il WD ci pinga mentre siamo qui, il handler ignorerà il segnale
        // perché watchdog_pid è ancora -1. Al prossimo ciclo saremo pronti.
        wait_for_watchdog_pid();
    }

    struct timespec last_render_time;
    clock_gettime(CLOCK_MONOTONIC, &last_render_time);

    // --- MAIN SIMULATION LOOP ---
    while (1) {
        
        // ====================================================================
        // STEP 1: INPUT FLUSHING (Drain the pipe)
        // Legge TUTTI i messaggi disponibili. Se la blackboard manda 10 messaggi,
        // li processiamo tutti ORA invece di aspettare 10 cicli.
        // ====================================================================
        current_state = STATE_PROCESSING_INPUT;
        
        while(1) {
            ssize_t n = read(fd_in, &msg, sizeof(Message));
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break; // Pipe vuota
                if (errno == EINTR) continue; // Interrotto da segnale, riprova
                break; // Errore vero
            }
            if (n == 0) break; // EOF

            // Handle Message
            switch (msg.type) {
                case MSG_TYPE_SIZE: {
                    sscanf(msg.data, "%d %d", &win_width, &win_height);
                    if (!spawned) {
                        // Logica Spawn (come richiesto)
                        if (role == MODE_SERVER) { drn.x = 3.0f; drn.y = 3.0f; } 
                        else if (role == MODE_CLIENT) { drn.x = (float)win_width - 5.0f; drn.y = (float)win_height - 5.0f; } 
                        else { drn.x = win_width / 2.0f; drn.y = win_height / 2.0f; }
                        
                        drn.x_1 = drn.x_2 = drn.x;
                        drn.y_1 = drn.y_2 = drn.y;
                        spawned = true;
                        // Invio immediato al primo spawn
                        send_position(msg, drn.x, drn.y, fd_out);
                    }
                    break;
                }
                case MSG_TYPE_INPUT: {
                    char ch = msg.data[0];
                    if(ch == 'q') goto quit;
                    // Apply Forces
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
                            drn.Fx *= 0.5f; drn.Fy *= 0.5f;
                            if(fabs(drn.Fx) <= 0.5f) drn.Fx = 0.0f;
                            if(fabs(drn.Fy) <= 0.5f) drn.Fy = 0.0f;
                            break;
                    }
                    break;
                }
                case MSG_TYPE_OBSTACLES: { 
                    int count; sscanf(msg.data, "%d", &count); 
                    free(obstacles); obstacles = count ? malloc(sizeof(Point)*count) : NULL; 
                    if (obstacles) read(fd_in, obstacles, sizeof(Point)*count);
                    num_obstacles = count; 
                    break; 
                }
                case MSG_TYPE_TARGETS: { 
                    int count; sscanf(msg.data, "%d", &count); 
                    free(targets); targets = count ? malloc(sizeof(Point)*count) : NULL; 
                    if (targets) read(fd_in, targets, sizeof(Point)*count);
                    num_targets = count; 
                    break; 
                }
            }
        } // End Input While

        // ====================================================================
        // STEP 2: PHYSICS CALCULATION (Run every cycle)
        // ====================================================================
        current_state = STATE_CALCULATING_PHYSICS;
        float repFx=0.0f, repFy=0.0f, repWallFx=0.0f, repWallFy=0.0f, abtrFx = 0.0f, abtrFy = 0.0f;        
        
        // A. Attractive (Targets)
        for(int i=0; i<num_targets; i++){
            float dx = drn.x - ((float)targets[i].x + 0.5);
            float dy = drn.y - ((float)targets[i].y + 0.5);
            float d = sqrt(dx*dx + dy*dy) - 0.5f;
            if(d < rho && d > 0.1f){
                float F = eta * (1.0f/d - 1.0f/rho) / (d*d);
                abtrFx += F * dx/d; abtrFy += F * dy/d;
            }
        }

        // B. Repulsive (Obstacles)
        for(int i=0; i<num_obstacles; i++){
            float dx = drn.x - ((float)obstacles[i].x + 0.5);
            float dy = drn.y - ((float)obstacles[i].y + 0.5);
            float d = sqrt(dx*dx + dy*dy) - 0.5f;
            if(d < rho && d > 0.1f){
                float F = eta * (1.0f/d - 1.0f/rho) / (d*d);
                repFx += F * dx/d; repFy += F * dy/d;
            }
        }

        // C. Walls
        float dR = (win_width-1) - drn.x;
        float dL = drn.x - 1;
        float dT = drn.y - 1;
        float dB = (win_height-1) - drn.y;
        if(dR < rho) repWallFx -= eta * (1.0f/dR - 1.0f/rho)/(dR*dR);
        if(dL < rho) repWallFx += eta * (1.0f/dL - 1.0f/rho)/(dL*dL);
        if(dT < rho) repWallFy += eta * (1.0f/dT - 1.0f/rho)/(dT*dT);
        if(dB < rho) repWallFy -= eta * (1.0f/dB - 1.0f/rho)/(dB*dB);

        // D. Sum & Clamp
        float totFx = drn.Fx + repFx + repWallFx - abtrFx;
        float totFy = drn.Fy + repFy + repWallFy - abtrFy;
        float forceMag = sqrt(totFx*totFx + totFy*totFy);
        if(forceMag > MAX_FORCE){
            totFx = totFx/forceMag*MAX_FORCE;
            totFy = totFy/forceMag*MAX_FORCE;
        }

        // E. Euler Integration
        drn.x_2 = drn.x_1; drn.x_1 = drn.x;
        drn.y_2 = drn.y_1; drn.y_1 = drn.y;
        drn.x = (DT*DT*totFx - drn.x_2 + (2+K*DT)*drn.x_1)/(1+K*DT);
        drn.y = (DT*DT*totFy - drn.y_2 + (2+K*DT)*drn.y_1)/(1+K*DT);

        // F. Collision
        for(int i=0; i<num_obstacles; i++){
            float dx = drn.x - (float)obstacles[i].x;
            float dy = drn.y - (float)obstacles[i].y;
            if(sqrt(dx*dx + dy*dy) <= 0.1f){
                drn.x = drn.x_1; drn.y = drn.y_1;
                break;
            }
        }

        // ====================================================================
        // STEP 3: OUTPUT THROTTLING (Send only at ~30 FPS)
        // ====================================================================
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        
        if (get_time_diff_ns(last_render_time, now) >= RENDER_DT_NS) {
            current_state = STATE_SENDING_OUTPUT;
            send_position(msg, drn.x, drn.y, fd_out);
            send_forces(msg, fd_out, drn.Fx, drn.Fy, repFx, repFy, repWallFx, repWallFy, abtrFx, abtrFy);
            last_render_time = now;
        }

        // Sleep to maintain ~1000Hz Physics Loop
        usleep(1000); 
    }

quit:
    free(obstacles);
    free(targets);
    close(fd_in);
    close(fd_out);
    return 0;
}