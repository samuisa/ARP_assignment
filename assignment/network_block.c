/* ======================================================================================
 * FILE: network.c
 * Gestisce la connessione TCP Server/Client e sincronizza la posizione dei droni.
 * ====================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h> // Necessario per va_list

#include "app_common.h"
#include "log.h"

// --- DEFINIZIONI ---
#define BUFSZ 256

typedef enum {
    SV_SEND_DRONE,
    SV_WAIT_DOK,
    SV_SEND_REQ_OBST,
    SV_WAIT_OBST_DATA,
    CL_WAIT_COMMAND,
    CL_WAIT_DRONE_DATA,
    CL_SEND_OBST_DATA,
    CL_WAIT_POK
} NetState;

static NetState net_state;
static int net_fd = -1;

// --- UTILITY LOGGING & NETWORKING ---

void send_msg(int fd, const char *fmt, ...) {
    char buf[BUFSZ];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    int len = strlen(buf);
    // Aggiungi newline se manca, per il protocollo basato su righe
    if (len > 0 && buf[len-1] != '\n') {
        buf[len] = '\n';
        buf[len+1] = '\0';
        len++;
    }

    ssize_t sent = write(fd, buf, len);
    if (sent < 0) {
        logMessage(LOG_PATH_SC, "[NET] ERROR sending: %s", strerror(errno));
    } else {
        // Rimuovi il newline per il log per pulizia
        if(buf[len-1] == '\n') buf[len-1] = '\0';
        logMessage(LOG_PATH_SC, "[NET] SENT: %s", buf);
    }
}

int read_line_blocking(int fd, char *out, int out_sz) {
    int pos = 0;
    char c;
    while (pos < out_sz - 1) {
        int n = read(fd, &c, 1);
        if (n <= 0) return -1;  // errore o connessione chiusa
        if (c == '\n') break;   // fine riga
        out[pos++] = c;
    }
    out[pos] = '\0';
    logMessage(LOG_PATH_SC, "[NET] RECV: %s", out);
    return pos;
}

// --- INIT CONNECTION ---

int init_server(int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(port);

    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        perror("bind");
        return -1;
    }
    listen(s, 1);

    logMessage(LOG_PATH, "[NET-SRV] Waiting for client on port %d...", port);
    printf("[NETWORK] Waiting for client...\n");
    
    int client_fd = accept(s, NULL, NULL);
    if (client_fd >= 0) {
        logMessage(LOG_PATH, "[NET-SRV] Client accepted.");
    }
    return client_fd;
}

int init_client(const char *addr, int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    inet_pton(AF_INET, addr, &a.sin_addr);

    logMessage(LOG_PATH, "[NET-CLI] Connecting to %s:%d...", addr, port);
    
    while (connect(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        printf("[NETWORK] Connecting...\n");
        sleep(1);
    }
    logMessage(LOG_PATH, "[NET-CLI] Connected to server.");
    return s;
}

// --- IPC HELPERS ---

void send_window_size(int fd_out, int w, int h) {
    Message msg;
    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", w, h);
    write(fd_out, &msg, sizeof(msg));  
    logMessage(LOG_PATH, "[NET] Sent SIZE %d %d to Blackboard", w, h);  
}

void receive_window_size(int fd_in, int *w, int *h){
    Message msg; 
    logMessage(LOG_PATH, "[NET] Waiting for window size from Blackboard...");
    ssize_t n = read(fd_in, &msg, sizeof(msg));

    if (n <= 0) {
        logMessage(LOG_PATH, "[NET] Pipe closed during size sync.");
        return;
    }
    
    int width, height; 
    if (sscanf(msg.data, "%d %d", &width, &height) == 2) {
        *w = width; 
        *h = height;
        logMessage(LOG_PATH, "[NET] Received window size: %dx%d", *w, *h);
    }
}

void receive_drone_position(float *x, float *y, int fd_in){
    Message msg;
    ssize_t n = read(fd_in, &msg, sizeof(msg));
    if (n <= 0) {
        logMessage(LOG_PATH, "[NET] Pipe blackboard closed (read drone), exiting.");
        exit(1);
    }
    float pos_x, pos_y;
    if (sscanf(msg.data, "%f %f", &pos_x, &pos_y) == 2) {
        *x = pos_x;
        *y = pos_y;
    }
}

// --- HANDSHAKE ---

int protocol_handshake(int mode, int fd, int *w, int *h, int fd_bb_out) {
    char buf[BUFSZ];

    if (mode == MODE_SERVER) {
        logMessage(LOG_PATH, "[NET-SRV] Starting Handshake...");
        
        // 1. Server invia "ok"
        send_msg(fd, "ok");
        
        // 2. Aspetta "ook"
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ook") != 0) {
            logMessage(LOG_PATH, "[NET-SRV] Handshake ERROR: expected 'ook', got '%s'", buf);
            return -1;
        }

        // 3. Server invia dimensioni
        send_msg(fd, "size %d %d", *w, *h);
        
        // 4. Aspetta conferma "sok"
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "sok %d %d", w, h) != 2) {
             logMessage(LOG_PATH, "[NET-SRV] Handshake ERROR: expected 'sok W H'");
            return -1;
        }

        logMessage(LOG_PATH, "[NET-SRV] Handshake COMPLETE. Map size: %dx%d", *w, *h);
    }

    if (mode == MODE_CLIENT) {
        logMessage(LOG_PATH, "[NET-CLI] Starting Handshake...");

        // 1. Aspetta "ok"
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ok") != 0) {
            logMessage(LOG_PATH, "[NET-CLI] Handshake ERROR: expected 'ok', got '%s'", buf);
            return -1;
        }
        
        // 2. Risponde "ook"
        send_msg(fd, "ook");

        // 3. Riceve dimensioni "size W H"
        int rw, rh;
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "size %d %d", &rw, &rh) != 2) {
            logMessage(LOG_PATH, "[NET-CLI] Handshake ERROR: expected 'size W H'");
            return -1;
        }
        *w = rw; *h = rh;

        // 4. Invia conferma e NOTIFICA LA BLACKBOARD
        send_msg(fd, "sok %d %d", *w, *h);
        
        // Importante: il client dice alla sua BB quanto è grande la mappa ricevuta dal server
        send_window_size(fd_bb_out, *w, *h);

        logMessage(LOG_PATH, "[NET-CLI] Handshake COMPLETE. Synced Map: %dx%d", *w, *h);
    }

    // Imposta stato iniziale
    net_state = (mode == MODE_SERVER) ? SV_SEND_DRONE : CL_WAIT_COMMAND;
    return 0;
}


// --- MAIN LOOP ---

void network_loop(int mode, int fd_bb_in, int fd_bb_out) {
    char net_buf[BUFSZ];
    float rx, ry; // Coordinate ricevute (Remote X, Remote Y)
    float drone_x, drone_y; // Coordinate locali
    Message msg; // Per inviare alla blackboard

    logMessage(LOG_PATH, "[NET] Starting Main Loop. Mode: %d", mode);

    while (1) {
        if (mode == MODE_SERVER) {
            switch (net_state) {
                case SV_SEND_DRONE:
                    // 1. Legge posizione drone LOCALE dalla blackboard
                    receive_drone_position(&drone_x, &drone_y, fd_bb_in);

                    // 2. Invia posizione al client
                    send_msg(net_fd, "drone");
                    send_msg(net_fd, "%f %f", drone_x, drone_y);
                    net_state = SV_WAIT_DOK;
                    break;

                case SV_WAIT_DOK:
                    read_line_blocking(net_fd, net_buf, sizeof(net_buf));
                    if (sscanf(net_buf, "dok %f %f", &rx, &ry) == 2) {
                        //logMessage(LOG_PATH_SC, "[NET-SRV] Ack received.");
                        net_state = SV_SEND_REQ_OBST;
                    } else if (strcmp(net_buf, "q") == 0) {
                        logMessage(LOG_PATH, "[NET-SRV] Client requested quit.");
                        goto exit_loop;
                    }
                    break;

                case SV_SEND_REQ_OBST:
                    send_msg(net_fd, "obst");
                    net_state = SV_WAIT_OBST_DATA;
                    break;

                case SV_WAIT_OBST_DATA:
                    read_line_blocking(net_fd, net_buf, sizeof(net_buf));
                    if (sscanf(net_buf, "%f %f", &rx, &ry) == 2) {
                        // logMessage(LOG_PATH_SC, "[NET-SRV] Received client drone: %.2f %.2f", rx, ry);
                        
                        // --- INOLTRO ALLA BLACKBOARD ---
                        msg.type = MSG_TYPE_DRONE; // Usiamo questo tipo per dire "Drone Remoto"
                        snprintf(msg.data, sizeof(msg.data), "%f %f", rx, ry);
                        if(write(fd_bb_out, &msg, sizeof(msg)) < 0) perror("write bb");
                        // -------------------------------

                        send_msg(net_fd, "pok %f %f", rx, ry);
                        net_state = SV_SEND_DRONE;
                    }
                    break;

                default: break;
            }
        } else { // CLIENT
            switch (net_state) {
                case CL_WAIT_COMMAND:
                    read_line_blocking(net_fd, net_buf, sizeof(net_buf));
                    if (strcmp(net_buf, "drone") == 0) net_state = CL_WAIT_DRONE_DATA;
                    else if (strcmp(net_buf, "obst") == 0) net_state = CL_SEND_OBST_DATA;
                    else if (strcmp(net_buf, "q") == 0) {
                        send_msg(net_fd, "qok");
                        goto exit_loop;
                    }
                    break;

                case CL_WAIT_DRONE_DATA:
                    read_line_blocking(net_fd, net_buf, sizeof(net_buf));
                    if (sscanf(net_buf, "%f %f", &rx, &ry) == 2) {
                        // logMessage(LOG_PATH_SC, "[NET-CLI] Received server drone: %.2f %.2f", rx, ry);
                        
                        // --- INOLTRO ALLA BLACKBOARD ---
                        msg.type = MSG_TYPE_DRONE; 
                        snprintf(msg.data, sizeof(msg.data), "%f %f", rx, ry);
                        if(write(fd_bb_out, &msg, sizeof(msg)) < 0) perror("write bb");
                        // -------------------------------

                        send_msg(net_fd, "dok %f %f", rx, ry);
                        net_state = CL_WAIT_COMMAND;
                    }
                    break; // FIX: break aggiunto qui

                case CL_SEND_OBST_DATA:
                    // Legge posizione drone LOCALE dalla blackboard e invia al server
                    receive_drone_position(&drone_x, &drone_y, fd_bb_in);
                    send_msg(net_fd, "%f %f", drone_x, drone_y);
                    net_state = CL_WAIT_POK;
                    break;

                case CL_WAIT_POK:
                    read_line_blocking(net_fd, net_buf, sizeof(net_buf));
                    float ox, oy;
                    if (sscanf(net_buf, "pok %f %f", &ox, &oy) == 2)
                        net_state = CL_WAIT_COMMAND;
                    break;

                default: break;
            }
        }
    }

exit_loop:
    if (net_fd >= 0) close(net_fd);
    logMessage(LOG_PATH, "[NET] Loop finished. Socket closed.");
}

int main(int argc, char *argv[]) {
    int mode;      // MODE_SERVER o MODE_CLIENT
    int port;
    const char *addr = NULL;
    int w = 100, h = 100;

    // Assicurati che il main.c passi argomenti corretti. 
    // network è forkato così: ./exec/network fd_in_bb fd_out_bb mode addr port role
    if (argc < 6) {
        fprintf(stderr, "Usage: %s <fd_in> <fd_out> <mode> <addr> <port>\n", argv[0]);
        return 1;
    }

    int fd_bb_in = atoi(argv[1]);   // Legge dalla BB
    int fd_bb_out = atoi(argv[2]);  // Scrive alla BB
    mode = atoi(argv[3]);
    if (argc > 4) addr = argv[4];
    port = atoi(argv[5]);
    
    // Inizializza log per sicurezza
    logMessage(LOG_PATH, "[NET] Process Started. Mode: %d, Port: %d", mode, port);

    if (mode == MODE_SERVER) {
        // Il Server decide la dimensione della finestra basandosi sulla sua BB
        receive_window_size(fd_bb_in, &w, &h);
        net_fd = init_server(port);
        if (net_fd < 0) { 
            logMessage(LOG_PATH, "[NET-SRV] Failed to init server.");
            return 1; 
        }
    } else {
        if (!addr) addr = "127.0.0.1";
        net_fd = init_client(addr, port);
        if (net_fd < 0) {
            logMessage(LOG_PATH, "[NET-CLI] Failed to connect.");
             return 1;
        }
    }

    // Handshake
    if (protocol_handshake(mode, net_fd, &w, &h, fd_bb_out) < 0) {
        logMessage(LOG_PATH, "[NET] Handshake FATAL ERROR. Exiting.");
        close(net_fd);
        return 1;
    }

    // Network loop
    // Passiamo fd_bb_out per poter scrivere i dati ricevuti sulla Blackboard
    network_loop(mode, fd_bb_in, fd_bb_out);

    return 0;
}