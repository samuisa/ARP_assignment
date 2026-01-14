/* ======================================================================================
 * FILE: network.c
 * Protocollo STRETTO: Request -> Datum -> Ack
 * Modalità: Non-Blocking I/O Multiplexing
 * ====================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/select.h> 
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h>

#include "app_common.h"
#include "log.h"

#define BUFSZ 1024 

// --- STATI DEL PROTOCOLLO (Esattamente come da specifica) ---
typedef enum {
    // Stati SERVER
    SV_SEND_CMD_DRONE,   // Invia "drone"
    SV_SEND_DATA_DRONE,  // Invia "x y"
    SV_WAIT_DOK,         // Aspetta "dok x y"
    SV_SEND_CMD_OBST,    // Invia "obst"
    SV_WAIT_DATA_OBST,   // Aspetta "x y" dal client
    
    // Stati CLIENT
    CL_WAIT_COMMAND,     // Aspetta "drone", "obst" o "q"
    CL_WAIT_DRONE_DATA,  // Ha ricevuto "drone", aspetta "x y"
    CL_SEND_OBST_DATA,   // Ha ricevuto "obst", deve inviare "x y"
    CL_WAIT_POK          // Ha inviato dati, aspetta "pok x y"
} NetState;

static NetState net_state;
static int net_fd = -1;

// Buffer per socket non bloccante
typedef struct {
    char data[BUFSZ];
    int len;
} SocketBuffer;

static SocketBuffer sock_buf = { .len = 0 };

// Variabili per conservare l'ultima posizione nota (per evitare blocchi)
static float my_last_x = 0.0f;
static float my_last_y = 0.0f;

// --- UTILITY ---

void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

void send_msg(int fd, const char *fmt, ...) {
    char buf[BUFSZ];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    int len = strlen(buf);
    // Assicura newline finale
    if (len > 0 && buf[len-1] != '\n') {
        buf[len] = '\n';
        buf[len+1] = '\0';
        len++;
    }

    ssize_t sent = write(fd, buf, len);
    if (sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EPIPE)
            logMessage(LOG_PATH_SC, "[NET] ERROR sending: %s", strerror(errno));
    }
}

// Legge chunk dal socket
int read_socket_chunk(int fd) {
    if (sock_buf.len >= BUFSZ - 1) return 0; 
    ssize_t n = read(fd, sock_buf.data + sock_buf.len, BUFSZ - 1 - sock_buf.len);
    if (n > 0) {
        sock_buf.len += n;
        sock_buf.data[sock_buf.len] = '\0';
        return 1; 
    }
    return (n == 0) ? -1 : 0;
}

// Estrae riga dal buffer
int get_line_from_buffer(char *out_line, int max_len) {
    char *newline_ptr = strchr(sock_buf.data, '\n');
    if (newline_ptr) {
        int line_len = newline_ptr - sock_buf.data;
        if (line_len >= max_len) line_len = max_len - 1;
        memcpy(out_line, sock_buf.data, line_len);
        out_line[line_len] = '\0';
        
        int remaining = sock_buf.len - (newline_ptr - sock_buf.data) - 1;
        memmove(sock_buf.data, newline_ptr + 1, remaining);
        sock_buf.len = remaining;
        sock_buf.data[sock_buf.len] = '\0';
        return 1;
    }
    return 0;
}

// Lettura bloccante per handshake
int read_line_blocking(int fd, char *out, int out_sz) {
    int pos = 0; char c;
    while (pos < out_sz - 1) {
        if (read(fd, &c, 1) <= 0) return -1;
        if (c == '\n') break;
        out[pos++] = c;
    }
    out[pos] = '\0';
    return pos;
}

// --- CONNESSIONE & HANDSHAKE ---

int init_server(int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1; setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) return -1;
    listen(s, 1);
    logMessage(LOG_PATH, "[NET-SRV] Waiting...");
    return accept(s, NULL, NULL);
}

int init_client(const char *addr, int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; a.sin_port = htons(port);
    inet_pton(AF_INET, addr, &a.sin_addr);
    while (connect(s, (struct sockaddr*)&a, sizeof(a)) < 0) sleep(1);
    return s;
}

void send_window_size(int fd_out, int w, int h) {
    Message msg; msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", w, h);
    write(fd_out, &msg, sizeof(msg));  
}

void receive_window_size(int fd_in, int *w, int *h){
    Message msg; 
    if (read(fd_in, &msg, sizeof(msg)) > 0) sscanf(msg.data, "%d %d", w, h);
}

// Lettura non bloccante della pipe locale (aggiorna cache)
void update_local_position(int fd_in) {
    Message msg;
    // Leggiamo tutto quello che c'è nella pipe per svuotarla e prendere l'ultimo dato
    while (read(fd_in, &msg, sizeof(msg)) > 0) {
        // Supponiamo che la blackboard invii MSG_TYPE_POSITION o DRONE
        if (sscanf(msg.data, "%f %f", &my_last_x, &my_last_y) == 2) {
            // Updated
        }
    }
}

int protocol_handshake(int mode, int fd, int *w, int *h, int fd_bb_out) {
    char buf[BUFSZ];
    if (mode == MODE_SERVER) {
        send_msg(fd, "ok");
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ook") != 0) return -1;
        send_msg(fd, "size %d %d", *w, *h);
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "sok %d %d", w, h) != 2) return -1;
    } else {
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ok") != 0) return -1;
        send_msg(fd, "ook");
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "size %d %d", w, h) != 2) return -1;
        send_window_size(fd_bb_out, *w, *h);
        send_msg(fd, "sok %d %d", *w, *h);
    }
    net_state = (mode == MODE_SERVER) ? SV_SEND_CMD_DRONE : CL_WAIT_COMMAND;
    return 0;
}

// --- MAIN LOOP ---

void network_loop(int mode, int fd_bb_in, int fd_bb_out) {
    char net_line[BUFSZ];
    float rx, ry; 
    Message msg; 
    fd_set read_fds;
    struct timeval timeout; 

    logMessage(LOG_PATH, "[NET] Starting Strict Protocol Loop. Mode: %d", mode);

    set_nonblocking(net_fd);
    set_nonblocking(fd_bb_in);

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(net_fd, &read_fds);
        FD_SET(fd_bb_in, &read_fds);
        
        int max_fd = (net_fd > fd_bb_in) ? net_fd : fd_bb_in;

        // Se siamo in uno stato che attende RETE e abbiamo dati bufferizzati, timeout 0
        int has_buf = (strchr(sock_buf.data, '\n') != NULL);
        timeout.tv_sec = 0;
        timeout.tv_usec = has_buf ? 0 : 2000; // 2ms polling

        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        if (activity < 0 && errno != EINTR) break;

        // 1. AGGIORNA POSIZIONE LOCALE (Sempre, indipendentemente dallo stato)
        if (FD_ISSET(fd_bb_in, &read_fds)) {
            update_local_position(fd_bb_in);
        }

        // 2. LEGGI RETE (Riempie solo il buffer)
        if (FD_ISSET(net_fd, &read_fds)) {
            if (read_socket_chunk(net_fd) == -1) goto exit_loop;
        }

        // 3. MACCHINA A STATI (Elabora Logica)
        int state_changed;
        do {
            state_changed = 0;

            if (mode == MODE_SERVER) {
                switch (net_state) {
                    case SV_SEND_CMD_DRONE:
                        // Invia comando "drone"
                        send_msg(net_fd, "drone");
                        net_state = SV_SEND_DATA_DRONE;
                        state_changed = 1; // Passa subito al prossimo
                        break;

                    case SV_SEND_DATA_DRONE:
                        // Invia coordinate "x y" (usa cache locale corrente)
                        send_msg(net_fd, "%f %f", my_last_x, my_last_y);
                        net_state = SV_WAIT_DOK;
                        // Non settiamo state_changed: ora dobbiamo aspettare rete
                        break;

                    case SV_WAIT_DOK:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            // Aspettiamo "dok <drone>" cioè "dok x y"
                            if (sscanf(net_line, "dok %f %f", &rx, &ry) == 2) {
                                // Ack ricevuto. Prossimo passo: chiedere ostacoli
                                net_state = SV_SEND_CMD_OBST;
                                state_changed = 1;
                            } else if (strcmp(net_line, "q") == 0) goto exit_loop;
                        }
                        break;

                    case SV_SEND_CMD_OBST:
                        send_msg(net_fd, "obst");
                        net_state = SV_WAIT_DATA_OBST;
                        break;

                    case SV_WAIT_DATA_OBST:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            // Aspettiamo "x y" puri
                            if (sscanf(net_line, "%f %f", &rx, &ry) == 2) {
                                // Ricevuto Ostacolo! Aggiorniamo Blackboard
                                msg.type = MSG_TYPE_DRONE;
                                snprintf(msg.data, sizeof(msg.data), "%f %f", rx, ry);
                                write(fd_bb_out, &msg, sizeof(msg));

                                // Inviamo ACK: "pok <obstacle>" cioè "pok x y"
                                send_msg(net_fd, "pok %f %f", rx, ry);
                                
                                // Ciclo completato, ricomincia
                                net_state = SV_SEND_CMD_DRONE;
                                state_changed = 1; 
                            }
                        }
                        break;
                }
            } 
            else { // CLIENT
                switch (net_state) {
                    case CL_WAIT_COMMAND:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (strcmp(net_line, "drone") == 0) {
                                net_state = CL_WAIT_DRONE_DATA;
                                state_changed = 1;
                            } 
                            else if (strcmp(net_line, "obst") == 0) {
                                net_state = CL_SEND_OBST_DATA;
                                state_changed = 1;
                            }
                            else if (strcmp(net_line, "q") == 0) {
                                send_msg(net_fd, "qok");
                                goto exit_loop;
                            }
                        }
                        break;

                    case CL_WAIT_DRONE_DATA:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            // Aspetta "x y"
                            if (sscanf(net_line, "%f %f", &rx, &ry) == 2) {
                                // Ricevuto Drone Server! Aggiorna BB
                                msg.type = MSG_TYPE_DRONE;
                                snprintf(msg.data, sizeof(msg.data), "%f %f", rx, ry);
                                write(fd_bb_out, &msg, sizeof(msg));

                                // Invia Ack "dok <drone>"
                                send_msg(net_fd, "dok %f %f", rx, ry);
                                
                                net_state = CL_WAIT_COMMAND;
                                // Non forziamo state_changed, aspettiamo prossimo cmd
                            }
                        }
                        break;

                    case CL_SEND_OBST_DATA:
                        // Invia i PROPRI dati "x y"
                        send_msg(net_fd, "%f %f", my_last_x, my_last_y);
                        net_state = CL_WAIT_POK;
                        // Aspettiamo ack dalla rete
                        break;

                    case CL_WAIT_POK:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            // Aspetta "pok x y"
                            if (sscanf(net_line, "pok %f %f", &rx, &ry) == 2) {
                                // Ack ricevuto, pronto per prossimo comando
                                net_state = CL_WAIT_COMMAND;
                                state_changed = 1; // Controlla subito se ci sono altri comandi
                            }
                        }
                        break;
                }
            }

        } while (state_changed);
    }

exit_loop:
    if (net_fd >= 0) close(net_fd);
    logMessage(LOG_PATH, "[NET] Loop finished.");
}

int main(int argc, char *argv[]) {
    signal(SIGPIPE, SIG_IGN); 

    if (argc < 6) return 1;

    int fd_bb_in = atoi(argv[1]);   
    int fd_bb_out = atoi(argv[2]);  
    int mode = atoi(argv[3]);
    const char *addr = (argc > 4) ? argv[4] : "127.0.0.1";
    int port = atoi(argv[5]);
    int w = 100, h = 100;

    if (mode == MODE_SERVER) {
        receive_window_size(fd_bb_in, &w, &h);
        net_fd = init_server(port);
    } else {
        net_fd = init_client(addr, port);
    }

    if (net_fd < 0 || protocol_handshake(mode, net_fd, &w, &h, fd_bb_out) < 0) {
        logMessage(LOG_PATH, "[NET] Init Failed.");
        return 1;
    }

    network_loop(mode, fd_bb_in, fd_bb_out);
    return 0;
}