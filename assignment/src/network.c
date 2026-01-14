/* ======================================================================================
 * FILE: network.c
 * Protocollo STRETTO: Request -> Datum -> Ack
 * Separatore: \n (Newline strict)
 * Modalità: Non-Blocking I/O Multiplexing
 * WITH VERBOSE LOGGING (LOG_PATH_SC)
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

// --- STATI DEL PROTOCOLLO ---
typedef enum {
    SV_SEND_CMD_DRONE,   
    SV_SEND_DATA_DRONE,  
    SV_WAIT_DOK,         
    SV_SEND_CMD_OBST,    
    SV_WAIT_DATA_OBST,   
    CL_WAIT_COMMAND,     
    CL_WAIT_DRONE_DATA,  
    CL_SEND_OBST_DATA,   
    CL_WAIT_POK          
} NetState;

static NetState net_state;
static int net_fd = -1;

// Buffer per socket non bloccante
typedef struct {
    char data[BUFSZ];
    int len;
} SocketBuffer;

static SocketBuffer sock_buf = { .len = 0 };

static float my_last_x = 0.0f;
static float my_last_y = 0.0f;

// --- UTILITY ---

const char* state_to_str(NetState s) {
    switch(s) {
        case SV_SEND_CMD_DRONE: return "SV_SEND_CMD_DRONE";
        case SV_SEND_DATA_DRONE: return "SV_SEND_DATA_DRONE";
        case SV_WAIT_DOK: return "SV_WAIT_DOK";
        case SV_SEND_CMD_OBST: return "SV_SEND_CMD_OBST";
        case SV_WAIT_DATA_OBST: return "SV_WAIT_DATA_OBST";
        case CL_WAIT_COMMAND: return "CL_WAIT_COMMAND";
        case CL_WAIT_DRONE_DATA: return "CL_WAIT_DRONE_DATA";
        case CL_SEND_OBST_DATA: return "CL_SEND_OBST_DATA";
        case CL_WAIT_POK: return "CL_WAIT_POK";
        default: return "UNKNOWN";
    }
}

void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    logMessage(LOG_PATH_SC, "[NET] FD %d set to non-blocking", fd);
}

// --- FUNZIONE CHIAVE PER INVIO CON \n ---
void send_msg(int fd, const char *fmt, ...) {
    char buf[BUFSZ];
    va_list args;
    
    // 1. Format stringa nel buffer, lasciando spazio per \n e \0
    va_start(args, fmt);
    // Usiamo sizeof(buf) - 2 per garantire spazio per '\n' e '\0'
    vsnprintf(buf, sizeof(buf) - 2, fmt, args);
    va_end(args);

    int len = strlen(buf);
    
    // 2. Logghiamo il contenuto "pulito" (senza ancora il newline forzato se manca)
    logMessage(LOG_PATH_SC, "[NET-OUT] Sending raw data: '%s'", buf);

    // 3. FORCE NEWLINE: Se non c'è, lo aggiungiamo.
    // Il protocollo richiede \n come terminatore, non \0.
    if (len == 0 || buf[len-1] != '\n') {
        buf[len] = '\n';
        buf[len+1] = '\0'; // Terminatore C solo per sicurezza locale
        len++;
    }

    // 4. WRITE: Scriviamo 'len' byte. 
    // Se buf è "ciao\n\0", len è 5. Write invia 'c','i','a','o','\n'.
    // Il terminatore '\0' NON viene inviato.
    ssize_t sent = write(fd, buf, len);
    
    if (sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EPIPE)
            logMessage(LOG_PATH_SC, "[NET] ERROR sending: %s", strerror(errno));
    }
}

// --- RICEZIONE E PARSING TRAMITE \n ---

int read_socket_chunk(int fd) {
    if (sock_buf.len >= BUFSZ - 1) {
        logMessage(LOG_PATH_SC, "[NET-ERR] Buffer full! Cannot read more.");
        return 0; 
    }
    // Leggiamo byte grezzi dalla rete
    ssize_t n = read(fd, sock_buf.data + sock_buf.len, BUFSZ - 1 - sock_buf.len);
    if (n > 0) {
        sock_buf.len += n;
        // Aggiungiamo \0 SOLO per permettere a strchr di funzionare localmente.
        // Questo \0 non viene dalla rete (che manda solo \n).
        sock_buf.data[sock_buf.len] = '\0';
        return 1; 
    }
    if (n == 0) logMessage(LOG_PATH_SC, "[NET-IN] Connection closed by peer (read 0).");
    return (n == 0) ? -1 : 0;
}

int get_line_from_buffer(char *out_line, int max_len) {
    // Cerchiamo ESPLICITAMENTE il newline
    char *newline_ptr = strchr(sock_buf.data, '\n');
    
    if (newline_ptr) {
        // Calcoliamo la lunghezza della riga ESCLUDENDO il \n
        int line_len = newline_ptr - sock_buf.data;
        
        if (line_len >= max_len) line_len = max_len - 1;
        
        // Copiamo i dati utili
        memcpy(out_line, sock_buf.data, line_len);
        
        // Aggiungiamo \0 locale per poter usare sscanf
        out_line[line_len] = '\0';
        
        logMessage(LOG_PATH_SC, "[NET-PARSE] Extracted line (via \\n): '%s'", out_line);

        // Spostiamo il resto del buffer (saltando il \n appena processato)
        int remaining = sock_buf.len - (newline_ptr - sock_buf.data) - 1;
        memmove(sock_buf.data, newline_ptr + 1, remaining);
        sock_buf.len = remaining;
        sock_buf.data[sock_buf.len] = '\0';
        return 1;
    }
    return 0;
}

// Lettura bloccante byte per byte fino a \n (per handshake)
int read_line_blocking(int fd, char *out, int out_sz) {
    int pos = 0; char c;
    while (pos < out_sz - 1) {
        if (read(fd, &c, 1) <= 0) return -1;
        if (c == '\n') break; // Stop al newline
        out[pos++] = c;
    }
    out[pos] = '\0'; // Null-terminate locale
    logMessage(LOG_PATH_SC, "[HANDSHAKE] Blocking read: '%s'", out);
    return pos;
}

// --- CONNESSIONE & HANDSHAKE ---

int init_server(int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1; setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
    
    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        logMessage(LOG_PATH_SC, "[NET-ERR] Bind failed: %s", strerror(errno));
        return -1;
    }
    listen(s, 1);
    logMessage(LOG_PATH_SC, "[NET-SRV] Waiting for connection on port %d...", port);
    
    struct sockaddr_in cli;
    socklen_t len = sizeof(cli);
    int client_fd = accept(s, (struct sockaddr*)&cli, &len);
    if (client_fd >= 0) {
        logMessage(LOG_PATH_SC, "[NET-SRV] Accepted connection from %s", inet_ntoa(cli.sin_addr));
    }
    return client_fd;
}

int init_client(const char *addr, int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; a.sin_port = htons(port);
    inet_pton(AF_INET, addr, &a.sin_addr);
    
    logMessage(LOG_PATH_SC, "[NET-CLI] Connecting to %s:%d ...", addr, port);
    while (connect(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        logMessage(LOG_PATH_SC, "[NET-CLI] Retry in 1s...");
        sleep(1);
    }
    logMessage(LOG_PATH_SC, "[NET-CLI] Connected!");
    return s;
}

void send_window_size(int fd_out, int w, int h) {
    Message msg; msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", w, h);
    write(fd_out, &msg, sizeof(msg));  
    logMessage(LOG_PATH_SC, "[BB-OUT] Sent Window Size: %d %d", w, h);
}

void receive_window_size(int fd_in, int *w, int *h){
    Message msg;
    if (read(fd_in, &msg, sizeof(msg)) > 0) {
        sscanf(msg.data, "%d %d", w, h);
        logMessage(LOG_PATH_SC, "[BB-IN] Received Window Size: %d %d", *w, *h);
    }
}

void update_local_position(int fd_in) {
    Message msg;
    while (read(fd_in, &msg, sizeof(msg)) > 0) {
        sscanf(msg.data, "%f %f", &my_last_x, &my_last_y);
    }
}

int protocol_handshake(int mode, int fd, int *w, int *h, int fd_bb_out) {
    char buf[BUFSZ];
    logMessage(LOG_PATH_SC, "[HANDSHAKE] Start Mode: %s", mode == MODE_SERVER ? "SERVER" : "CLIENT");
    
    // NOTA: send_msg aggiunge \n automaticamente.
    // read_line_blocking legge fino a \n e lo scarta.
    
    if (mode == MODE_SERVER) {
        send_msg(fd, "ok"); // Invia "ok\n"
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ook") != 0) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'ook', got '%s'", buf);
            return -1;
        }
        send_msg(fd, "size %d %d", *w, *h); // Invia "size w h\n"
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "sok %d %d", w, h) != 2) {
             logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'sok', got '%s'", buf);
             return -1;
        }
    } else {
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ok") != 0) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'ok', got '%s'", buf);
            return -1;
        }
        send_msg(fd, "ook"); // Invia "ook\n"
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "size %d %d", w, h) != 2) {
             logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'size', got '%s'", buf);
             return -1;
        }
        send_window_size(fd_bb_out, *w, *h);
        send_msg(fd, "sok %d %d", *w, *h); // Invia "sok w h\n"
    }
    
    net_state = (mode == MODE_SERVER) ? SV_SEND_CMD_DRONE : CL_WAIT_COMMAND;
    logMessage(LOG_PATH_SC, "[HANDSHAKE] Done. State: %s", state_to_str(net_state));
    return 0;
}

// --- MAIN LOOP ---

void network_loop(int mode, int fd_bb_in, int fd_bb_out) {
    char net_line[BUFSZ];
    float rx, ry; 
    Message msg; 
    fd_set read_fds;
    struct timeval timeout; 

    set_nonblocking(net_fd);
    set_nonblocking(fd_bb_in);

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(net_fd, &read_fds);
        FD_SET(fd_bb_in, &read_fds);
        
        int max_fd = (net_fd > fd_bb_in) ? net_fd : fd_bb_in;
        int has_buf = (strchr(sock_buf.data, '\n') != NULL);
        timeout.tv_sec = 0;
        timeout.tv_usec = has_buf ? 0 : 2000; 

        if (select(max_fd + 1, &read_fds, NULL, NULL, &timeout) < 0 && errno != EINTR) {
             logMessage(LOG_PATH_SC, "[NET-ERR] Select failed: %s", strerror(errno));
             break;
        }

        if (FD_ISSET(fd_bb_in, &read_fds)) update_local_position(fd_bb_in);

        if (FD_ISSET(net_fd, &read_fds)) {
            if (read_socket_chunk(net_fd) == -1) {
                logMessage(LOG_PATH_SC, "[NET] Socket closed.");
                goto exit_loop;
            }
        }

        int state_changed;
        do {
            state_changed = 0;
            if (mode == MODE_SERVER) {
                switch (net_state) {
                    case SV_SEND_CMD_DRONE:
                        logMessage(LOG_PATH_SC, "[SV] >> Sending 'drone'");
                        send_msg(net_fd, "drone");
                        net_state = SV_SEND_DATA_DRONE;
                        state_changed = 1; 
                        break;
                    case SV_SEND_DATA_DRONE:
                        send_msg(net_fd, "%f %f", my_last_x, my_last_y);
                        net_state = SV_WAIT_DOK;
                        break;
                    case SV_WAIT_DOK:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "dok %f %f", &rx, &ry) == 2) {
                                logMessage(LOG_PATH_SC, "[SV] << ACK 'dok'");
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
                            if (sscanf(net_line, "%f %f", &rx, &ry) == 2) {
                                logMessage(LOG_PATH_SC, "[SV] << Obst Data");
                                msg.type = MSG_TYPE_DRONE;
                                snprintf(msg.data, sizeof(msg.data), "%f %f", rx, ry);
                                write(fd_bb_out, &msg, sizeof(msg));
                                send_msg(net_fd, "pok %f %f", rx, ry);
                                net_state = SV_SEND_CMD_DRONE;
                                state_changed = 1; 
                            }
                        }
                        break;
                }
            } else { // CLIENT
                switch (net_state) {
                    case CL_WAIT_COMMAND:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (strcmp(net_line, "drone") == 0) {
                                net_state = CL_WAIT_DRONE_DATA;
                                state_changed = 1;
                            } else if (strcmp(net_line, "obst") == 0) {
                                net_state = CL_SEND_OBST_DATA;
                                state_changed = 1;
                            } else if (strcmp(net_line, "q") == 0) {
                                send_msg(net_fd, "qok");
                                goto exit_loop;
                            }
                        }
                        break;
                    case CL_WAIT_DRONE_DATA:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "%f %f", &rx, &ry) == 2) {
                                msg.type = MSG_TYPE_DRONE;
                                snprintf(msg.data, sizeof(msg.data), "%f %f", rx, ry);
                                write(fd_bb_out, &msg, sizeof(msg));
                                send_msg(net_fd, "dok %f %f", rx, ry);
                                net_state = CL_WAIT_COMMAND;
                            }
                        }
                        break;
                    case CL_SEND_OBST_DATA:
                        send_msg(net_fd, "%f %f", my_last_x, my_last_y);
                        net_state = CL_WAIT_POK;
                        break;
                    case CL_WAIT_POK:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "pok %f %f", &rx, &ry) == 2) {
                                net_state = CL_WAIT_COMMAND;
                                state_changed = 1; 
                            }
                        }
                        break;
                }
            }
        } while (state_changed);
    }

exit_loop:
    if (net_fd >= 0) close(net_fd);
    logMessage(LOG_PATH_SC, "[NET] Loop finished.");
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
        logMessage(LOG_PATH_SC, "[NET-FATAL] Init Failed.");
        return 1;
    }

    network_loop(mode, fd_bb_in, fd_bb_out);
    return 0;
}