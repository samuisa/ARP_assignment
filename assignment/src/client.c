#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include "log.h"
#include "app_common.h"

#define BUFSZ 256

void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

void send_msg(int fd, const char *msg) {
    ssize_t n = write(fd, msg, strlen(msg));
    if (n < 0) die("write");
    logMessage(LOG_PATH_SC, "[CLIENT] Sent: '%s'", msg);
}

void recv_msg(int fd, char *buf) {
    bzero(buf, BUFSZ);
    ssize_t n = read(fd, buf, BUFSZ - 1);
    if (n < 0) die("read");
    buf[n] = '\0';
    logMessage(LOG_PATH_SC, "[CLIENT] Received: '%s'", buf);
}

/* ==================================================================================
 * Connect to server at host:port
 * ================================================================================== */
int connect_to_server(const char *host, int port, const char *identity) {
    int sockfd;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) die("socket");

    server = gethostbyname(host);
    if (!server) die("gethostbyname");

    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
    serv_addr.sin_port = htons(port);

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        die("connect");

    logMessage(LOG_PATH_SC, "[CLIENT] Connected to %s:%d", host, port);
    send_msg(sockfd, identity);
    logMessage(LOG_PATH_SC, "[CLIENT] Sent identity '%s'", identity);

    return sockfd;
}

/* ==================================================================================
 * Setup listening socket for blackboard
 * ================================================================================== */
int setup_bb_listener(int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) die("socket");

    struct sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) die("bind");
    if (listen(sockfd, 1) < 0) die("listen");

    logMessage(LOG_PATH_SC, "[CLIENT] Listening for Blackboard on port %d", port);
    return sockfd;
}

/* ==================================================================================
 * Accept blackboard connection
 * ================================================================================== */
int accept_bb(int listen_fd) {
    int bb_fd = accept(listen_fd, NULL, NULL);
    if (bb_fd < 0) die("accept");
    logMessage(LOG_PATH_SC, "[CLIENT] Blackboard connected");
    send_msg(bb_fd, "CLIENT\n"); // identify to BB
    return bb_fd;
}

/* ==================================================================================
 * MAIN
 * ================================================================================== */
int main() {
    char buffer[BUFSZ];

    // 1. Connect to server (porta 5000)
    int server_fd = connect_to_server("localhost", 5000, "CLIENT\n");

    // Handshake con server
    recv_msg(server_fd, buffer); // ok
    send_msg(server_fd, "ook\n");

    // Ricevi size dal server
    recv_msg(server_fd, buffer); // size l h
    logMessage(LOG_PATH_SC, "[CLIENT] Size received: %s", buffer);
    send_msg(server_fd, "sok\n");

    // 2. Setup listening socket per Blackboard (porta 5001)
    int bb_listen_fd = setup_bb_listener(5001);
    int bb_fd = accept_bb(bb_listen_fd);
    close(bb_listen_fd); // non serve più il socket di ascolto

    // 3. MAIN LOOP
    int running = 1;
    while (running) {
        // Ricevo messaggi dal server
        recv_msg(server_fd, buffer);

        if (strncmp(buffer, "drone", 5) == 0) {
            // Ricevo posizione drone dal server
            recv_msg(server_fd, buffer);  // x y
            logMessage(LOG_PATH_SC, "[CLIENT] Drone position received: %s", buffer);
            send_msg(server_fd, "dok\n"); // acknowledge
        }
        else if (strncmp(buffer, "obst", 4) == 0) {
            logMessage(LOG_PATH_SC, "[CLIENT] Obstacle request received from server");

            // Richiedo array di ostacoli alla BB
            send_msg(bb_fd, "send_obst\n");  // trigger BB per inviare ostacoli
            recv_msg(bb_fd, buffer);         // leggo ostacoli (ad esempio "x1 y1;x2 y2;...")
            logMessage(LOG_PATH_SC, "[CLIENT] Obstacles received from BB: %s", buffer);

            // Inoltro ostacoli al server
            send_msg(server_fd, buffer);
            recv_msg(server_fd, buffer);     // attendo "pok"
            if (strncmp(buffer, "pok", 3) == 0) {
                logMessage(LOG_PATH_SC, "[CLIENT] Obstacle acknowledged by server");
            }
        }
        else if (strncmp(buffer, "q", 1) == 0) {
            // Quit
            send_msg(server_fd, "qok\n");
            logMessage(LOG_PATH_SC, "[CLIENT] Quit received from server, exiting");
            running = 0;
        }
    }

    close(server_fd);
    close(bb_fd);
    logMessage(LOG_PATH_SC, "[CLIENT] Connections closed, client terminated");

    return 0;
}
