#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "app_common.h"
#include "log.h"

#define PORT 5000
#define BACKLOG 5
#define BUFSZ 256

void die(const char *msg) {
    perror(msg);
    exit(EXIT_FAILURE);
}

ssize_t send_msg(int fd, const char *msg) {
    ssize_t n = write(fd, msg, strlen(msg));
    if (n < 0) die("write");
    logMessage(LOG_PATH_SC, "[SERVER] Sent: '%s'", msg);
    return n;
}

ssize_t recv_msg(int fd, char *buf) {
    bzero(buf, BUFSZ);
    ssize_t n = read(fd, buf, BUFSZ - 1);
    if (n < 0) die("read");
    buf[n] = '\0';
    logMessage(LOG_PATH_SC, "[SERVER] Received: '%s'", buf);
    return n;
}

int main(void) {
    int sockfd, client_fd = -1, bb_fd = -1;
    struct sockaddr_in serv_addr, cli_addr;
    socklen_t clilen;
    char buf[BUFSZ];

    /* 1. SOCKET */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) die("socket");

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* 2. BIND */
    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family      = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port        = htons(PORT);

    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        die("bind");

    /* 3. LISTEN */
    if (listen(sockfd, BACKLOG) < 0) die("listen");
    logMessage(LOG_PATH_SC, "[SERVER] Listening on port %d", PORT);

    /* 4. ACCEPT BB AND CLIENT */
    clilen = sizeof(cli_addr);
    while (bb_fd < 0 || client_fd < 0) {
        int fd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
        if (fd < 0) die("accept");

        recv_msg(fd, buf);

        // Rimuove eventuale newline alla fine
        buf[strcspn(buf, "\r\n")] = 0;  

        if (strncmp(buf, "BB", 2) == 0 && bb_fd < 0) {
            bb_fd = fd;
            logMessage(LOG_PATH_SC, "[SERVER] Blackboard connected");
        } else if (strncmp(buf, "CLIENT", 6) == 0 && client_fd < 0) {
            client_fd = fd;
            logMessage(LOG_PATH_SC, "[SERVER] Client connected");
        } else {
            logMessage(LOG_PATH_SC, "[SERVER] Unknown client, closing: '%s'", buf);
            close(fd);
        }

    }

    /* 5. HANDSHAKE CLIENT */
    send_msg(client_fd, "ok\n");
    recv_msg(client_fd, buf);  // expect "ook"

    /* 6. SIZE PASS THROUGH FROM BB */
    recv_msg(bb_fd, buf);       // size l h
    logMessage(LOG_PATH_SC, "[SERVER] Size received from BB: %s", buf);
    send_msg(client_fd, buf);   // forward to client
    recv_msg(client_fd, buf);   // expect "sok"

    /* 7. MAIN LOOP */
    while (1) {
        /* DRONE */
        recv_msg(bb_fd, buf);
        logMessage(LOG_PATH_SC, "[SERVER] Drone position received from BB: %s", buf);
        send_msg(client_fd, "drone\n");
        send_msg(client_fd, buf);
        recv_msg(client_fd, buf);  // expect "dok"
        logMessage(LOG_PATH_SC, "[SERVER] Drone acknowledged by client");

        /* OBSTACLE */
        send_msg(client_fd, "obst\n");
        recv_msg(client_fd, buf);  // receive x y from client
        logMessage(LOG_PATH_SC, "[SERVER] Obstacle received with coordinates: %s", buf);
        send_msg(client_fd, "pok\n");

        /* QUIT */
        send_msg(client_fd, "q\n");
        recv_msg(client_fd, buf);  // expect "qok"
        logMessage(LOG_PATH_SC, "[SERVER] Quit acknowledged, shutting down");
        break;
    }

    /* CLEANUP */
    close(bb_fd);
    close(client_fd);
    close(sockfd);
    logMessage(LOG_PATH_SC, "[SERVER] Shutdown clean");
    return 0;
}
