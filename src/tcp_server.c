/*
 * tcp_server.c - TCP Server
 * XVC Server for Digilent HS2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <time.h>
#include "tcp_server.h"
#include "logging.h"

int tcp_server_init(tcp_server_t *server, int port, whitelist_t *whitelist)
{
    if (!server) return -1;
    
    memset(server, 0, sizeof(tcp_server_t));
    server->port = port;
    server->whitelist = whitelist;
    server->listen_fd = -1;
    
    return 0;
}

int tcp_server_start(tcp_server_t *server)
{
    if (!server) return -1;
    
    /* Create socket */
    server->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server->listen_fd < 0) {
        LOG_ERROR("socket(): %s", strerror(errno));
        return -1;
    }
    
    /* Set SO_REUSEADDR */
    int optval = 1;
    if (setsockopt(server->listen_fd, SOL_SOCKET, SO_REUSEADDR, 
                   &optval, sizeof(optval)) < 0) {
        LOG_WARN("setsockopt(SO_REUSEADDR): %s", strerror(errno));
    }
    
    /* Bind */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(server->port);
    
    if (bind(server->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERROR("bind(%d): %s", server->port, strerror(errno));
        close(server->listen_fd);
        server->listen_fd = -1;
        return -1;
    }
    
    /* Listen */
    if (listen(server->listen_fd, 5) < 0) {
        LOG_ERROR("listen(): %s", strerror(errno));
        close(server->listen_fd);
        server->listen_fd = -1;
        return -1;
    }
    
    server->running = true;
    server->max_fd = server->listen_fd;
    
    LOG_INFO("TCP server listening on port %d", server->port);
    return 0;
}

void tcp_server_stop(tcp_server_t *server)
{
    if (!server) return;
    
    server->running = false;
    
    /* Close all connections */
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (server->connections[i].state != CONN_STATE_CLOSED) {
            if (server->on_disconnect) {
                server->on_disconnect(server->user_data, &server->connections[i]);
            }
            close(server->connections[i].fd);
            server->connections[i].state = CONN_STATE_CLOSED;
        }
    }
    server->connection_count = 0;
    
    /* Close listen socket */
    if (server->listen_fd >= 0) {
        close(server->listen_fd);
        server->listen_fd = -1;
    }
    
    LOG_INFO("TCP server stopped");
}

static tcp_connection_t* find_free_slot(tcp_server_t *server)
{
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (server->connections[i].state == CONN_STATE_CLOSED) {
            return &server->connections[i];
        }
    }
    return NULL;
}

static void update_max_fd(tcp_server_t *server)
{
    server->max_fd = server->listen_fd;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (server->connections[i].state != CONN_STATE_CLOSED) {
            if (server->connections[i].fd > server->max_fd) {
                server->max_fd = server->connections[i].fd;
            }
        }
    }
}

int tcp_server_poll(tcp_server_t *server, int timeout_ms)
{
    if (!server || !server->running) return -1;
    
    fd_set read_fds, except_fds;
    FD_ZERO(&read_fds);
    FD_ZERO(&except_fds);
    
    /* Add listen socket */
    FD_SET(server->listen_fd, &read_fds);
    FD_SET(server->listen_fd, &except_fds);
    
    /* Add client sockets */
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (server->connections[i].state != CONN_STATE_CLOSED) {
            FD_SET(server->connections[i].fd, &read_fds);
            FD_SET(server->connections[i].fd, &except_fds);
        }
    }
    
    /* Set timeout */
    struct timeval tv, *tvp = NULL;
    if (timeout_ms >= 0) {
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        tvp = &tv;
    }
    
    int ret = select(server->max_fd + 1, &read_fds, NULL, &except_fds, tvp);
    if (ret < 0) {
        if (errno == EINTR) return 0;
        LOG_ERROR("select(): %s", strerror(errno));
        return -1;
    }
    
    if (ret == 0) return 0;  /* Timeout */
    
    int events = 0;
    
    /* Check for new connections */
    if (FD_ISSET(server->listen_fd, &read_fds)) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        int client_fd = accept(server->listen_fd, 
                               (struct sockaddr*)&client_addr, &addr_len);
        
        if (client_fd >= 0) {
            /* Check whitelist */
            if (server->whitelist) {
                whitelist_result_t wl_result = whitelist_check(server->whitelist,
                    (struct sockaddr*)&client_addr);
                
                if (wl_result == WHITELIST_BLOCKED) {
                    char ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
                    LOG_WARN("Connection from %s blocked by whitelist", ip);
                    close(client_fd);
                    goto check_clients;
                }
            }
            
            tcp_connection_t *conn = find_free_slot(server);
            if (!conn) {
                LOG_WARN("Maximum connections reached, rejecting");
                close(client_fd);
            } else {
                memset(conn, 0, sizeof(tcp_connection_t));
                conn->fd = client_fd;
                conn->state = CONN_STATE_CONNECTED;
                memcpy(&conn->addr, &client_addr, sizeof(client_addr));
                conn->connected_at = time(NULL);
                
                /* Set TCP_NODELAY */
                int flag = 1;
                setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, 
                           &flag, sizeof(flag));
                
                server->connection_count++;
                update_max_fd(server);
                
                char ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
                LOG_INFO("Connection accepted from %s (fd=%d)", ip, client_fd);
                
                if (server->on_connect) {
                    server->on_connect(server->user_data, conn);
                }
                
                events++;
            }
        }
    }
    
check_clients:
    /* Check existing connections */
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        tcp_connection_t *conn = &server->connections[i];
        if (conn->state == CONN_STATE_CLOSED) continue;
        
        if (FD_ISSET(conn->fd, &except_fds)) {
            /* Exception - close connection */
            LOG_INFO("Connection exception (fd=%d)", conn->fd);
            tcp_server_close_connection(server, conn);
            events++;
            continue;
        }
        
        if (FD_ISSET(conn->fd, &read_fds)) {
            /* Data available */
            conn->state = CONN_STATE_ACTIVE;
            
            if (server->on_data) {
                int result = server->on_data(server->user_data, conn);
                if (result != 0) {
                    tcp_server_close_connection(server, conn);
                }
            }
            events++;
        }
    }
    
    return events;
}

int tcp_server_run(tcp_server_t *server)
{
    if (!server) return -1;
    
    while (server->running) {
        int ret = tcp_server_poll(server, 1000);
        if (ret < 0) {
            return -1;
        }
    }
    
    return 0;
}

void tcp_server_set_callbacks(tcp_server_t *server,
                               int (*on_connect)(void*, tcp_connection_t*),
                               int (*on_data)(void*, tcp_connection_t*),
                               void (*on_disconnect)(void*, tcp_connection_t*),
                               void *user_data)
{
    if (!server) return;
    
    server->on_connect = on_connect;
    server->on_data = on_data;
    server->on_disconnect = on_disconnect;
    server->user_data = user_data;
}

void tcp_server_close_connection(tcp_server_t *server, tcp_connection_t *conn)
{
    if (!server || !conn) return;
    if (conn->state == CONN_STATE_CLOSED) return;
    
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &conn->addr.sin_addr, ip, sizeof(ip));
    LOG_INFO("Closing connection from %s (fd=%d)", ip, conn->fd);
    
    if (server->on_disconnect) {
        server->on_disconnect(server->user_data, conn);
    }
    
    close(conn->fd);
    conn->state = CONN_STATE_CLOSED;
    conn->fd = -1;
    server->connection_count--;
    
    update_max_fd(server);
}

const char* tcp_connection_ip(const tcp_connection_t *conn, char *buf, size_t len)
{
    if (!conn || !buf || len == 0) return NULL;
    return inet_ntop(AF_INET, &conn->addr.sin_addr, buf, len);
}
