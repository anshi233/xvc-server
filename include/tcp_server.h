/*
 * tcp_server.h - TCP Server
 * XVC Server for Digilent HS2
 */

#ifndef XVC_TCP_SERVER_H
#define XVC_TCP_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>
#include "whitelist.h"

/* Maximum concurrent connections per instance */
#define MAX_CONNECTIONS 16

/* Connection state */
typedef enum {
    CONN_STATE_CLOSED = 0,
    CONN_STATE_CONNECTED,
    CONN_STATE_ACTIVE
} conn_state_t;

/* Client connection */
typedef struct {
    int fd;
    conn_state_t state;
    struct sockaddr_in addr;
    time_t connected_at;
    uint64_t bytes_rx;
    uint64_t bytes_tx;
} tcp_connection_t;

/* TCP server context */
typedef struct {
    int listen_fd;
    int port;
    bool running;
    
    /* Connections */
    tcp_connection_t connections[MAX_CONNECTIONS];
    int connection_count;
    int max_fd;
    
    /* Whitelist */
    whitelist_t *whitelist;
    
    /* Callbacks */
    void *user_data;
    int (*on_connect)(void *user_data, tcp_connection_t *conn);
    int (*on_data)(void *user_data, tcp_connection_t *conn);
    void (*on_disconnect)(void *user_data, tcp_connection_t *conn);
} tcp_server_t;

/* TCP Server API */

/**
 * Initialize TCP server
 * @param server Server context
 * @param port Port to listen on
 * @param whitelist Whitelist (can be NULL)
 * @return 0 on success, -1 on error
 */
int tcp_server_init(tcp_server_t *server, int port, whitelist_t *whitelist);

/**
 * Start listening
 * @param server Server context
 * @return 0 on success, -1 on error
 */
int tcp_server_start(tcp_server_t *server);

/**
 * Stop server
 */
void tcp_server_stop(tcp_server_t *server);

/**
 * Run server event loop (blocking)
 * @param server Server context
 * @return 0 on clean shutdown, -1 on error
 */
int tcp_server_run(tcp_server_t *server);

/**
 * Process events once (non-blocking)
 * @param server Server context
 * @param timeout_ms Timeout in milliseconds (-1 for infinite)
 * @return Number of events processed, or -1 on error
 */
int tcp_server_poll(tcp_server_t *server, int timeout_ms);

/**
 * Set callbacks
 */
void tcp_server_set_callbacks(tcp_server_t *server,
                               int (*on_connect)(void*, tcp_connection_t*),
                               int (*on_data)(void*, tcp_connection_t*),
                               void (*on_disconnect)(void*, tcp_connection_t*),
                               void *user_data);

/**
 * Close connection
 */
void tcp_server_close_connection(tcp_server_t *server, tcp_connection_t *conn);

/**
 * Get client IP as string
 */
const char* tcp_connection_ip(const tcp_connection_t *conn, char *buf, size_t len);

#endif /* XVC_TCP_SERVER_H */
