/*
 * xvc_protocol.h - XVC Protocol Handler
 * XVC Server for Digilent HS2
 */

#ifndef XVC_PROTOCOL_H
#define XVC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* XVC protocol version */
#define XVC_VERSION "xvcServer_v1.0"

/* Maximum vector size in bytes (TMS + TDI combined) */
#define XVC_MAX_VECTOR_SIZE 2048

/* JTAG states */
typedef enum {
    JTAG_TEST_LOGIC_RESET = 0,
    JTAG_RUN_TEST_IDLE,
    JTAG_SELECT_DR_SCAN,
    JTAG_CAPTURE_DR,
    JTAG_SHIFT_DR,
    JTAG_EXIT1_DR,
    JTAG_PAUSE_DR,
    JTAG_EXIT2_DR,
    JTAG_UPDATE_DR,
    JTAG_SELECT_IR_SCAN,
    JTAG_CAPTURE_IR,
    JTAG_SHIFT_IR,
    JTAG_EXIT1_IR,
    JTAG_PAUSE_IR,
    JTAG_EXIT2_IR,
    JTAG_UPDATE_IR,
    JTAG_NUM_STATES
} jtag_state_t;

/* XVC command types */
typedef enum {
    XVC_CMD_UNKNOWN = 0,
    XVC_CMD_GETINFO,
    XVC_CMD_SETTCK,
    XVC_CMD_SHIFT
} xvc_cmd_type_t;

/* Forward declaration */
struct ftdi_context_s;

/* XVC protocol context */
typedef struct {
    int socket_fd;
    jtag_state_t jtag_state;
    bool seen_tlr;          /* Seen test-logic-reset */
    uint32_t frequency;     /* Current TCK frequency */
    
    /* FTDI adapter reference */
    struct ftdi_context_s *ftdi;
    
    /* Buffers */
    uint8_t cmd_buf[16];
    uint8_t vector_buf[XVC_MAX_VECTOR_SIZE];
    uint8_t result_buf[XVC_MAX_VECTOR_SIZE];
    
    /* Statistics */
    uint64_t bytes_rx;
    uint64_t bytes_tx;
    uint64_t commands;
} xvc_context_t;

/* XVC Protocol API */

/**
 * Initialize XVC protocol context
 * @param ctx XVC context
 * @param socket_fd Client socket
 * @param ftdi FTDI adapter context
 * @return 0 on success, -1 on error
 */
int xvc_init(xvc_context_t *ctx, int socket_fd, struct ftdi_context_s *ftdi);

/**
 * Handle incoming data from client
 * @param ctx XVC context
 * @param frequency Force frequency (0 to use client-specified)
 * @return 0 to continue, 1 to close connection, -1 on error
 */
int xvc_handle(xvc_context_t *ctx, uint32_t frequency);

/**
 * Close XVC context
 */
void xvc_close(xvc_context_t *ctx);

/**
 * Step JTAG state machine
 * @param state Current state
 * @param tms TMS value
 * @return New state
 */
jtag_state_t jtag_step(jtag_state_t state, int tms);

/**
 * Get JTAG state name
 */
const char* jtag_state_name(jtag_state_t state);

/**
 * Read exact number of bytes from socket
 * @param fd Socket file descriptor
 * @param buf Buffer
 * @param len Bytes to read
 * @return 1 on success, 0 on EOF, -1 on error
 */
int xvc_read_exact(int fd, void *buf, int len);

/**
 * Write exact number of bytes to socket
 * @param fd Socket file descriptor  
 * @param buf Buffer
 * @param len Bytes to write
 * @return 0 on success, -1 on error
 */
int xvc_write_exact(int fd, const void *buf, int len);

/**
 * Read 32-bit little-endian integer from buffer
 */
int32_t xvc_get_int32(const uint8_t *data);

/**
 * Write 32-bit little-endian integer to buffer
 */
void xvc_put_int32(uint8_t *data, int32_t value);

#endif /* XVC_PROTOCOL_H */
