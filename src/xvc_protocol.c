/*
 * xvc_protocol.c - XVC Protocol Handler
 * XVC Server for Digilent HS2
 * Based on reference implementation in xvcd/src/xvcd.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "xvc_protocol.h"
#include "ftdi_adapter.h"
#include "logging.h"

/* Helper to dump buffer in trace mode */
static void log_buffer(const char *prefix, const uint8_t *buf, int len)
{
    if (!log_enabled(XVC_LOG_TRACE)) return;

    char line[256];
    int pos = 0;
    for (int i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02x ", buf[i]);
        /* Flush line every 32 bytes or at end */
        if ((i + 1) % 16 == 0 || i == len - 1) {
            LOG_TRACE("%s: %s", prefix, line);
            pos = 0;
            line[0] = '\0';
        }
    }
}

/* JTAG state machine transitions */
static const int jtag_next_state[JTAG_NUM_STATES][2] = {
    [JTAG_TEST_LOGIC_RESET] = {JTAG_RUN_TEST_IDLE, JTAG_TEST_LOGIC_RESET},
    [JTAG_RUN_TEST_IDLE]    = {JTAG_RUN_TEST_IDLE, JTAG_SELECT_DR_SCAN},
    [JTAG_SELECT_DR_SCAN]   = {JTAG_CAPTURE_DR, JTAG_SELECT_IR_SCAN},
    [JTAG_CAPTURE_DR]       = {JTAG_SHIFT_DR, JTAG_EXIT1_DR},
    [JTAG_SHIFT_DR]         = {JTAG_SHIFT_DR, JTAG_EXIT1_DR},
    [JTAG_EXIT1_DR]         = {JTAG_PAUSE_DR, JTAG_UPDATE_DR},
    [JTAG_PAUSE_DR]         = {JTAG_PAUSE_DR, JTAG_EXIT2_DR},
    [JTAG_EXIT2_DR]         = {JTAG_SHIFT_DR, JTAG_UPDATE_DR},
    [JTAG_UPDATE_DR]        = {JTAG_RUN_TEST_IDLE, JTAG_SELECT_DR_SCAN},
    [JTAG_SELECT_IR_SCAN]   = {JTAG_CAPTURE_IR, JTAG_TEST_LOGIC_RESET},
    [JTAG_CAPTURE_IR]       = {JTAG_SHIFT_IR, JTAG_EXIT1_IR},
    [JTAG_SHIFT_IR]         = {JTAG_SHIFT_IR, JTAG_EXIT1_IR},
    [JTAG_EXIT1_IR]         = {JTAG_PAUSE_IR, JTAG_UPDATE_IR},
    [JTAG_PAUSE_IR]         = {JTAG_PAUSE_IR, JTAG_EXIT2_IR},
    [JTAG_EXIT2_IR]         = {JTAG_SHIFT_IR, JTAG_UPDATE_IR},
    [JTAG_UPDATE_IR]        = {JTAG_RUN_TEST_IDLE, JTAG_SELECT_DR_SCAN}
};

/* JTAG state names */
static const char *jtag_state_names[] = {
    "TEST_LOGIC_RESET", "RUN_TEST_IDLE",
    "SELECT_DR_SCAN", "CAPTURE_DR", "SHIFT_DR",
    "EXIT1_DR", "PAUSE_DR", "EXIT2_DR", "UPDATE_DR",
    "SELECT_IR_SCAN", "CAPTURE_IR", "SHIFT_IR",
    "EXIT1_IR", "PAUSE_IR", "EXIT2_IR", "UPDATE_IR"
};

int xvc_read_exact(int fd, void *buf, int len)
{
    uint8_t *p = buf;
    while (len > 0) {
        int r = read(fd, p, len);
        if (r <= 0) return r;
        p += r;
        len -= r;
    }
    return 1;
}

int xvc_write_exact(int fd, const void *buf, int len)
{
    const uint8_t *p = buf;
    while (len > 0) {
        int w = write(fd, p, len);
        if (w <= 0) return -1;
        p += w;
        len -= w;
    }
    return 0;
}

int32_t xvc_get_int32(const uint8_t *data)
{
    /* Little-endian */
    return (int32_t)data[0] | 
           ((int32_t)data[1] << 8) |
           ((int32_t)data[2] << 16) |
           ((int32_t)data[3] << 24);
}

void xvc_put_int32(uint8_t *data, int32_t value)
{
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
}

jtag_state_t jtag_step(jtag_state_t state, int tms)
{
    return jtag_next_state[state][tms ? 1 : 0];
}

const char* jtag_state_name(jtag_state_t state)
{
    if (state >= 0 && state < JTAG_NUM_STATES) {
        return jtag_state_names[state];
    }
    return "UNKNOWN";
}

int xvc_init(xvc_context_t *ctx, int socket_fd, struct ftdi_context_s *ftdi)
{
    if (!ctx) return -1;
    
    memset(ctx, 0, sizeof(xvc_context_t));
    ctx->socket_fd = socket_fd;
    ctx->ftdi = ftdi;
    ctx->jtag_state = JTAG_TEST_LOGIC_RESET;
    ctx->seen_tlr = false;
    
    return 0;
}

void xvc_close(xvc_context_t *ctx)
{
    if (!ctx) return;
    
    LOG_DBG("XVC session closed: rx=%lu tx=%lu cmds=%lu",
              ctx->bytes_rx, ctx->bytes_tx, ctx->commands);
}

int xvc_handle(xvc_context_t *ctx, uint32_t frequency)
{
    if (!ctx) return -1;
    
    char xvc_info[64];
    snprintf(xvc_info, sizeof(xvc_info), "%s:%d\n", XVC_VERSION, XVC_MAX_VECTOR_SIZE);
    
    do {
        /* Read command (first 2 bytes) */
        if (xvc_read_exact(ctx->socket_fd, ctx->cmd_buf, 2) != 1) {
            return 1;  /* Connection closed */
        }
        ctx->bytes_rx += 2;
        
        /* Handle getinfo */
        if (memcmp(ctx->cmd_buf, "ge", 2) == 0) {
            if (xvc_read_exact(ctx->socket_fd, ctx->cmd_buf + 2, 6) != 1) {
                return 1;
            }
            ctx->bytes_rx += 6;
            
            if (xvc_write_exact(ctx->socket_fd, xvc_info, strlen(xvc_info)) < 0) {
                LOG_ERROR("Write failed for getinfo");
                return -1;
            }
            ctx->bytes_tx += strlen(xvc_info);
            ctx->commands++;
            
            LOG_DBG("getinfo: replied with '%s'", xvc_info);
            break;
        }
        
        /* Handle settck */
        if (memcmp(ctx->cmd_buf, "se", 2) == 0) {
            if (xvc_read_exact(ctx->socket_fd, ctx->cmd_buf + 2, 9) != 1) {
                return 1;
            }
            ctx->bytes_rx += 9;
            
            int32_t period;
            if (frequency == 0) {
                period = xvc_get_int32(ctx->cmd_buf + 7);
            } else {
                period = 1000000000 / frequency;
            }
            
            int actual = ftdi_adapter_set_period(ctx->ftdi, period);
            if (actual < 0) {
                actual = period;  /* Echo back on error */
            }
            
            xvc_put_int32(ctx->result_buf, actual);
            if (xvc_write_exact(ctx->socket_fd, ctx->result_buf, 4) < 0) {
                LOG_ERROR("Write failed for settck");
                return -1;
            }
            ctx->bytes_tx += 4;
            ctx->commands++;
            
            LOG_DBG("settck: period=%d, actual=%d", period, actual);
            break;
        }
        
        /* Handle shift */
        if (memcmp(ctx->cmd_buf, "sh", 2) == 0) {
            if (xvc_read_exact(ctx->socket_fd, ctx->cmd_buf + 2, 4) != 1) {
                return 1;
            }
            ctx->bytes_rx += 4;
            
            /* Read length */
            if (xvc_read_exact(ctx->socket_fd, ctx->cmd_buf + 6, 4) != 1) {
                LOG_ERROR("Reading shift length failed");
                return 1;
            }
            ctx->bytes_rx += 4;
            
            int32_t len = xvc_get_int32(ctx->cmd_buf + 6);
            int nr_bytes = (len + 7) / 8;
            
            if (nr_bytes > XVC_MAX_VECTOR_SIZE) {
                LOG_ERROR("Vector size exceeded: %d", nr_bytes);
                return -1;
            }
            
            /* Read TMS and TDI data */
            if (xvc_read_exact(ctx->socket_fd, ctx->vector_buf, nr_bytes * 2) != 1) {
                LOG_ERROR("Reading shift data failed");
                return 1;
            }
            ctx->bytes_rx += nr_bytes * 2;
            
            memset(ctx->result_buf, 0, nr_bytes);
            
            /* Track TLR state for safe client switching */
            ctx->seen_tlr = (ctx->seen_tlr || ctx->jtag_state == JTAG_TEST_LOGIC_RESET) &&
                            (ctx->jtag_state != JTAG_CAPTURE_DR) &&
                            (ctx->jtag_state != JTAG_CAPTURE_IR);
            
            /* Skip bogus state movements (Xilinx impact workaround) */
            bool skip = false;
            if ((ctx->jtag_state == JTAG_EXIT1_IR && len == 5 && ctx->vector_buf[0] == 0x17) ||
                (ctx->jtag_state == JTAG_EXIT1_DR && len == 4 && ctx->vector_buf[0] == 0x0b)) {
                LOG_DBG("Ignoring bogus jtag state movement in state %s", 
                          jtag_state_name(ctx->jtag_state));
                skip = true;
            }
            
            if (!skip) {
                /* Update JTAG state machine */
                for (int i = 0; i < len; i++) {
                    int tms = !!(ctx->vector_buf[i / 8] & (1 << (i & 7)));
                    ctx->jtag_state = jtag_step(ctx->jtag_state, tms);
                }
                
                /* Perform scan operation */
                log_buffer("TMS", ctx->vector_buf, nr_bytes);
                log_buffer("TDI", ctx->vector_buf + nr_bytes, nr_bytes);

                if (ftdi_adapter_scan(ctx->ftdi, 
                                      ctx->vector_buf, 
                                      ctx->vector_buf + nr_bytes,
                                      ctx->result_buf, 
                                      len) < 0) {
                    LOG_ERROR("FTDI scan failed");
                    return -1;
                }
                
                log_buffer("TDO", ctx->result_buf, nr_bytes);
            }
            
            /* Send TDO result */
            if (xvc_write_exact(ctx->socket_fd, ctx->result_buf, nr_bytes) < 0) {
                LOG_ERROR("Write failed for shift result");
                return -1;
            }
            ctx->bytes_tx += nr_bytes;
            ctx->commands++;
            
        } else {
            LOG_ERROR("Invalid command: '%c%c' (0x%02x 0x%02x)", 
                      ctx->cmd_buf[0], ctx->cmd_buf[1],
                      ctx->cmd_buf[0], ctx->cmd_buf[1]);
            return -1;
        }
        
    } while (!(ctx->seen_tlr && ctx->jtag_state == JTAG_RUN_TEST_IDLE));
    
    return 0;
}
