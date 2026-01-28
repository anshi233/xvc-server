/*
 * ftdi_adapter.c - FTDI Adapter Layer with MPSSE Mode
 * XVC Server for Digilent HS2 / FT232H
 * 
 * Uses MPSSE (Multi-Protocol Synchronous Serial Engine) for high-speed JTAG
 * Supports up to 30 MHz TCK frequency
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ftdi.h>
#include "ftdi_adapter.h"
#include "logging.h"

/* MPSSE Pin definitions for JTAG (directly wired) */
#define MPSSE_TCK   0x01   /* ADBUS0 - TCK output */
#define MPSSE_TDI   0x02   /* ADBUS1 - TDI output */
#define MPSSE_TDO   0x04   /* ADBUS2 - TDO input */
#define MPSSE_TMS   0x08   /* ADBUS3 - TMS output */

/* MPSSE output direction mask (all outputs except TDO) */
#define MPSSE_DIR_OUT  (MPSSE_TCK | MPSSE_TDI | MPSSE_TMS)

/* MPSSE default output state */
#define MPSSE_INIT_OUT 0x08  /* TMS high, others low */

/* MPSSE Commands from ftdi.h and AN_108 */
#define MPSSE_CMD_CLOCK_DATA_BYTES_OUT_POS   0x19  /* TDI bytes out on +ve edge, no read */
#define MPSSE_CMD_CLOCK_DATA_BYTES_IN_POS    0x28  /* TDO bytes in on +ve edge */
#define MPSSE_CMD_CLOCK_DATA_BYTES_IO_POS    0x39  /* TDI out, TDO in on +ve edge */
#define MPSSE_CMD_CLOCK_DATA_BITS_OUT_POS    0x1B  /* TDI bits out on +ve edge, no read */
#define MPSSE_CMD_CLOCK_DATA_BITS_IN_POS     0x2A  /* TDO bits in on +ve edge */
#define MPSSE_CMD_CLOCK_DATA_BITS_IO_POS     0x3B  /* TDI bits out, TDO in on +ve edge */
#define MPSSE_CMD_TMS_OUT_NEG                0x4B  /* TMS out on -ve edge, no read */
#define MPSSE_CMD_TMS_IO_NEG                 0x6B  /* TMS out on -ve, TDO in on +ve */

/* Max buffer size for MPSSE commands */
#define MPSSE_MAX_BUFFER 65536

/* Internal context structure */
struct ftdi_context_s {
    struct ftdi_context ftdi;
    bool is_open;
    int verbose;
    char error[256];
    uint32_t freq_hz;          /* Current TCK frequency */
    uint8_t *cmd_buffer;       /* Command buffer */
    size_t cmd_buffer_size;    /* Buffer size */
};

/* Forward declarations */
static int mpsse_sync(ftdi_context_t *ctx);
static int mpsse_set_divisor(ftdi_context_t *ctx, uint32_t freq_hz);
static int mpsse_flush_and_read(ftdi_context_t *ctx, uint8_t *read_buf, size_t read_len);

ftdi_context_t* ftdi_adapter_create(void)
{
    ftdi_context_t *ctx = calloc(1, sizeof(ftdi_context_t));
    if (!ctx) {
        return NULL;
    }
    
    int ret = ftdi_init(&ctx->ftdi);
    if (ret < 0) {
        free(ctx);
        return NULL;
    }
    
    /* Allocate command buffer */
    ctx->cmd_buffer = malloc(MPSSE_MAX_BUFFER);
    if (!ctx->cmd_buffer) {
        ftdi_deinit(&ctx->ftdi);
        free(ctx);
        return NULL;
    }
    ctx->cmd_buffer_size = MPSSE_MAX_BUFFER;
    
    return ctx;
}

void ftdi_adapter_destroy(ftdi_context_t *ctx)
{
    if (!ctx) return;
    
    if (ctx->is_open) {
        ftdi_adapter_close(ctx);
    }
    
    if (ctx->cmd_buffer) {
        free(ctx->cmd_buffer);
    }
    
    ftdi_deinit(&ctx->ftdi);
    free(ctx);
}

int ftdi_adapter_open(ftdi_context_t *ctx, 
                      int vendor, int product,
                      const char *serial,
                      int index, int interface)
{
    if (!ctx) return -1;
    
    if (product < 0) product = 0x6010;
    if (vendor < 0) vendor = 0x0403;
    
    /* Select interface */
    enum ftdi_interface selected_interface;
    switch (interface) {
        case 0: selected_interface = INTERFACE_A; break;
        case 1: selected_interface = INTERFACE_B; break;
        case 2: selected_interface = INTERFACE_C; break;
        case 3: selected_interface = INTERFACE_D; break;
        default: selected_interface = INTERFACE_ANY; break;
    }
    
    int ret = ftdi_set_interface(&ctx->ftdi, selected_interface);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error), 
                 "ftdi_set_interface(%d): %d (%s)", 
                 interface, ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Ignore index if serial provided */
    if (serial) index = 0;
    
    ret = ftdi_usb_open_desc_index(&ctx->ftdi, vendor, product, NULL, serial, index);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_usb_open(0x%04x, 0x%04x): %d (%s)",
                 vendor, product, ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Reset device */
    ret = ftdi_usb_reset(&ctx->ftdi);
    if (ret < 0) {
        LOG_WARN("ftdi_usb_reset: %d (%s)", ret, ftdi_get_error_string(&ctx->ftdi));
    }
    
    /* Set latency timer to minimum for maximum throughput */
    ret = ftdi_set_latency_timer(&ctx->ftdi, 1);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_latency_timer: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Flush buffers */
    ret = ftdi_tcioflush(&ctx->ftdi);
    if (ret < 0) {
        LOG_WARN("ftdi_tcioflush: %d (%s)", ret, ftdi_get_error_string(&ctx->ftdi));
    }
    
    /* Reset MPSSE controller */
    ret = ftdi_set_bitmode(&ctx->ftdi, 0, BITMODE_RESET);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_bitmode(RESET): %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Enable MPSSE mode */
    ret = ftdi_set_bitmode(&ctx->ftdi, 0, BITMODE_MPSSE);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_bitmode(MPSSE): %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Synchronize MPSSE by sending bad command 0xAA */
    ret = mpsse_sync(ctx);
    if (ret < 0) {
        LOG_WARN("MPSSE sync failed, continuing anyway");
    }
    
    /* Configure MPSSE for JTAG */
    uint8_t setup_cmds[] = {
        /* Disable divide-by-5 (enable 60MHz master clock on FT232H) */
        DIS_DIV_5,
        /* Disable adaptive clocking */
        DIS_ADAPTIVE,
        /* Disable 3-phase clocking */
        DIS_3_PHASE,
        /* Set low byte direction and initial state */
        SET_BITS_LOW, MPSSE_INIT_OUT, MPSSE_DIR_OUT,
        /* Set high byte (all inputs) */
        SET_BITS_HIGH, 0x00, 0x00,
        /* Loopback off */
        LOOPBACK_END,
    };
    
    ret = ftdi_write_data(&ctx->ftdi, setup_cmds, sizeof(setup_cmds));
    if (ret != sizeof(setup_cmds)) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "MPSSE setup failed: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Set default frequency (6 MHz) */
    ctx->freq_hz = 6000000;
    ret = mpsse_set_divisor(ctx, ctx->freq_hz);
    if (ret < 0) {
        LOG_WARN("Failed to set initial frequency");
    }
    
    /* Send 5 TMS=1 clock cycles to reset JTAG TAP to Test-Logic-Reset */
    uint8_t reset_cmds[] = {
        MPSSE_CMD_TMS_OUT_NEG,
        4,      /* 5 bits (0-indexed) */
        0x1F,   /* TMS = 11111, TDI = 0 */
    };
    ret = ftdi_write_data(&ctx->ftdi, reset_cmds, sizeof(reset_cmds));
    if (ret < 0) {
        LOG_WARN("JTAG reset failed: %d", ret);
    }
    
    ctx->is_open = true;
    LOG_INFO("FTDI device opened in MPSSE mode (up to 30MHz TCK)");
    return 0;
}

int ftdi_adapter_open_bus(ftdi_context_t *ctx, int bus, int device, int interface)
{
    (void)bus;
    (void)device;
    (void)interface;
    
    snprintf(ctx->error, sizeof(ctx->error), "Bus-based opening not yet implemented");
    LOG_ERROR("%s", ctx->error);
    return -1;
}

void ftdi_adapter_close(ftdi_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return;
    
    /* Return to reset mode */
    ftdi_set_bitmode(&ctx->ftdi, 0, BITMODE_RESET);
    ftdi_usb_close(&ctx->ftdi);
    ctx->is_open = false;
    LOG_INFO("FTDI device closed");
}

bool ftdi_adapter_is_open(const ftdi_context_t *ctx)
{
    return ctx && ctx->is_open;
}

/* Synchronize with MPSSE by sending bad command */
static int mpsse_sync(ftdi_context_t *ctx)
{
    uint8_t cmd = 0xAA;  /* Invalid command */
    uint8_t response[2];
    
    /* Flush any existing data */
    ftdi_tcioflush(&ctx->ftdi);
    
    /* Send bad command */
    int ret = ftdi_write_data(&ctx->ftdi, &cmd, 1);
    if (ret != 1) return -1;
    
    /* Read response - should be 0xFA followed by the bad command */
    int timeout = 100;
    int total = 0;
    while (total < 2 && timeout-- > 0) {
        ret = ftdi_read_data(&ctx->ftdi, response + total, 2 - total);
        if (ret > 0) total += ret;
        if (ret < 0) return -1;
    }
    
    if (total >= 2 && response[0] == 0xFA && response[1] == 0xAA) {
        LOG_DBG("MPSSE sync successful");
        return 0;
    }
    
    LOG_DBG("MPSSE sync: got 0x%02x 0x%02x (expected 0xFA 0xAA)", 
            total > 0 ? response[0] : 0, total > 1 ? response[1] : 0);
    return -1;
}

/* Set MPSSE clock divisor for target frequency */
static int mpsse_set_divisor(ftdi_context_t *ctx, uint32_t freq_hz)
{
    /* 
     * FT232H: 60 MHz master clock with DIS_DIV_5
     * TCK frequency = 60MHz / ((1 + divisor) * 2)
     * divisor = (60MHz / (2 * freq)) - 1
     *
     * Max frequency: 30 MHz (divisor = 0)
     * Min frequency: ~92 Hz (divisor = 0xFFFF)
     */
    
    uint32_t divisor;
    
    if (freq_hz >= 30000000) {
        divisor = 0;  /* 30 MHz */
    } else if (freq_hz <= 92) {
        divisor = 0xFFFF;  /* ~92 Hz */
    } else {
        divisor = (30000000 / freq_hz) - 1;
        if (divisor > 0xFFFF) divisor = 0xFFFF;
    }
    
    uint8_t cmd[] = {
        TCK_DIVISOR,
        divisor & 0xFF,
        (divisor >> 8) & 0xFF
    };
    
    int ret = ftdi_write_data(&ctx->ftdi, cmd, sizeof(cmd));
    if (ret != sizeof(cmd)) {
        return -1;
    }
    
    uint32_t actual_freq = 30000000 / (1 + divisor);
    LOG_INFO("TCK frequency set: requested=%uHz, actual=%uHz (divisor=%u)", 
             freq_hz, actual_freq, divisor);
    
    ctx->freq_hz = actual_freq;
    return 0;
}

int ftdi_adapter_set_period(ftdi_context_t *ctx, unsigned int period_ns)
{
    if (!ctx || !ctx->is_open) return -1;
    
    /* Convert period to frequency */
    uint32_t freq_hz = 1000000000 / period_ns;
    
    int ret = mpsse_set_divisor(ctx, freq_hz);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error), "Failed to set TCK divisor");
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Return actual period */
    return 1000000000 / ctx->freq_hz;
}

int ftdi_adapter_set_frequency(ftdi_context_t *ctx, uint32_t frequency_hz)
{
    if (!ctx || frequency_hz == 0) return -1;
    
    int ret = mpsse_set_divisor(ctx, frequency_hz);
    return ret;
}

/*
 * JTAG scan using MPSSE
 * 
 * MPSSE provides two main ways to shift JTAG data:
 * 1. TMS shifting with data (for state machine navigation + small data)
 * 2. Data shifting (for bulk data in Shift-DR/Shift-IR states)
 *
 * For XVC, we need to handle arbitrary TMS/TDI patterns, so we use
 * the TMS shift command which allows simultaneous TDI and TDO.
 */
int ftdi_adapter_scan(ftdi_context_t *ctx,
                      const uint8_t *tms,
                      const uint8_t *tdi,
                      uint8_t *tdo,
                      int bits)
{
    if (!ctx || !ctx->is_open || !tms || !tdi || !tdo) return -1;
    if (bits <= 0) return 0;
    
    /* Clear TDO output */
    memset(tdo, 0, (bits + 7) / 8);
    
    /* 
     * Use MPSSE_CMD_TMS_IO_NEG (0x6B) for each bit:
     *   - Clocks TMS data out on negative edge
     *   - Reads TDO on positive edge
     *   - Also drives TDI (bit 7 of data byte)
     *
     * Format: 0x6B, length-1, data
     * Data byte: bit 0-6 = TMS data, bit 7 = TDI state
     * 
     * We can shift up to 7 bits at a time with one command
     */
    
    uint8_t *cmd = ctx->cmd_buffer;
    size_t cmd_len = 0;
    size_t expected_read = 0;
    
    int bit_idx = 0;
    
    while (bit_idx < bits) {
        /* Determine how many bits to send (max 7 per TMS command) */
        int chunk_bits = bits - bit_idx;
        if (chunk_bits > 7) chunk_bits = 7;
        
        /* Build TMS data byte with TDI in bit 7 */
        uint8_t tms_data = 0;
        for (int i = 0; i < chunk_bits; i++) {
            int src_bit = bit_idx + i;
            if (tms[src_bit / 8] & (1 << (src_bit % 8))) {
                tms_data |= (1 << i);
            }
        }
        
        /* Get TDI for first bit of this chunk (TDI is held constant) */
        int tdi_bit = bit_idx;
        if (tdi[tdi_bit / 8] & (1 << (tdi_bit % 8))) {
            tms_data |= 0x80;  /* TDI in bit 7 */
        }
        
        /* Check buffer space */
        if (cmd_len + 3 > ctx->cmd_buffer_size) {
            /* Flush and process */
            int ret = mpsse_flush_and_read(ctx, tdo, expected_read);
            if (ret < 0) return ret;
            cmd_len = 0;
            expected_read = 0;
        }
        
        /* Add TMS shift command */
        cmd[cmd_len++] = MPSSE_CMD_TMS_IO_NEG;
        cmd[cmd_len++] = chunk_bits - 1;  /* Length is 0-indexed */
        cmd[cmd_len++] = tms_data;
        
        expected_read++;  /* Each TMS command returns 1 byte */
        bit_idx += chunk_bits;
    }
    
    /* Send SEND_IMMEDIATE to flush */
    if (cmd_len + 1 <= ctx->cmd_buffer_size) {
        cmd[cmd_len++] = SEND_IMMEDIATE;
    }
    
    /* Write commands */
    int ret = ftdi_write_data(&ctx->ftdi, cmd, cmd_len);
    if (ret != (int)cmd_len) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_write_data: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Read TDO data */
    uint8_t *read_buf = malloc(expected_read);
    if (!read_buf) {
        LOG_ERROR("Failed to allocate read buffer");
        return -1;
    }
    
    size_t total_read = 0;
    int timeout = 1000;
    while (total_read < expected_read && timeout-- > 0) {
        ret = ftdi_read_data(&ctx->ftdi, read_buf + total_read, expected_read - total_read);
        if (ret < 0) {
            snprintf(ctx->error, sizeof(ctx->error),
                     "ftdi_read_data: %d (%s)",
                     ret, ftdi_get_error_string(&ctx->ftdi));
            LOG_ERROR("%s", ctx->error);
            free(read_buf);
            return -1;
        }
        total_read += ret;
    }
    
    if (total_read < expected_read) {
        LOG_ERROR("Timeout reading TDO data (got %zu, expected %zu)", total_read, expected_read);
        free(read_buf);
        return -1;
    }
    
    /* Extract TDO bits from read data
     * Each TMS command returns 1 byte with TDO in bit 7
     * For multi-bit TMS commands, the bits are LSB first in positions 0-6
     */
    bit_idx = 0;
    size_t read_idx = 0;
    
    while (bit_idx < bits && read_idx < total_read) {
        int chunk_bits = bits - bit_idx;
        if (chunk_bits > 7) chunk_bits = 7;
        
        uint8_t tdo_byte = read_buf[read_idx++];
        
        /* TDO bits are in positions 0-6 (shifted right from bit 7) */
        for (int i = 0; i < chunk_bits; i++) {
            int dst_bit = bit_idx + i;
            /* TDO for bit i is in bit (7 - chunk_bits + 1 + i) of the read byte */
            /* Actually for TMS command, TDO is shifted LSB first into bit 7 */
            if (tdo_byte & (1 << (7 - chunk_bits + 1 + i))) {
                tdo[dst_bit / 8] |= (1 << (dst_bit % 8));
            }
        }
        
        bit_idx += chunk_bits;
    }
    
    free(read_buf);
    return 0;
}

/* Helper to flush commands and read response */
static int mpsse_flush_and_read(ftdi_context_t *ctx, uint8_t *tdo, size_t expected_read)
{
    /* Add SEND_IMMEDIATE */
    uint8_t flush_cmd = SEND_IMMEDIATE;
    int ret = ftdi_write_data(&ctx->ftdi, ctx->cmd_buffer, 0);  /* Just flush what we have */
    if (ret < 0) return ret;
    
    ret = ftdi_write_data(&ctx->ftdi, &flush_cmd, 1);
    if (ret < 0) return ret;
    
    /* Read response */
    uint8_t *read_buf = malloc(expected_read);
    if (!read_buf) return -1;
    
    size_t total_read = 0;
    int timeout = 1000;
    while (total_read < expected_read && timeout-- > 0) {
        ret = ftdi_read_data(&ctx->ftdi, read_buf + total_read, expected_read - total_read);
        if (ret < 0) {
            free(read_buf);
            return ret;
        }
        total_read += ret;
    }
    
    /* TODO: Extract TDO bits from read_buf */
    (void)tdo;
    
    free(read_buf);
    return 0;
}

const char* ftdi_adapter_error(const ftdi_context_t *ctx)
{
    if (!ctx) return "NULL context";
    return ctx->error[0] ? ctx->error : "No error";
}

void ftdi_adapter_set_verbose(ftdi_context_t *ctx, int level)
{
    if (ctx) ctx->verbose = level;
}

int ftdi_adapter_set_latency(ftdi_context_t *ctx, int latency_ms)
{
    if (!ctx || !ctx->is_open) return -1;
    
    int ret = ftdi_set_latency_timer(&ctx->ftdi, latency_ms);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_latency_timer: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    LOG_DBG("Latency timer set to %dms", latency_ms);
    return 0;
}
