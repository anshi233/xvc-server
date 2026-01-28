/*
 * ftdi_adapter.c - FTDI Adapter Layer
 * XVC Server for Digilent HS2
 * Based on reference implementation in xvcd/src/io_ftdi.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ftdi.h>
#include "ftdi_adapter.h"
#include "logging.h"

/* I/O direction mask */
#define IO_OUTPUT (FTDI_PORT_MISC | FTDI_PORT_TCK | FTDI_PORT_TDI | FTDI_PORT_TMS)

/* Max write size for non-async mode */
#define FTDI_MAX_WRITESIZE 256

/* Internal context structure */
struct ftdi_context_s {
    struct ftdi_context ftdi;
    bool is_open;
    int verbose;
    char error[256];
};

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
    
    return ctx;
}

void ftdi_adapter_destroy(ftdi_context_t *ctx)
{
    if (!ctx) return;
    
    if (ctx->is_open) {
        ftdi_adapter_close(ctx);
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
    
    /* Set latency timer */
    ret = ftdi_set_latency_timer(&ctx->ftdi, FTDI_DEFAULT_LATENCY);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_latency_timer: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Set bit mode */
    ftdi_set_bitmode(&ctx->ftdi, 0xFF, BITMODE_CBUS);
    ret = ftdi_set_bitmode(&ctx->ftdi, IO_OUTPUT, BITMODE_SYNCBB);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_bitmode: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Set default output state */
    unsigned char init_buf[1] = { FTDI_DEFAULT_OUT };
    ret = ftdi_write_data(&ctx->ftdi, init_buf, 1);
    if (ret < 0) {
        LOG_WARN("ftdi_write_data (init): %d (%s)", 
                 ret, ftdi_get_error_string(&ctx->ftdi));
    }
    
    /* Initialize JTAG to TEST-LOGIC-RESET state */
    /* Send 5 TMS=1 cycles to reset JTAG TAP controller */
    unsigned char reset_seq[10] = {
        FTDI_PORT_TMS,                   /* TMS=1, TCK=0 */
        FTDI_PORT_TMS | FTDI_PORT_TCK,  /* TMS=1, TCK=1 */
        FTDI_PORT_TMS,                   /* TMS=1, TCK=0 */
        FTDI_PORT_TMS | FTDI_PORT_TCK,  /* TMS=1, TCK=1 */
        FTDI_PORT_TMS,                   /* TMS=1, TCK=0 */
        FTDI_PORT_TMS | FTDI_PORT_TCK,  /* TMS=1, TCK=1 */
        FTDI_PORT_TMS,                   /* TMS=1, TCK=0 */
        FTDI_PORT_TMS | FTDI_PORT_TCK,  /* TMS=1, TCK=1 */
        FTDI_PORT_TMS,                   /* TMS=1, TCK=0 */
        FTDI_PORT_TMS | FTDI_PORT_TCK   /* TMS=1, TCK=1 */
    };
    ret = ftdi_write_data(&ctx->ftdi, reset_seq, 10);
    if (ret < 0) {
        LOG_WARN("ftdi_write_data (jtag_reset): %d (%s)", 
                 ret, ftdi_get_error_string(&ctx->ftdi));
    }
    
    /* Ensure TDO is stable after reset */
    ftdi_read_data(&ctx->ftdi, init_buf, 1);
    
    /* Flush buffers */
    ret = ftdi_tcioflush(&ctx->ftdi);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_tcioflush: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    /* Set default baudrate */
    /* Flush buffers */
    if (ftdi_tcioflush(&ctx->ftdi) < 0) {
        LOG_WARN("Failed to flush FTDI buffers: %s", ftdi_get_error_string(&ctx->ftdi));
    }

    ret = ftdi_set_baudrate(&ctx->ftdi, FTDI_DEFAULT_BAUDRATE);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_baudrate: %d (%s)",
                 ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        ftdi_usb_close(&ctx->ftdi);
        return -1;
    }
    
    ctx->is_open = true;
    LOG_INFO("FTDI device opened successfully");
    return 0;
}

int ftdi_adapter_open_bus(ftdi_context_t *ctx, int bus, int device, int interface)
{
    (void)bus;
    (void)device;
    (void)interface;
    
    /* TODO: Implement bus-based opening using libusb */
    snprintf(ctx->error, sizeof(ctx->error), "Bus-based opening not yet implemented");
    LOG_ERROR("%s", ctx->error);
    return -1;
}

void ftdi_adapter_close(ftdi_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return;
    
    ftdi_usb_close(&ctx->ftdi);
    ctx->is_open = false;
    LOG_INFO("FTDI device closed");
}

bool ftdi_adapter_is_open(const ftdi_context_t *ctx)
{
    return ctx && ctx->is_open;
}

int ftdi_adapter_set_period(ftdi_context_t *ctx, unsigned int period_ns)
{
    if (!ctx || !ctx->is_open) return -1;
    
    /* Convert period to baudrate for FT232H synchronous bitbang mode
     * 
     * In sync bitbang mode, the bit rate is approximately 4x the baudrate,
     * because each byte clocks out 8 bits but the effective rate is higher
     * due to USB buffering.
     *
     * FT232H limits:
     *   - Max baudrate: 3,000,000 (3 Mbaud)
     *   - This gives effective JTAG speed of ~6-12 MHz depending on USB
     * 
     * From period (ns) to baudrate:
     *   target_freq_hz = 1,000,000,000 / period_ns
     *   baudrate = target_freq_hz (we need one baud per bit)
     *   But since we send 2 bytes per JTAG bit (low+high TCK), divide by 2
     */
    int target_freq = 1000000000 / period_ns;
    int baudrate = target_freq * 2;  /* 2 bytes per bit cycle */
    
    /* Clamp to valid range for FT232H
     * Min: 300 baud
     * Max: 3000000 baud (3 MHz - FT232H max in sync bitbang)
     */
    if (baudrate < 300) baudrate = 300;
    if (baudrate > 3000000) baudrate = 3000000;
    
    int ret = ftdi_set_baudrate(&ctx->ftdi, baudrate);
    if (ret < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "ftdi_set_baudrate(%d): %d (%s)",
                 baudrate, ret, ftdi_get_error_string(&ctx->ftdi));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Return actual period */
    int actual_period = 2000000000 / baudrate;  /* 2 bytes per bit */
    LOG_INFO("TCK set: requested=%uns, actual=%uns, baudrate=%d", 
             period_ns, actual_period, baudrate);
    return actual_period;
}

int ftdi_adapter_set_frequency(ftdi_context_t *ctx, uint32_t frequency_hz)
{
    if (!ctx || frequency_hz == 0) return -1;
    
    unsigned int period_ns = 1000000000 / frequency_hz;
    int actual = ftdi_adapter_set_period(ctx, period_ns);
    
    return (actual > 0) ? 0 : -1;
}

int ftdi_adapter_scan(ftdi_context_t *ctx,
                      const uint8_t *tms,
                      const uint8_t *tdi,
                      uint8_t *tdo,
                      int bits)
{
    if (!ctx || !ctx->is_open || !tms || !tdi || !tdo) return -1;
    
    /* Allocate buffer for double-clocked data */
    unsigned char *buffer = malloc(bits * 2);
    if (!buffer) {
        LOG_ERROR("Failed to allocate scan buffer");
        return -1;
    }
    
    /* Build output buffer: for each bit, output data then clock high */
    /* Similar to xvcpi approach - explicit timing control */
    for (int i = 0; i < bits; i++) {
        unsigned char v = FTDI_DEFAULT_OUT;
        
        if (tms[i / 8] & (1 << (i & 7))) {
            v |= FTDI_PORT_TMS;
        }
        if (tdi[i / 8] & (1 << (i & 7))) {
            v |= FTDI_PORT_TDI;
        }
        
        /* Set TDI/TMS first, then raise TCK */
        buffer[i * 2 + 0] = v;              
        buffer[i * 2 + 1] = v | FTDI_PORT_TCK;
    }
    
    /* Write and read in chunks */
    int r = 0;
    while (r < bits * 2) {
        int t = bits * 2 - r;
        if (t > FTDI_MAX_WRITESIZE) {
            t = FTDI_MAX_WRITESIZE;
        }
        
        int ret = ftdi_write_data(&ctx->ftdi, buffer + r, t);
        if (ret != t) {
            snprintf(ctx->error, sizeof(ctx->error),
                     "ftdi_write_data: %d (%s)",
                     ret, ftdi_get_error_string(&ctx->ftdi));
            LOG_ERROR("%s", ctx->error);
            free(buffer);
            return -1;
        }
        
        /* Read back */
        int i = 0;
        while (i < t) {
            ret = ftdi_read_data(&ctx->ftdi, buffer + r + i, t - i);
            if (ret < 0) {
                snprintf(ctx->error, sizeof(ctx->error),
                         "ftdi_read_data: %d (%s)",
                         ret, ftdi_get_error_string(&ctx->ftdi));
                LOG_ERROR("%s", ctx->error);
                free(buffer);
                return -1;
            }
            i += ret;
        }
        
        r += t;
    }
    
    /* Extract TDO bits from returned data */
    memset(tdo, 0, (bits + 7) / 8);
    for (int i = 0; i < bits; i++) {
        if (buffer[i * 2 + 1] & FTDI_PORT_TDO) {
            tdo[i / 8] |= 1 << (i & 7);
        }
    }
    
    free(buffer);
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
