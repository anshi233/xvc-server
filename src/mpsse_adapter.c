/*
 * mpsse_adapter.c - FTDI MPSSE Adapter Layer (High-Speed JTAG)
 * XVC Server for Digilent HS2
 * 
 * Based on OpenOCD's mpsse.c implementation.
 * Provides 10-20x faster JTAG speeds compared to bit-bang mode.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include "mpsse_adapter.h"
#include "logging.h"
#include "bit_copy.h"

/* FTDI USB control transfer constants */
#define FTDI_DEVICE_OUT_REQTYPE     (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define FTDI_DEVICE_IN_REQTYPE      (0x80 | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)

#define FTDI_BITMODE_MPSSE          0x02

#define FTDI_SIO_RESET              0x00
#define FTDI_SIO_SET_LATENCY        0x09
#define FTDI_SIO_SET_BITMODE        0x0B

#define FTDI_SIO_RESET_SIO          0
#define FTDI_SIO_RESET_PURGE_RX     1
#define FTDI_SIO_RESET_PURGE_TX     2

/* FTDI chip types */
typedef enum {
    FTDI_TYPE_FT2232C,
    FTDI_TYPE_FT2232H,
    FTDI_TYPE_FT4232H,
    FTDI_TYPE_FT232H
} ftdi_chip_type_t;

/* Internal context structure */
struct mpsse_context_s {
    libusb_context *usb_ctx;
    libusb_device_handle *usb_dev;
    
    uint8_t in_ep;
    uint8_t out_ep;
    uint16_t max_packet_size;
    uint16_t index;
    uint8_t interface;
    ftdi_chip_type_t type;
    
    uint8_t *write_buffer;
    unsigned int write_size;
    unsigned int write_count;
    
    uint8_t *read_buffer;
    unsigned int read_size;
    unsigned int read_count;
    
    bool is_open;
    int verbose;
    char error[256];
};

/* Forward declarations */
static int mpsse_purge(mpsse_context_t *ctx);
static int mpsse_write(mpsse_context_t *ctx, const uint8_t *data, int len);
static int mpsse_read(mpsse_context_t *ctx, uint8_t *data, int len);

mpsse_context_t* mpsse_adapter_create(void)
{
    mpsse_context_t *ctx = calloc(1, sizeof(mpsse_context_t));
    if (!ctx) {
        return NULL;
    }
    
    ctx->write_size = MPSSE_WRITE_BUFFER_SIZE;
    ctx->read_size = MPSSE_READ_BUFFER_SIZE;
    
    ctx->write_buffer = malloc(ctx->write_size);
    ctx->read_buffer = malloc(ctx->read_size);
    
    if (!ctx->write_buffer || !ctx->read_buffer) {
        free(ctx->write_buffer);
        free(ctx->read_buffer);
        free(ctx);
        return NULL;
    }
    
    return ctx;
}

void mpsse_adapter_destroy(mpsse_context_t *ctx)
{
    if (!ctx) return;
    
    if (ctx->is_open) {
        mpsse_adapter_close(ctx);
    }
    
    free(ctx->write_buffer);
    free(ctx->read_buffer);
    
    if (ctx->usb_ctx) {
        libusb_exit(ctx->usb_ctx);
    }
    
    free(ctx);
}

int mpsse_adapter_open(mpsse_context_t *ctx,
                       int vendor, int product,
                       const char *serial,
                       int index, int interface)
{
    if (!ctx) return -1;
    
    if (vendor < 0) vendor = 0x0403;
    if (product < 0) product = 0x6010;
    
    int err = libusb_init(&ctx->usb_ctx);
    if (err != LIBUSB_SUCCESS) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "libusb_init failed: %s", libusb_error_name(err));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Find and open device */
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(ctx->usb_ctx, &list);
    if (cnt < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "libusb_get_device_list failed: %s", libusb_error_name((int)cnt));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    bool found = false;
    int match_index = 0;
    
    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        err = libusb_get_device_descriptor(list[i], &desc);
        if (err != LIBUSB_SUCCESS) continue;
        
        if (desc.idVendor != (uint16_t)vendor || desc.idProduct != (uint16_t)product)
            continue;
        
        err = libusb_open(list[i], &ctx->usb_dev);
        if (err != LIBUSB_SUCCESS) {
            LOG_WARN("Cannot open device: %s", libusb_error_name(err));
            continue;
        }
        
        /* Check serial if specified */
        if (serial) {
            char desc_serial[256] = {0};
            if (desc.iSerialNumber > 0) {
                libusb_get_string_descriptor_ascii(ctx->usb_dev, desc.iSerialNumber,
                    (unsigned char *)desc_serial, sizeof(desc_serial));
            }
            if (strcmp(serial, desc_serial) != 0) {
                libusb_close(ctx->usb_dev);
                ctx->usb_dev = NULL;
                continue;
            }
        }
        
        /* Check index */
        if (match_index < index) {
            match_index++;
            libusb_close(ctx->usb_dev);
            ctx->usb_dev = NULL;
            continue;
        }
        
        /* Determine chip type */
        switch (desc.bcdDevice) {
            case 0x500: ctx->type = FTDI_TYPE_FT2232C; break;
            case 0x700: ctx->type = FTDI_TYPE_FT2232H; break;
            case 0x800: ctx->type = FTDI_TYPE_FT4232H; break;
            case 0x900: ctx->type = FTDI_TYPE_FT232H; break;
            default:
                LOG_WARN("Unknown FTDI chip type: 0x%04x, assuming FT2232H", desc.bcdDevice);
                ctx->type = FTDI_TYPE_FT2232H;
        }
        
        found = true;
        break;
    }
    
    libusb_free_device_list(list, 1);
    
    if (!found) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "No matching FTDI device found (VID:0x%04x PID:0x%04x)", vendor, product);
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    /* Set interface and calculate index */
    ctx->interface = interface;
    ctx->index = interface + 1;
    
    /* Detach kernel driver if attached */
    err = libusb_detach_kernel_driver(ctx->usb_dev, ctx->interface);
    if (err != LIBUSB_SUCCESS && err != LIBUSB_ERROR_NOT_FOUND && err != LIBUSB_ERROR_NOT_SUPPORTED) {
        LOG_WARN("Failed to detach kernel driver: %s", libusb_error_name(err));
    }
    
    /* Claim interface */
    err = libusb_claim_interface(ctx->usb_dev, ctx->interface);
    if (err != LIBUSB_SUCCESS) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "libusb_claim_interface failed: %s", libusb_error_name(err));
        LOG_ERROR("%s", ctx->error);
        libusb_close(ctx->usb_dev);
        return -1;
    }
    
    /* Get endpoint info */
    struct libusb_config_descriptor *config;
    err = libusb_get_active_config_descriptor(libusb_get_device(ctx->usb_dev), &config);
    if (err == LIBUSB_SUCCESS) {
        const struct libusb_interface_descriptor *iface = 
            &config->interface[ctx->interface].altsetting[0];
        for (int i = 0; i < iface->bNumEndpoints; i++) {
            const struct libusb_endpoint_descriptor *ep = &iface->endpoint[i];
            if (ep->bEndpointAddress & 0x80) {
                ctx->in_ep = ep->bEndpointAddress;
                ctx->max_packet_size = ep->wMaxPacketSize;
            } else {
                ctx->out_ep = ep->bEndpointAddress;
            }
        }
        libusb_free_config_descriptor(config);
    }
    
    if (ctx->in_ep == 0) ctx->in_ep = 0x81 + ctx->interface * 2;
    if (ctx->out_ep == 0) ctx->out_ep = 0x02 + ctx->interface * 2;
    if (ctx->max_packet_size == 0) ctx->max_packet_size = 512;
    
    /* Reset FTDI */
    err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            FTDI_SIO_RESET, FTDI_SIO_RESET_SIO, ctx->index,
            NULL, 0, MPSSE_USB_TIMEOUT);
    if (err < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "FTDI reset failed: %s", libusb_error_name(err));
        LOG_ERROR("%s", ctx->error);
        libusb_close(ctx->usb_dev);
        return -1;
    }
    
    /* Set latency timer to minimum (1ms would be best but some devices don't support it) */
    err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            FTDI_SIO_SET_LATENCY, 2, ctx->index,
            NULL, 0, MPSSE_USB_TIMEOUT);
    if (err < 0) {
        LOG_WARN("Failed to set latency timer: %s", libusb_error_name(err));
    }
    
    /* Enable MPSSE mode */
    err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            FTDI_SIO_SET_BITMODE,
            (FTDI_BITMODE_MPSSE << 8) | MPSSE_DIR_MASK,
            ctx->index,
            NULL, 0, MPSSE_USB_TIMEOUT);
    if (err < 0) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "Failed to enable MPSSE mode: %s", libusb_error_name(err));
        LOG_ERROR("%s", ctx->error);
        libusb_close(ctx->usb_dev);
        return -1;
    }
    
    /* Purge buffers */
    mpsse_purge(ctx);
    
    /* Initial MPSSE configuration */
    uint8_t setup[] = {
        MPSSE_CMD_LOOPBACK_OFF,                 /* Disable loopback */
        MPSSE_CMD_SET_LOW_BYTE,                 /* Set GPIO low byte */
        MPSSE_DEFAULT_OUT,                      /* Data */
        MPSSE_DIR_MASK,                         /* Direction */
        MPSSE_CMD_SET_HIGH_BYTE,                /* Set GPIO high byte */
        0x00,                                   /* Data */
        0x00,                                   /* Direction (all inputs) */
    };
    
    err = mpsse_write(ctx, setup, sizeof(setup));
    if (err < 0) {
        snprintf(ctx->error, sizeof(ctx->error), "MPSSE setup failed");
        LOG_ERROR("%s", ctx->error);
        libusb_close(ctx->usb_dev);
        return -1;
    }
    
    /* Set default frequency */
    if (mpsse_adapter_set_frequency(ctx, MPSSE_DEFAULT_FREQUENCY) < 0) {
        LOG_WARN("Failed to set default frequency, continuing anyway");
    }
    
    ctx->is_open = true;
    LOG_INFO("MPSSE adapter opened (chip type: %d, max packet: %d)",
             ctx->type, ctx->max_packet_size);
    
    return 0;
}

void mpsse_adapter_close(mpsse_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return;
    
    mpsse_adapter_flush(ctx);
    
    if (ctx->usb_dev) {
        libusb_release_interface(ctx->usb_dev, ctx->interface);
        libusb_close(ctx->usb_dev);
        ctx->usb_dev = NULL;
    }
    
    ctx->is_open = false;
    LOG_INFO("MPSSE adapter closed");
}

bool mpsse_adapter_is_open(const mpsse_context_t *ctx)
{
    return ctx && ctx->is_open;
}

int mpsse_adapter_set_frequency(mpsse_context_t *ctx, uint32_t frequency_hz)
{
    if (!ctx || !ctx->is_open) return -1;
    
    if (frequency_hz > MPSSE_MAX_FREQUENCY)
        frequency_hz = MPSSE_MAX_FREQUENCY;
    
    int base_clock;
    bool disable_div5 = false;
    
    /* High-speed chips (FT2232H, FT4232H, FT232H) can disable divide-by-5 */
    if (ctx->type != FTDI_TYPE_FT2232C) {
        /* For frequencies > 6MHz, disable divide-by-5 to get 60MHz base clock */
        if (frequency_hz > 6000000) {
            base_clock = 60000000;
            disable_div5 = true;
        } else {
            base_clock = 12000000;
        }
    } else {
        base_clock = 12000000;
    }
    
    /* Calculate divisor: TCK = base_clock / ((1 + divisor) * 2) */
    int divisor = (base_clock / 2 + frequency_hz - 1) / frequency_hz - 1;
    if (divisor < 0) divisor = 0;
    if (divisor > 65535) divisor = 65535;
    
    int actual_freq = base_clock / 2 / (1 + divisor);
    
    /* Build command sequence */
    uint8_t cmd[4];
    int cmd_len = 0;
    
    if (disable_div5 && ctx->type != FTDI_TYPE_FT2232C) {
        cmd[cmd_len++] = MPSSE_CMD_DISABLE_DIV5;
    } else if (ctx->type != FTDI_TYPE_FT2232C) {
        cmd[cmd_len++] = MPSSE_CMD_ENABLE_DIV5;
    }
    
    /* Flush any pending commands first */
    mpsse_adapter_flush(ctx);
    
    if (cmd_len > 0) {
        if (mpsse_write(ctx, cmd, cmd_len) < 0) {
            LOG_ERROR("Failed to set clock divider mode");
            return -1;
        }
    }
    
    /* Set divisor */
    uint8_t div_cmd[] = {
        MPSSE_CMD_SET_DIVISOR,
        divisor & 0xff,
        (divisor >> 8) & 0xff
    };
    
    if (mpsse_write(ctx, div_cmd, sizeof(div_cmd)) < 0) {
        LOG_ERROR("Failed to set clock divisor");
        return -1;
    }
    
    LOG_INFO("MPSSE frequency: requested=%uHz, actual=%dHz (base=%dMHz, div=%d)",
             frequency_hz, actual_freq, base_clock / 1000000, divisor);
    
    return actual_freq;
}

int mpsse_adapter_scan(mpsse_context_t *ctx,
                       const uint8_t *tms,
                       const uint8_t *tdi,
                       uint8_t *tdo,
                       int bits)
{
    if (!ctx || !ctx->is_open || !tms || !tdi || !tdo || bits <= 0) return -1;
    
    int bytes = (bits + 7) / 8;
    memset(tdo, 0, bytes);
    
    ctx->write_count = 0;
    
    /*
     * MPSSE scan with proper TDO response handling:
     * 
     * Key insight: MPSSE returns per-command responses, NOT a contiguous bitstream.
     * - Byte-mode (0x39): Returns N bytes for N bytes clocked
     * - Bit-mode (0x23): Returns 1 byte with data right-justified (bits 7 down to 8-count)
     * - TMS-mode (0x6b): Returns 1 byte with TDO in bits 7 down to 8-count
     * 
     * We track:
     * - read_count: Expected response bytes
     * - copy_queue: Pending bit-copy operations to assemble TDO from responses
     */
    
    bit_copy_entry_t copy_queue[MAX_BIT_COPY_ENTRIES];
    int copy_count = 0;
    int read_count = 0;
    int tdo_bit_pos = 0;
    int bit_pos = 0;
    
    LOG_TRACE("MPSSE scan: bits=%d, bytes=%d", bits, bytes);
    
    while (bit_pos < bits) {
        /* Find TMS run length */
        int tms_run = 0;
        int current_tms = (tms[bit_pos / 8] >> (bit_pos & 7)) & 1;
        
        while ((bit_pos + tms_run) < bits) {
            int bit_idx = bit_pos + tms_run;
            int tms_bit = (tms[bit_idx / 8] >> (bit_idx & 7)) & 1;
            if (tms_bit != current_tms) {
                break;
            }
            tms_run++;
        }
        
        if (current_tms == 0) {
            /* TMS=0: Data shift mode - use efficient byte transfers */
            int remaining = tms_run;
            while (remaining > 0) {
                /* Flush if buffer getting full */
                if ((ctx->write_count + 4) > (MPSSE_WRITE_BUFFER_SIZE - 4)) {
                    ctx->write_buffer[ctx->write_count++] = MPSSE_CMD_SEND_IMMEDIATE;
                    if (mpsse_write(ctx, ctx->write_buffer, ctx->write_count) < 0) {
                        LOG_ERROR("MPSSE write failed during flush");
                        return -1;
                    }
                    ctx->write_count = 0;
                }
                
                if (remaining < 8) {
                    /* Bit mode for < 8 bits */
                    uint8_t data = 0;
                    for (int i = 0; i < remaining; i++) {
                        int bit_idx = bit_pos + i;
                        int tdi_bit = (tdi[bit_idx / 8] >> (bit_idx & 7)) & 1;
                        if (tdi_bit) {
                            data |= (1 << i);
                        }
                    }
                    LOG_TRACE("TMS=0: bit-mode: %d bits, data=0x%02x", remaining, data);
                    
                    /* 0x23 = Clock data bits in and out LSB first on -ve/+ve edges */
                    ctx->write_buffer[ctx->write_count++] = 0x3b;  /* Read bits on +ve edge, LSB first */
                    ctx->write_buffer[ctx->write_count++] = remaining - 1;
                    ctx->write_buffer[ctx->write_count++] = data;
                    
                    /* Queue bit-copy: MPSSE returns data right-justified in byte */
                    /* Data is in bits (8-remaining) to 7, we need to extract 'remaining' bits */
                    if (copy_count < MAX_BIT_COPY_ENTRIES) {
                        copy_queue[copy_count].tdo_bit_offset = tdo_bit_pos;
                        copy_queue[copy_count].src_byte_offset = read_count;
                        copy_queue[copy_count].src_bit_offset = 8 - remaining;
                        copy_queue[copy_count].bit_count = remaining;
                        copy_count++;
                    }
                    read_count += 1;  /* Bit-mode always returns 1 byte */
                    
                    tdo_bit_pos += remaining;
                    bit_pos += remaining;
                    remaining = 0;
                } else {
                    /* Byte mode for >= 8 bits */
                    int this_bytes = remaining / 8;
                    if (this_bytes > 65536) {
                        this_bytes = 65536;
                    }
                    int this_bits = this_bytes * 8;
                    
                    LOG_TRACE("TMS=0: byte-mode: %d bytes (%d bits)", this_bytes, this_bits);
                    
                    /* 0x39 = Clock data bytes in and out LSB first on -ve/+ve edges */
                    ctx->write_buffer[ctx->write_count++] = 0x39;
                    ctx->write_buffer[ctx->write_count++] = (this_bytes - 1) & 0xff;
                    ctx->write_buffer[ctx->write_count++] = ((this_bytes - 1) >> 8) & 0xff;
                    
                    for (int i = 0; i < this_bytes; i++) {
                        int byte_idx = (bit_pos + i * 8) / 8;
                        ctx->write_buffer[ctx->write_count++] = tdi[byte_idx];
                    }
                    
                    /* Queue bit-copy: Byte-mode returns bytes directly, no shifting needed */
                    if (copy_count < MAX_BIT_COPY_ENTRIES) {
                        copy_queue[copy_count].tdo_bit_offset = tdo_bit_pos;
                        copy_queue[copy_count].src_byte_offset = read_count;
                        copy_queue[copy_count].src_bit_offset = 0;
                        copy_queue[copy_count].bit_count = this_bits;
                        copy_count++;
                    }
                    read_count += this_bytes;
                    
                    tdo_bit_pos += this_bits;
                    bit_pos += this_bits;
                    remaining -= this_bits;
                }
            }
        } else {
            /* TMS≠0: TMS transition mode - use bit-mode with TMS */
            int remaining = tms_run;
            while (remaining > 0) {
                /* Flush if buffer getting full */
                if ((ctx->write_count + 3) > (MPSSE_WRITE_BUFFER_SIZE - 4)) {
                    ctx->write_buffer[ctx->write_count++] = MPSSE_CMD_SEND_IMMEDIATE;
                    if (mpsse_write(ctx, ctx->write_buffer, ctx->write_count) < 0) {
                        LOG_ERROR("MPSSE write failed during flush");
                        return -1;
                    }
                    ctx->write_count = 0;
                }
                
                /* Max 7 bits per command (FT2232 bug) */
                int this_bits = (remaining > 7) ? 7 : remaining;
                
                /* Extract TMS bits and TDI bit */
                uint8_t tms_data = 0;
                uint8_t tdi_bit = 0;
                
                for (int i = 0; i < this_bits; i++) {
                    int bit_idx = bit_pos + i;
                    int tms_bit = (tms[bit_idx / 8] >> (bit_idx & 7)) & 1;
                    int tdi_val = (tdi[bit_idx / 8] >> (bit_idx & 7)) & 1;
                    
                    if (tms_bit) {
                        tms_data |= (1 << i);
                    }
                    /* For TMS mode, TDI is repeated for all bits (use first bit's value) */
                    if (i == 0) {
                        tdi_bit = tdi_val;
                    }
                }
                
                LOG_TRACE("TMS≠0: tms_data=0x%02x, tdi=%d, bits=%d", tms_data, tdi_bit, this_bits);
                
                /* 0x6b = Clock data to TMS pin with read, LSB first */
                ctx->write_buffer[ctx->write_count++] = 0x6b;
                ctx->write_buffer[ctx->write_count++] = this_bits - 1;
                ctx->write_buffer[ctx->write_count++] = tms_data | (tdi_bit << 7);
                
                /* Queue bit-copy: TMS-mode returns TDO right-justified like bit-mode */
                if (copy_count < MAX_BIT_COPY_ENTRIES) {
                    copy_queue[copy_count].tdo_bit_offset = tdo_bit_pos;
                    copy_queue[copy_count].src_byte_offset = read_count;
                    copy_queue[copy_count].src_bit_offset = 8 - this_bits;
                    copy_queue[copy_count].bit_count = this_bits;
                    copy_count++;
                }
                read_count += 1;  /* TMS-mode always returns 1 byte */
                
                tdo_bit_pos += this_bits;
                bit_pos += this_bits;
                remaining -= this_bits;
            }
        }
    }
    
    /* Final flush */
    if (ctx->write_count > 0) {
        LOG_TRACE("MPSSE: flushing %d bytes, expecting %d response bytes", ctx->write_count, read_count);
        ctx->write_buffer[ctx->write_count++] = MPSSE_CMD_SEND_IMMEDIATE;
        if (mpsse_write(ctx, ctx->write_buffer, ctx->write_count) < 0) {
            LOG_ERROR("MPSSE final write failed");
            return -1;
        }
        ctx->write_count = 0;
    }
    
    /* Read all response data */
    if (read_count > 0) {
        uint8_t *read_buffer = malloc(read_count);
        if (!read_buffer) {
            LOG_ERROR("Failed to allocate read buffer (%d bytes)", read_count);
            return -1;
        }
        
        int bytes_read = 0;
        while (bytes_read < read_count) {
            int chunk = read_count - bytes_read;
            if (chunk > (int)ctx->read_size) {
                chunk = ctx->read_size;
            }
            LOG_TRACE("MPSSE: reading %d bytes (total %d/%d)", chunk, bytes_read, read_count);
            if (mpsse_read(ctx, ctx->read_buffer, chunk) < 0) {
                LOG_ERROR("MPSSE read failed at byte %d", bytes_read);
                free(read_buffer);
                return -1;
            }
            memcpy(read_buffer + bytes_read, ctx->read_buffer, chunk);
            bytes_read += chunk;
        }
        
        /* Log raw response in hex format */
        if (read_count > 0 && read_count <= 64) {
            char raw_hex[read_count * 3 + 1];
            for (int i = 0; i < read_count; i++) {
                snprintf(raw_hex + i * 3, 4, "%02x ", read_buffer[i]);
            }
            raw_hex[read_count * 3] = '\0';
            LOG_TRACE("MPSSE: raw response (%d bytes): %s", read_count, raw_hex);
        }
        
        /* Execute all queued bit-copy operations to assemble TDO */
        for (int i = 0; i < copy_count; i++) {
            LOG_TRACE("bit_copy: tdo_off=%d src_byte=%d src_bit=%d count=%d",
                     copy_queue[i].tdo_bit_offset,
                     copy_queue[i].src_byte_offset,
                     copy_queue[i].src_bit_offset,
                     copy_queue[i].bit_count);
            bit_copy(tdo, copy_queue[i].tdo_bit_offset,
                    read_buffer + copy_queue[i].src_byte_offset,
                    copy_queue[i].src_bit_offset,
                    copy_queue[i].bit_count);
        }
        
        free(read_buffer);
    }
    
    /* Log final TDO in hex format */
    if (bytes > 0 && bytes <= 64) {
        char tdo_hex[bytes * 3 + 1];
        for (int i = 0; i < bytes; i++) {
            snprintf(tdo_hex + i * 3, 4, "%02x ", tdo[i]);
        }
        tdo_hex[bytes * 3] = '\0';
        LOG_TRACE("MPSSE: TDO result (%d bytes): %s", bytes, tdo_hex);
    }
    
    return 0;
}

int mpsse_adapter_flush(mpsse_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return -1;
    
    if (ctx->write_count > 0) {
        ctx->write_buffer[ctx->write_count++] = MPSSE_CMD_SEND_IMMEDIATE;
        int ret = mpsse_write(ctx, ctx->write_buffer, ctx->write_count);
        ctx->write_count = 0;
        return ret;
    }
    
    return 0;
}

const char* mpsse_adapter_error(const mpsse_context_t *ctx)
{
    if (!ctx) return "NULL context";
    return ctx->error[0] ? ctx->error : "No error";
}

void mpsse_adapter_set_verbose(mpsse_context_t *ctx, int level)
{
    if (ctx) ctx->verbose = level;
}

/* Internal helper: purge FTDI buffers */
static int mpsse_purge(mpsse_context_t *ctx)
{
    if (!ctx || !ctx->usb_dev) return -1;
    
    int err;
    
    err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            FTDI_SIO_RESET, FTDI_SIO_RESET_PURGE_RX, ctx->index,
            NULL, 0, MPSSE_USB_TIMEOUT);
    if (err < 0) {
        LOG_WARN("Failed to purge RX: %s", libusb_error_name(err));
    }
    
    err = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            FTDI_SIO_RESET, FTDI_SIO_RESET_PURGE_TX, ctx->index,
            NULL, 0, MPSSE_USB_TIMEOUT);
    if (err < 0) {
        LOG_WARN("Failed to purge TX: %s", libusb_error_name(err));
    }
    
    ctx->write_count = 0;
    ctx->read_count = 0;
    
    return 0;
}

/* Internal helper: write data to FTDI */
static int mpsse_write(mpsse_context_t *ctx, const uint8_t *data, int len)
{
    if (!ctx || !ctx->usb_dev || len <= 0) return -1;
    
    int transferred = 0;
    int err = libusb_bulk_transfer(ctx->usb_dev, ctx->out_ep,
            (uint8_t *)data, len, &transferred, MPSSE_USB_TIMEOUT);
    
    if (err != LIBUSB_SUCCESS) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "USB write failed: %s", libusb_error_name(err));
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    if (transferred != len) {
        snprintf(ctx->error, sizeof(ctx->error),
                 "USB short write: %d/%d", transferred, len);
        LOG_ERROR("%s", ctx->error);
        return -1;
    }
    
    return 0;
}

/* Internal helper: read data from FTDI */
static int mpsse_read(mpsse_context_t *ctx, uint8_t *data, int len)
{
    if (!ctx || !ctx->usb_dev || len <= 0) return -1;
    
    int total_read = 0;
    uint8_t temp_buf[512];
    
    while (total_read < len) {
        int transferred = 0;
        int err = libusb_bulk_transfer(ctx->usb_dev, ctx->in_ep,
                temp_buf, sizeof(temp_buf), &transferred, MPSSE_USB_TIMEOUT);
        
        if (err != LIBUSB_SUCCESS && err != LIBUSB_ERROR_TIMEOUT) {
            snprintf(ctx->error, sizeof(ctx->error),
                     "USB read failed: %s", libusb_error_name(err));
            LOG_ERROR("%s", ctx->error);
            return -1;
        }
        
        if (transferred == 0) {
            /* Timeout with no data, retry */
            continue;
        }
        
        /* FTDI adds 2 status bytes at the start of each packet */
        int packet_size = ctx->max_packet_size;
        int data_start = 0;
        
        while (data_start < transferred && total_read < len) {
            /* Skip 2 status bytes at packet boundaries */
            int skip = 2;
            if (data_start + skip >= transferred) break;
            
            int payload_len = packet_size - 2;
            if (data_start + skip + payload_len > transferred) {
                payload_len = transferred - data_start - skip;
            }
            if (total_read + payload_len > len) {
                payload_len = len - total_read;
            }
            
            if (payload_len > 0) {
                memcpy(data + total_read, temp_buf + data_start + skip, payload_len);
                total_read += payload_len;
            }
            
            data_start += packet_size;
        }
    }
    
    return 0;
}
