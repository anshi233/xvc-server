/*
 * mpsse_adapter.c - FTDI MPSSE Adapter Layer (High-Speed JTAG)
 * XVC Server for Digilent HS2
 * 
 * Based on TinyXVC implementation by Sergey Guralnik.
 * TinyXVC is a minimalistic XVC server that uses MPSSE-capable FTDI chips.
 * Provides 10-20x faster JTAG speeds compared to bit-bang mode.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ftdi.h>
#include "mpsse_adapter.h"
#include "logging.h"

#define OP_SHIFT_WR_FALLING_FLAG  0x01
#define OP_SHIFT_BITMODE_FLAG     0x02
#define OP_SHIFT_LSB_FIRST_FLAG   0x08
#define OP_SHIFT_WR_TDI_FLAG      0x10
#define OP_SHIFT_RD_TDO_FLAG      0x20
#define OP_SHIFT_WR_TMS_FLAG      0x40

#define OP_SET_DBUS_LOBYTE         0x80
#define OP_SET_TCK_DIVISOR         0x86
#define OP_DISABLE_CLK_DIVIDE_BY_5  0x8A
#define OP_LOOPBACK_OFF            0x85

#define MPSSE_DEFAULT_LATENCY   1
#define MPSSE_DEFAULT_CLOCK    10000000
#define MPSSE_MAX_FREQUENCY    30000000

enum jtag_state {
    TEST_LOGIC_RESET,
    RUN_TEST_IDLE,
    SELECT_DR_SCAN,
    CAPTURE_DR,
    SHIFT_DR,
    EXIT_1_DR,
    PAUSE_DR,
    EXIT_2_DR,
    UPDATE_DR,
    SELECT_IR_SCAN,
    CAPTURE_IR,
    SHIFT_IR,
    EXIT_1_IR,
    PAUSE_IR,
    EXIT_2_IR,
    UPDATE_IR
};

typedef void (*rx_observer_fn)(const uint8_t *rxData, void *extra);

struct rx_observer_node {
    struct rx_observer_node *next;
    rx_observer_fn fn;
    const uint8_t *data;
    void *extra;
};

struct mpsse_buffer {
    uint8_t *tx_buffer;
    int tx_num_bytes;
    int max_tx_buffer_bytes;
    uint8_t *rx_buffer;
    int rx_num_bytes;
    int max_rx_buffer_bytes;
    struct rx_observer_node *rx_observer_first;
    struct rx_observer_node *rx_observer_last;
};

struct mpsse_context_s {
    struct ftdi_context ftdi;
    struct mpsse_buffer buffer;
    enum jtag_state state;
    bool last_tdi;
    bool is_open;
    int verbose;
    int chip_buffer_size;  /* Chip FIFO buffer size: 1024 for FT232H, 4096 for FT2232H */
    char error[256];
};

static inline int min(int a, int b)
{
    return a < b ? a : b;
}

static inline int get_bit(const uint8_t *p, int idx)
{
    return !!(p[idx / 8] & (1 << (idx % 8)));
}

static inline void set_bit(uint8_t *p, int idx, bool bit)
{
    uint8_t *octet = p + idx / 8;
    if (bit) *octet |= 1 << (idx % 8);
    else *octet &= ~(1 << (idx % 8));
}

static void copy_bits(const uint8_t *src, int fromIdx, uint8_t *dst, int toIdx, int numBits, bool duplicateLastBit)
{
    for (int i = 0; i < numBits; i++) {
        set_bit(dst, toIdx++, get_bit(src, fromIdx++));
    }
    if (duplicateLastBit) {
        set_bit(dst, toIdx, get_bit(src, fromIdx - 1));
    }
}

mpsse_context_t* mpsse_adapter_create(void)
{
    mpsse_context_t *ctx = calloc(1, sizeof(mpsse_context_t));
    if (!ctx) return NULL;

    /* Initial buffer allocation with minimum size (1KB for FT232H)
     * Buffers will be reallocated in mpsse_adapter_open based on actual chip type
     */
    ctx->buffer.max_tx_buffer_bytes = 3 * 1024;
    ctx->buffer.max_rx_buffer_bytes = 1024;

    ctx->buffer.tx_buffer = malloc(ctx->buffer.max_tx_buffer_bytes);
    ctx->buffer.rx_buffer = malloc(ctx->buffer.max_rx_buffer_bytes);

    if (!ctx->buffer.tx_buffer || !ctx->buffer.rx_buffer) {
        free(ctx->buffer.tx_buffer);
        free(ctx->buffer.rx_buffer);
        free(ctx);
        return NULL;
    }

    ctx->state = TEST_LOGIC_RESET;
    ctx->last_tdi = 0;
    ctx->chip_buffer_size = 1024;  /* Default to minimum (FT232H) */

    return ctx;
}

void mpsse_adapter_destroy(mpsse_context_t *ctx)
{
    if (!ctx) return;
    
    if (ctx->is_open) {
        mpsse_adapter_close(ctx);
    }
    
    free(ctx->buffer.tx_buffer);
    free(ctx->buffer.rx_buffer);
    
    ftdi_deinit(&ctx->ftdi);
    
    free(ctx);
}

static enum jtag_state next_state(enum jtag_state curState, bool tmsHigh)
{
    switch (curState) {
        case TEST_LOGIC_RESET: return tmsHigh ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
        case RUN_TEST_IDLE: return tmsHigh ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        case SELECT_DR_SCAN: return tmsHigh ? SELECT_IR_SCAN : CAPTURE_DR;
        case CAPTURE_DR: return tmsHigh ? EXIT_1_DR : SHIFT_DR;
        case SHIFT_DR: return tmsHigh ? EXIT_1_DR : SHIFT_DR;
        case EXIT_1_DR: return tmsHigh ? UPDATE_DR : PAUSE_DR;
        case PAUSE_DR: return tmsHigh ? EXIT_2_DR : PAUSE_DR;
        case EXIT_2_DR: return tmsHigh ? UPDATE_DR : SHIFT_DR;
        case UPDATE_DR: return tmsHigh ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        case SELECT_IR_SCAN: return tmsHigh ? TEST_LOGIC_RESET : CAPTURE_IR;
        case CAPTURE_IR: return tmsHigh ? EXIT_1_IR : SHIFT_IR;
        case SHIFT_IR: return tmsHigh ? EXIT_1_IR : SHIFT_IR;
        case EXIT_1_IR: return tmsHigh ? UPDATE_IR : PAUSE_IR;
        case PAUSE_IR: return tmsHigh ? EXIT_2_IR : PAUSE_IR;
        case EXIT_2_IR: return tmsHigh ? UPDATE_IR : SHIFT_IR;
        case UPDATE_IR: return tmsHigh ? SELECT_DR_SCAN : RUN_TEST_IDLE;
    }
    return RUN_TEST_IDLE;
}

static int mpsse_chip_recovery(mpsse_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return -1;
    
    LOG_WARN("Attempting chip recovery...");
    
    uint8_t reset_cmds[] = {
        OP_LOOPBACK_OFF,
        OP_SET_TCK_DIVISOR,
        29 & 0xFF,
        (29 >> 8) & 0xFF,
        OP_DISABLE_CLK_DIVIDE_BY_5,
    };
    
    ftdi_usb_reset(&ctx->ftdi);
    ftdi_tcioflush(&ctx->ftdi);
    
    if (ftdi_set_bitmode(&ctx->ftdi, 0x00, BITMODE_RESET) < 0) {
        LOG_ERROR("Failed to reset bitmode during recovery");
        return -1;
    }
    
    if (ftdi_set_bitmode(&ctx->ftdi, 0x00, BITMODE_MPSSE) < 0) {
        LOG_ERROR("Failed to set MPSSE mode during recovery");
        return -1;
    }
    
    usleep(10000);
    
    if (ftdi_write_data(&ctx->ftdi, reset_cmds, sizeof(reset_cmds)) < 0) {
        LOG_ERROR("Failed to write recovery commands");
        return -1;
    }
    
    uint8_t junk[256];
    ftdi_read_data(&ctx->ftdi, junk, sizeof(junk));
    
    LOG_INFO("Chip recovery completed");
    return 0;
}

static int mpsse_buffer_flush(mpsse_context_t *ctx)
{
    struct mpsse_buffer *b = &ctx->buffer;
    
    if (b->tx_num_bytes > 0) {
        int ret = ftdi_write_data(&ctx->ftdi, b->tx_buffer, b->tx_num_bytes);
        if (ret < 0) {
            LOG_ERROR("Failed to write to FTDI: %s", ftdi_get_error_string(&ctx->ftdi));
            mpsse_chip_recovery(ctx);
            return -1;
        }
        if (ret != b->tx_num_bytes) {
            LOG_ERROR("Only wrote %d of %d bytes", ret, b->tx_num_bytes);
            mpsse_chip_recovery(ctx);
            return -1;
        }
        b->tx_num_bytes = 0;
    }
    
    if (b->rx_num_bytes > 0) {
        int bytes_read = 0;
        int timeout_ms = 1000 + (b->rx_num_bytes * 10);
        if (timeout_ms < 2000) timeout_ms = 2000;
        
        LOG_TRACE("Reading %d bytes with timeout %dms", b->rx_num_bytes, timeout_ms);
        
        while (bytes_read < b->rx_num_bytes && timeout_ms > 0) {
            int ret = ftdi_read_data(&ctx->ftdi, b->rx_buffer + bytes_read, b->rx_num_bytes - bytes_read);
            if (ret < 0) {
                LOG_ERROR("Failed to read from FTDI: %s", ftdi_get_error_string(&ctx->ftdi));
                mpsse_chip_recovery(ctx);
                return -1;
            }
            if (ret > 0) {
                bytes_read += ret;
            } else {
                usleep(1000);
                timeout_ms--;
            }
        }
        
        if (bytes_read != b->rx_num_bytes) {
            LOG_ERROR("Only read %d of %d bytes after timeout", bytes_read, b->rx_num_bytes);
            mpsse_chip_recovery(ctx);
            return -1;
        }
        
        for (struct rx_observer_node *o = b->rx_observer_first; o; o = o->next) {
            o->fn(b->rx_buffer + (o->data - b->rx_buffer), o->extra);
        }
        
        b->rx_observer_first = b->rx_observer_last = NULL;
        b->rx_num_bytes = 0;
    }
    
    return 0;
}

struct bit_copier_extra {
    int from_bit;
    uint8_t *dst;
    int to_bit;
    int num_bits;
};

static void bit_copier_rx_observer_fn(const uint8_t *rxData, void *extra)
{
    const struct bit_copier_extra *e = extra;
    copy_bits(rxData, e->from_bit, e->dst, e->to_bit, e->num_bits, false);
}

struct byte_copier_extra {
    uint8_t *dst;
    int num_bytes;
};

static void byte_copier_rx_observer_fn(const uint8_t *rxData, void *extra)
{
    const struct byte_copier_extra *e = extra;
    memcpy(e->dst, rxData, e->num_bytes);
}

struct bulk_byte_copier_extra {
    uint8_t *dst;
    int total_bytes;
    int bytes_copied;
};

static void bulk_byte_copier_rx_observer_fn(const uint8_t *rxData, void *extra)
{
    struct bulk_byte_copier_extra *e = extra;
    memcpy(e->dst + e->bytes_copied, rxData, e->total_bytes - e->bytes_copied);
    e->bytes_copied = e->total_bytes;
}

static int mpsse_buffer_ensure_can_append(mpsse_context_t *ctx, int tx_bytes, int rx_bytes)
{
    struct mpsse_buffer *b = &ctx->buffer;
    
    if (b->tx_num_bytes + tx_bytes > b->max_tx_buffer_bytes ||
        b->rx_num_bytes + rx_bytes > b->max_rx_buffer_bytes) {
        if (mpsse_buffer_flush(ctx) < 0) {
            return -1;
        }
    }
    
    if (tx_bytes > 512 || rx_bytes > 512) {
        if (mpsse_buffer_flush(ctx) < 0) {
            return -1;
        }
    }
    
    return 0;
}

static int mpsse_buffer_append(mpsse_context_t *ctx, const uint8_t *tx_data, int tx_bytes,
                               rx_observer_fn observer, void *observer_extra, int rx_bytes)
{
    struct mpsse_buffer *b = &ctx->buffer;
    
    if (tx_bytes > 0) {
        memcpy(b->tx_buffer + b->tx_num_bytes, tx_data, tx_bytes);
        b->tx_num_bytes += tx_bytes;
    }
    
    if (rx_bytes > 0) {
        if (observer) {
            struct rx_observer_node *node = malloc(sizeof(struct rx_observer_node));
            if (!node) {
                LOG_ERROR("Failed to allocate observer node");
                return -1;
            }
            node->next = NULL;
            node->fn = observer;
            node->data = b->rx_buffer + b->rx_num_bytes;
            node->extra = observer_extra;
            
            if (!b->rx_observer_first) {
                b->rx_observer_first = b->rx_observer_last = node;
            } else {
                b->rx_observer_last->next = node;
                b->rx_observer_last = node;
            }
        }
        b->rx_num_bytes += rx_bytes;
    }
    
    return 0;
}

static int mpsse_buffer_add_write_with_readback(mpsse_context_t *ctx, const uint8_t *tx_data,
                                                  int tx_bytes, rx_observer_fn observer,
                                                  void *observer_extra, int rx_bytes)
{
    if (mpsse_buffer_ensure_can_append(ctx, tx_bytes, rx_bytes) < 0) {
        return -1;
    }
    return mpsse_buffer_append(ctx, tx_data, tx_bytes, observer, observer_extra, rx_bytes);
}

static int mpsse_buffer_add_write_simple(mpsse_context_t *ctx, const uint8_t *tx_data, int tx_bytes,
                                         uint8_t *rx_data, int rx_bytes)
{
    struct byte_copier_extra *extra = malloc(sizeof(struct byte_copier_extra));
    if (!extra) {
        LOG_ERROR("Failed to allocate byte copier extra");
        return -1;
    }
    extra->dst = rx_data;
    extra->num_bytes = rx_bytes;
    
    return mpsse_buffer_add_write_with_readback(ctx, tx_data, tx_bytes,
                                                 byte_copier_rx_observer_fn, extra, rx_bytes);
}

static int append_tms_shift(mpsse_context_t *ctx, const uint8_t *tms, int from_bit_idx, int to_bit_idx)
{
    while (from_bit_idx < to_bit_idx) {
        const int max_tms_bits_per_command = 6;
        const int bits_to_transfer = min(to_bit_idx - from_bit_idx, max_tms_bits_per_command);
        
        uint8_t cmd[] = {
            OP_SHIFT_WR_TMS_FLAG | OP_SHIFT_LSB_FIRST_FLAG | OP_SHIFT_BITMODE_FLAG | OP_SHIFT_WR_FALLING_FLAG,
            bits_to_transfer - 1,
            (!!ctx->last_tdi) << 7,
        };
        copy_bits(tms, from_bit_idx, &cmd[2], 0, bits_to_transfer, true);
        from_bit_idx += bits_to_transfer;
        
        if (mpsse_buffer_append(ctx, cmd, 3, NULL, NULL, 0) < 0) {
            return -1;
        }
    }
    return 0;
}

static int append_tdi_shift(mpsse_context_t *ctx, const uint8_t *tdi, uint8_t *tdo,
                            int from_bit_idx, int to_bit_idx, bool last_tms_bit_high)
{
    const int last_bit_idx = to_bit_idx - 1;
    const int num_regular_bits = last_bit_idx - from_bit_idx;
    const int num_first_octet_bits = 8 - from_bit_idx % 8;
    const int num_leading_bits = min(num_first_octet_bits == 8 ? 0 : num_first_octet_bits, num_regular_bits);
    const bool leading_only = num_leading_bits == num_regular_bits;
    const int inner_end_idx = leading_only ? -1 : last_bit_idx - last_bit_idx % 8;
    const int num_trailing_bits = leading_only ? 0 : last_bit_idx % 8;
    (void)leading_only;
    
    struct bulk_byte_copier_extra *bulk_extra = NULL;
    int total_inner_bytes = 0;
    
    if (inner_end_idx > from_bit_idx && !leading_only) {
        total_inner_bytes = (inner_end_idx - (from_bit_idx + num_leading_bits)) / 8;
        if (total_inner_bytes > 0) {
            bulk_extra = malloc(sizeof(struct bulk_byte_copier_extra));
            if (!bulk_extra) {
                LOG_ERROR("Failed to allocate bulk byte copier extra");
                return -1;
            }
            bulk_extra->dst = tdo + (from_bit_idx + num_leading_bits) / 8;
            bulk_extra->total_bytes = total_inner_bytes;
            bulk_extra->bytes_copied = 0;
        }
    }
    
    for (int cur_idx = from_bit_idx; cur_idx < to_bit_idx;) {
        if (cur_idx == from_bit_idx && num_leading_bits > 0) {
            uint8_t cmd[] = {
                OP_SHIFT_RD_TDO_FLAG | OP_SHIFT_WR_TDI_FLAG | OP_SHIFT_LSB_FIRST_FLAG | OP_SHIFT_BITMODE_FLAG | OP_SHIFT_WR_FALLING_FLAG,
                num_leading_bits - 1,
                tdi[from_bit_idx / 8] >> (from_bit_idx % 8),
            };
            
            struct bit_copier_extra *extra = malloc(sizeof(struct bit_copier_extra));
            if (!extra) {
                LOG_ERROR("Failed to allocate bit copier extra");
                free(bulk_extra);
                return -1;
            }
            extra->from_bit = 8 - num_leading_bits;
            extra->dst = tdo;
            extra->to_bit = from_bit_idx;
            extra->num_bits = num_leading_bits;
            
            if (mpsse_buffer_add_write_with_readback(ctx, cmd, 3, bit_copier_rx_observer_fn, extra, 1) < 0) {
                free(extra);
                free(bulk_extra);
                return -1;
            }
            cur_idx += num_leading_bits;
        }
        
        if (cur_idx < last_bit_idx && inner_end_idx > cur_idx) {
            const int inner_octets_to_send = min((inner_end_idx - cur_idx) / 8, ctx->chip_buffer_size);
            
            uint8_t cmd[] = {
                OP_SHIFT_RD_TDO_FLAG | OP_SHIFT_WR_TDI_FLAG | OP_SHIFT_LSB_FIRST_FLAG | OP_SHIFT_WR_FALLING_FLAG,
                ((inner_octets_to_send - 1) >> 0) & 0xff,
                ((inner_octets_to_send - 1) >> 8) & 0xff,
            };
            
            int remaining_bytes = total_inner_bytes - (cur_idx - (from_bit_idx + num_leading_bits)) / 8;
            bool is_last_chunk = (inner_octets_to_send >= remaining_bytes);
            
            if (mpsse_buffer_append(ctx, cmd, 3, NULL, NULL, 0) < 0) {
                free(bulk_extra);
                return -1;
            }
            
            if (is_last_chunk && bulk_extra) {
                if (mpsse_buffer_add_write_with_readback(ctx, tdi + cur_idx / 8, inner_octets_to_send,
                                                          bulk_byte_copier_rx_observer_fn, bulk_extra, inner_octets_to_send) < 0) {
                    free(bulk_extra);
                    return -1;
                }
            } else {
                if (mpsse_buffer_add_write_simple(ctx, tdi + cur_idx / 8, inner_octets_to_send,
                                                   tdo + cur_idx / 8, inner_octets_to_send) < 0) {
                    free(bulk_extra);
                    return -1;
                }
            }
            
            /* CRITICAL: Flush after each chunk to avoid USB bulk write failures
             * TinyXVC does this implicitly via their buffer management, but we need
             * to be explicit. This prevents building up huge transfers that exceed
             * the chip's ability to handle them.
             */
            if (mpsse_buffer_flush(ctx) < 0) {
                free(bulk_extra);
                return -1;
            }
            
            cur_idx += inner_octets_to_send * 8;
        }
        
        if (num_trailing_bits > 0 && cur_idx < last_bit_idx) {
            uint8_t cmd[] = {
                OP_SHIFT_RD_TDO_FLAG | OP_SHIFT_WR_TDI_FLAG | OP_SHIFT_LSB_FIRST_FLAG | OP_SHIFT_BITMODE_FLAG | OP_SHIFT_WR_FALLING_FLAG,
                num_trailing_bits - 1,
                tdi[inner_end_idx / 8],
            };
            
            struct bit_copier_extra *extra = malloc(sizeof(struct bit_copier_extra));
            if (!extra) {
                LOG_ERROR("Failed to allocate bit copier extra");
                return -1;
            }
            extra->from_bit = 8 - num_trailing_bits;
            extra->dst = tdo;
            extra->to_bit = inner_end_idx;
            extra->num_bits = num_trailing_bits;
            
            if (mpsse_buffer_add_write_with_readback(ctx, cmd, 3, bit_copier_rx_observer_fn, extra, 1) < 0) {
                free(extra);
                return -1;
            }
            cur_idx += num_trailing_bits;
        }
        
        if (cur_idx == last_bit_idx) {
            const int last_tdi_bit = !!get_bit(tdi, last_bit_idx);
            const int last_tms_bit = !!last_tms_bit_high;
            
            uint8_t cmd[] = {
                OP_SHIFT_WR_TMS_FLAG | OP_SHIFT_RD_TDO_FLAG | OP_SHIFT_LSB_FIRST_FLAG | OP_SHIFT_BITMODE_FLAG | OP_SHIFT_WR_FALLING_FLAG,
                0x00,
                (last_tdi_bit << 7) | (last_tms_bit << 1) | last_tms_bit,
            };
            
            struct bit_copier_extra *extra = malloc(sizeof(struct bit_copier_extra));
            if (!extra) {
                LOG_ERROR("Failed to allocate bit copier extra");
                return -1;
            }
            extra->from_bit = 7;
            extra->dst = tdo;
            extra->to_bit = last_bit_idx;
            extra->num_bits = 1;
            
            if (mpsse_buffer_add_write_with_readback(ctx, cmd, 3, bit_copier_rx_observer_fn, extra, 1) < 0) {
                free(extra);
                return -1;
            }
            
            ctx->last_tdi = last_tdi_bit;
            cur_idx += 1;
        }
    }
    return 0;
}

int mpsse_adapter_open(mpsse_context_t *ctx, int vendor, int product, const char *serial,
                        int index, int interface)
{
    if (!ctx) return -1;
    
    if (ftdi_init(&ctx->ftdi) < 0) {
        LOG_ERROR("Failed to initialize FTDI: %s", ftdi_get_error_string(&ctx->ftdi));
        return -1;
    }
    
    if (ftdi_set_interface(&ctx->ftdi, (enum ftdi_interface)interface) < 0) {
        LOG_ERROR("Failed to set FTDI interface: %s", ftdi_get_error_string(&ctx->ftdi));
        return -1;
    }
    
    if (ftdi_usb_open_desc_index(&ctx->ftdi, vendor, product, NULL, serial, index) < 0) {
        LOG_ERROR("Failed to open FTDI device: %s", ftdi_get_error_string(&ctx->ftdi));
        return -1;
    }

    /* Detect chip type and set appropriate buffer sizes
     * FT2232H has 4KB buffer, FT232H has 1KB buffer
     */
    int chip_buffer_size = 4096;  /* Default for FT2232H */
    const char *chip_name = "FT2232H";

    if (ctx->ftdi.type == TYPE_232H) {
        chip_buffer_size = 1024;  /* FT232H has 1KB buffer */
        chip_name = "FT232H";
    }

    /* Store chip buffer size for use in shift operations */
    ctx->chip_buffer_size = chip_buffer_size;

    /* Reallocate buffers with correct size for the chip */
    ctx->buffer.max_tx_buffer_bytes = 3 * chip_buffer_size;
    ctx->buffer.max_rx_buffer_bytes = chip_buffer_size;

    free(ctx->buffer.tx_buffer);
    free(ctx->buffer.rx_buffer);

    ctx->buffer.tx_buffer = malloc(ctx->buffer.max_tx_buffer_bytes);
    ctx->buffer.rx_buffer = malloc(ctx->buffer.max_rx_buffer_bytes);

    if (!ctx->buffer.tx_buffer || !ctx->buffer.rx_buffer) {
        LOG_ERROR("Failed to allocate buffers for %s", chip_name);
        free(ctx->buffer.tx_buffer);
        free(ctx->buffer.rx_buffer);
        ctx->buffer.tx_buffer = NULL;
        ctx->buffer.rx_buffer = NULL;
        return -1;
    }

    LOG_INFO("Detected %s chip, using %d byte buffer", chip_name, chip_buffer_size);

    ftdi_usb_reset(&ctx->ftdi);
    ftdi_setflowctrl(&ctx->ftdi, SIO_RTS_CTS_HS);
    ftdi_set_baudrate(&ctx->ftdi, 115200);
    ftdi_set_bitmode(&ctx->ftdi, 0x00, BITMODE_RESET);
    ftdi_set_bitmode(&ctx->ftdi, 0x00, BITMODE_MPSSE);
    
    int ret = ftdi_set_latency_timer(&ctx->ftdi, MPSSE_DEFAULT_LATENCY);
    if (ret < 0) {
        LOG_WARN("Failed to set latency timer: %d (continuing anyway)", ret);
    } else {
        LOG_DBG("Latency timer set to %dms", MPSSE_DEFAULT_LATENCY);
    }
    
    uint8_t setup_cmds[] = {
        OP_LOOPBACK_OFF,
        OP_SET_TCK_DIVISOR,
        29 & 0xFF,
        (29 >> 8) & 0xFF,
        OP_DISABLE_CLK_DIVIDE_BY_5,
        OP_SET_DBUS_LOBYTE,
        0x08,
        0x0B,
    };
    
    if (ftdi_write_data(&ctx->ftdi, setup_cmds, sizeof(setup_cmds)) < 0) {
        LOG_ERROR("Failed to write setup commands: %s", ftdi_get_error_string(&ctx->ftdi));
        return -1;
    }
    
    uint8_t junk[256];
    ftdi_read_data(&ctx->ftdi, junk, sizeof(junk));
    
    ctx->is_open = true;
    ctx->state = TEST_LOGIC_RESET;
    
    LOG_INFO("MPSSE adapter opened");
    
    return 0;
}

void mpsse_adapter_close(mpsse_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return;
    
    mpsse_buffer_flush(ctx);
    ftdi_usb_close(&ctx->ftdi);
    ctx->is_open = false;
    
    LOG_INFO("MPSSE adapter closed");
}

int mpsse_adapter_set_frequency(mpsse_context_t *ctx, uint32_t frequency_hz)
{
    if (!ctx || !ctx->is_open) return -1;
    
    if (frequency_hz > MPSSE_MAX_FREQUENCY) frequency_hz = MPSSE_MAX_FREQUENCY;
    if (frequency_hz < 1) frequency_hz = 1;
    
    unsigned int divisor = ((60000000 / 2) + frequency_hz - 1) / frequency_hz;
    if (divisor > 0xFFFF) divisor = 0xFFFF;
    if (divisor < 1) divisor = 1;
    
    unsigned int actual = 60000000 / (2 * divisor);
    
    uint8_t cmd[] = {
        OP_SET_TCK_DIVISOR,
        divisor & 0xFF,
        (divisor >> 8) & 0xFF,
        OP_DISABLE_CLK_DIVIDE_BY_5,
    };
    
    if (mpsse_buffer_append(ctx, cmd, 4, NULL, NULL, 0) < 0) {
        return -1;
    }
    
    LOG_INFO("MPSSE frequency: requested=%uHz, actual=%uHz (div=%u)", frequency_hz, actual, divisor);
    
    return actual;
}

int mpsse_adapter_scan(mpsse_context_t *ctx, const uint8_t *tms, const uint8_t *tdi,
                        uint8_t *tdo, int bits)
{
    if (!ctx || !ctx->is_open || !tms || !tdi || !tdo || bits <= 0) return -1;
    
    LOG_TRACE("MPSSE scan: bits=%d, bytes=%d", bits, (bits + 7) / 8);
    
    int first_pending_bit_idx = 0;
    enum jtag_state jtag_state = ctx->state;
    
    for (int bit_idx = 0; bit_idx < bits;) {
        uint8_t tms_byte = tms[bit_idx / 8];
        const int this_round_end_bit_idx = bit_idx + 8 > bits ? bits : bit_idx + 8;
        
        for (; bit_idx < this_round_end_bit_idx; tms_byte >>= 1, bit_idx++) {
            const bool tms_bit = tms_byte & 1;
            const enum jtag_state next_jtag_state = next_state(jtag_state, tms_bit);
            const bool is_shift = jtag_state == SHIFT_DR || jtag_state == SHIFT_IR;
            const bool next_is_shift = next_jtag_state == SHIFT_DR || next_jtag_state == SHIFT_IR;
            const bool entering_shift = !is_shift && next_is_shift;
            const bool leaving_shift = is_shift && !next_is_shift;
            const bool end_of_vector = bit_idx == bits - 1;
            const bool event = end_of_vector || entering_shift || leaving_shift;
            
            if (event) {
                const int next_pending_bit_idx = bit_idx + 1;
                if (is_shift) {
                    if (append_tdi_shift(ctx, tdi, tdo, first_pending_bit_idx, next_pending_bit_idx, leaving_shift) < 0) {
                        LOG_ERROR("MPSSE scan failed");
                        return -1;
                    }
                } else {
                    if (append_tms_shift(ctx, tms, first_pending_bit_idx, next_pending_bit_idx) < 0) {
                        LOG_ERROR("MPSSE scan failed");
                        return -1;
                    }
                }
                first_pending_bit_idx = next_pending_bit_idx;
            }
            jtag_state = next_jtag_state;
        }
    }
    
    if (mpsse_buffer_flush(ctx) < 0) {
        LOG_ERROR("MPSSE flush failed");
        return -1;
    }
    
    ctx->state = jtag_state;
    
    return 0;
}

int mpsse_adapter_flush(mpsse_context_t *ctx)
{
    if (!ctx || !ctx->is_open) return -1;
    return mpsse_buffer_flush(ctx);
}

const char* mpsse_adapter_error(const mpsse_context_t *ctx)
{
    if (!ctx) return "NULL context";
    return ctx->error[0] ? ctx->error : "No error";
}
