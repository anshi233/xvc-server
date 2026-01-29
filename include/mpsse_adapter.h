/*
 * mpsse_adapter.h - FTDI MPSSE Adapter Layer (High-Speed JTAG)
 * XVC Server for Digilent HS2
 * 
 * MPSSE mode provides significantly faster JTAG speeds (up to 30MHz)
 * compared to bit-bang mode (~3MHz max).
 */

#ifndef XVC_MPSSE_ADAPTER_H
#define XVC_MPSSE_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>

/* MPSSE mode flags */
#define MPSSE_LSB_FIRST     0x08
#define MPSSE_NEG_EDGE_OUT  0x01
#define MPSSE_NEG_EDGE_IN   0x04
#define MPSSE_WRITE_TMS     0x40

/* JTAG mode: LSB first, data changes on -ve edge, sampled on +ve edge */
#define MPSSE_JTAG_MODE     (MPSSE_LSB_FIRST | MPSSE_NEG_EDGE_OUT)

/* MPSSE commands */
#define MPSSE_CMD_CLOCK_DATA_BYTES_OUT      0x10
#define MPSSE_CMD_CLOCK_DATA_BYTES_IN       0x20
#define MPSSE_CMD_CLOCK_DATA_BITS_OUT       0x02
#define MPSSE_CMD_CLOCK_DATA_BITS_IN        0x02
#define MPSSE_CMD_CLOCK_TMS                 0x4b
#define MPSSE_CMD_SET_LOW_BYTE              0x80
#define MPSSE_CMD_SET_HIGH_BYTE             0x82
#define MPSSE_CMD_READ_LOW_BYTE             0x81
#define MPSSE_CMD_READ_HIGH_BYTE            0x83
#define MPSSE_CMD_SET_DIVISOR               0x86
#define MPSSE_CMD_SEND_IMMEDIATE            0x87
#define MPSSE_CMD_DISABLE_DIV5              0x8a
#define MPSSE_CMD_ENABLE_DIV5               0x8b
#define MPSSE_CMD_DISABLE_ADAPTIVE          0x97
#define MPSSE_CMD_ENABLE_ADAPTIVE           0x96
#define MPSSE_CMD_LOOPBACK_OFF              0x85
#define MPSSE_CMD_LOOPBACK_ON               0x84
#define MPSSE_CMD_DISABLE_3_PHASE_CLOCK      0x8d

/* FTDI pin assignments for JTAG (low byte of GPIO) */
#define MPSSE_PIN_TCK       0x01
#define MPSSE_PIN_TDI       0x02
#define MPSSE_PIN_TDO       0x04
#define MPSSE_PIN_TMS       0x08

/* Direction mask: outputs = TCK, TDI, TMS; input = TDO */
#define MPSSE_DIR_MASK      (MPSSE_PIN_TCK | MPSSE_PIN_TDI | MPSSE_PIN_TMS)

/* Default output state */
#define MPSSE_DEFAULT_OUT   0xE0

/* Buffer sizes */
#define MPSSE_WRITE_BUFFER_SIZE     16384
#define MPSSE_READ_BUFFER_SIZE      16384

/* USB timeouts (ms) */
#define MPSSE_USB_TIMEOUT           10000   /* 10s for large transfers */

/* Max commands per batch (avoid timeout on large scans) */
#define MPSSE_MAX_CMD_BATCH         1024    /* ~340 bits per batch */

/* Speed limits */
#define MPSSE_MAX_FREQUENCY         30000000    /* 30 MHz (FT2232H/FT232H) */
#define MPSSE_DEFAULT_FREQUENCY     15000000    /* 15 MHz (safe default) */

/* MPSSE context (opaque) */
typedef struct mpsse_context_s mpsse_context_t;

/* MPSSE Adapter API */

/**
 * Create MPSSE context
 * @return New context or NULL on error
 */
mpsse_context_t* mpsse_adapter_create(void);

/**
 * Destroy MPSSE context
 */
void mpsse_adapter_destroy(mpsse_context_t *ctx);

/**
 * Open FTDI device in MPSSE mode
 * @param ctx MPSSE context
 * @param vendor Vendor ID (-1 for default 0x0403)
 * @param product Product ID (-1 for default 0x6010)
 * @param serial Serial number (NULL for any)
 * @param index Device index (0 for first)
 * @param interface Interface number (0-3)
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_open(mpsse_context_t *ctx,
                       int vendor, int product,
                       const char *serial,
                       int index, int interface);

/**
 * Close MPSSE device
 */
void mpsse_adapter_close(mpsse_context_t *ctx);

/**
 * Check if device is open
 */
bool mpsse_adapter_is_open(const mpsse_context_t *ctx);

/**
 * Set TCK frequency
 * @param ctx MPSSE context
 * @param frequency_hz Frequency in Hz (max 30MHz)
 * @return Actual frequency achieved, or -1 on error
 */
int mpsse_adapter_set_frequency(mpsse_context_t *ctx, uint32_t frequency_hz);

/**
 * Perform JTAG scan operation
 * @param ctx MPSSE context
 * @param tms TMS data bytes
 * @param tdi TDI data bytes
 * @param tdo Output TDO data bytes
 * @param bits Number of bits to scan
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_scan(mpsse_context_t *ctx,
                       const uint8_t *tms,
                       const uint8_t *tdi,
                       uint8_t *tdo,
                       int bits);

/**
 * Flush command buffer
 * @param ctx MPSSE context
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_flush(mpsse_context_t *ctx);

/**
 * Get last error message
 */
const char* mpsse_adapter_error(const mpsse_context_t *ctx);

/**
 * Set verbosity level
 */
void mpsse_adapter_set_verbose(mpsse_context_t *ctx, int level);

#endif /* XVC_MPSSE_ADAPTER_H */
