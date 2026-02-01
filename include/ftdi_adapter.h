/*
 * ftdi_adapter.h - FTDI Adapter Layer
 * XVC Server for Digilent HS2
 */

#ifndef XVC_FTDI_ADAPTER_H
#define XVC_FTDI_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>

/* JTAG port bit definitions */
#define FTDI_PORT_TCK   0x01
#define FTDI_PORT_TDI   0x02
#define FTDI_PORT_TDO   0x04
#define FTDI_PORT_TMS   0x08
#define FTDI_PORT_MISC  0x90

/* Default output state */
#define FTDI_DEFAULT_OUT    0xE0

/* Default settings */
#define FTDI_DEFAULT_LATENCY    16      /* ms - increased for better stability */
#define FTDI_DEFAULT_BAUDRATE   1000000 /* 1MHz for fast initial JTAG speed */

/* Adapter mode */
typedef enum {
    ADAPTER_MODE_MPSSE = 0,     /* MPSSE mode (fast, default) */
    ADAPTER_MODE_BITBANG        /* Bit-bang mode (legacy fallback) */
} adapter_mode_t;

/* FTDI context (opaque) */
typedef struct ftdi_context_s ftdi_context_t;

/* FTDI Adapter API */

/**
 * Create FTDI context
 * @return New context or NULL on error
 */
ftdi_context_t* ftdi_adapter_create(void);

/**
 * Destroy FTDI context
 */
void ftdi_adapter_destroy(ftdi_context_t *ctx);

/**
 * Open FTDI device
 * @param ctx FTDI context
 * @param vendor Vendor ID (-1 for default 0x0403)
 * @param product Product ID (-1 for default 0x6010)
 * @param serial Serial number (NULL for any)
 * @param index Device index (0 for first)
 * @param interface Interface number (0-3)
 * @return 0 on success, -1 on error
 */
int ftdi_adapter_open(ftdi_context_t *ctx, 
                      int vendor, int product,
                      const char *serial,
                      int index, int interface);

/**
 * Open FTDI device with specific mode
 * @param ctx FTDI context
 * @param vendor Vendor ID (-1 for default 0x0403)
 * @param product Product ID (-1 for default 0x6010)
 * @param serial Serial number (NULL for any)
 * @param index Device index (0 for first)
 * @param interface Interface number (0-3)
 * @param mode Adapter mode (MPSSE or bitbang)
 * @return 0 on success, -1 on error
 */
int ftdi_adapter_open_with_mode(ftdi_context_t *ctx, 
                                int vendor, int product,
                                const char *serial,
                                int index, int interface,
                                adapter_mode_t mode);

/**
 * Get current adapter mode
 */
adapter_mode_t ftdi_adapter_get_mode(const ftdi_context_t *ctx);

/**
 * Open FTDI device by bus location
 * @param ctx FTDI context
 * @param bus USB bus number
 * @param device USB device address
 * @param interface Interface number (0-3)
 * @return 0 on success, -1 on error
 */
int ftdi_adapter_open_bus(ftdi_context_t *ctx,
                          int bus, int device,
                          int interface);

/**
 * Close FTDI device
 */
void ftdi_adapter_close(ftdi_context_t *ctx);

/**
 * Check if device is open
 */
bool ftdi_adapter_is_open(const ftdi_context_t *ctx);

/**
 * Set TCK period
 * @param ctx FTDI context
 * @param period_ns Desired period in nanoseconds
 * @return Actual period in nanoseconds, or -1 on error
 */
int ftdi_adapter_set_period(ftdi_context_t *ctx, unsigned int period_ns);

/**
 * Set TCK frequency
 * @param ctx FTDI context
 * @param frequency_hz Frequency in Hz
 * @return 0 on success, -1 on error
 */
int ftdi_adapter_set_frequency(ftdi_context_t *ctx, uint32_t frequency_hz);

/**
 * Perform JTAG scan operation
 * @param ctx FTDI context
 * @param tms TMS data bytes
 * @param tdi TDI data bytes
 * @param tdo Output TDO data bytes
 * @param bits Number of bits to scan
 * @return 0 on success, -1 on error
 */
int ftdi_adapter_scan(ftdi_context_t *ctx,
                      const uint8_t *tms,
                      const uint8_t *tdi,
                      uint8_t *tdo,
                      int bits);

/**
 * Get last error message
 */
const char* ftdi_adapter_error(const ftdi_context_t *ctx);

/**
 * Set verbosity level
 */
void ftdi_adapter_set_verbose(ftdi_context_t *ctx, int level);

/**
 * Set latency timer
 * @param ctx FTDI context
 * @param latency_ms Latency in milliseconds
 * @return 0 on success, -1 on error
 */
int ftdi_adapter_set_latency(ftdi_context_t *ctx, int latency_ms);

#endif /* XVC_FTDI_ADAPTER_H */
