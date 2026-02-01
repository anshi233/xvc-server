/*
 * mpsse_adapter.h - FTDI MPSSE Adapter Layer (High-Speed JTAG)
 * XVC Server for Digilent HS2
 */

#ifndef XVC_MPSSE_ADAPTER_H
#define XVC_MPSSE_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>

/* MPSSE context (opaque) */
typedef struct mpsse_context_s mpsse_context_t;

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
 * Open MPSSE device
 * @param ctx MPSSE context
 * @param vendor Vendor ID
 * @param product Product ID
 * @param serial Serial number (NULL for any)
 * @param index Device index
 * @param interface Interface number
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_open(mpsse_context_t *ctx, int vendor, int product, 
                       const char *serial, int index, int interface);

/**
 * Close MPSSE device
 */
void mpsse_adapter_close(mpsse_context_t *ctx);

/**
 * Set TCK frequency
 * @param ctx MPSSE context
 * @param frequency_hz Frequency in Hz
 * @return Actual frequency in Hz, or -1 on error
 */
int mpsse_adapter_set_frequency(mpsse_context_t *ctx, uint32_t frequency_hz);

/**
 * Set buffer size for MPSSE operations
 * @param ctx MPSSE context
 * @param buffer_size Buffer size in bytes (2048 to 131072)
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_set_buffer_size(mpsse_context_t *ctx, int buffer_size);

/**
 * Perform JTAG scan operation
 * @param ctx MPSSE context
 * @param tms TMS data bytes
 * @param tdi TDI data bytes
 * @param tdo Output TDO data bytes
 * @param bits Number of bits to scan
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_scan(mpsse_context_t *ctx, const uint8_t *tms,
                       const uint8_t *tdi, uint8_t *tdo, int bits);

/**
 * Flush any pending data
 * @param ctx MPSSE context
 * @return 0 on success, -1 on error
 */
int mpsse_adapter_flush(mpsse_context_t *ctx);

/**
 * Get last error message
 * @param ctx MPSSE context
 * @return Error string
 */
const char* mpsse_adapter_error(const mpsse_context_t *ctx);

#endif /* XVC_MPSSE_ADAPTER_H */
