/*
 * bit_copy.h - Bit-level copy operations
 * XVC Server for Digilent HS2
 * 
 * Based on OpenOCD's binarybuffer.h implementation.
 */

#ifndef XVC_BIT_COPY_H
#define XVC_BIT_COPY_H

#include <stdint.h>

/**
 * Copy bits from source to destination with arbitrary bit offsets
 * @param dst Destination buffer
 * @param dst_offset Bit offset in destination (0 = LSB of first byte)
 * @param src Source buffer
 * @param src_offset Bit offset in source (0 = LSB of first byte)
 * @param bit_count Number of bits to copy
 */
static inline void bit_copy(uint8_t *dst, unsigned int dst_offset,
                            const uint8_t *src, unsigned int src_offset,
                            unsigned int bit_count)
{
    /* Advance pointers to start bytes */
    src += src_offset / 8;
    dst += dst_offset / 8;
    src_offset %= 8;
    dst_offset %= 8;
    
    /* Fast path: byte-aligned copy */
    if (src_offset == 0 && dst_offset == 0 && (bit_count % 8) == 0) {
        for (unsigned int i = 0; i < bit_count / 8; i++) {
            dst[i] = src[i];
        }
        return;
    }
    
    /* Slow path: bit-by-bit copy */
    for (unsigned int i = 0; i < bit_count; i++) {
        unsigned int si = src_offset + i;
        unsigned int di = dst_offset + i;
        int bit = (src[si / 8] >> (si % 8)) & 1;
        if (bit)
            dst[di / 8] |= (1 << (di % 8));
        else
            dst[di / 8] &= ~(1 << (di % 8));
    }
}

/**
 * Structure to track a pending bit-copy operation
 */
typedef struct {
    int tdo_bit_offset;   /* Destination bit position in TDO buffer */
    int src_byte_offset;  /* Source byte position in read buffer */
    int src_bit_offset;   /* Bit offset within source byte */
    int bit_count;        /* Number of bits to copy */
    int is_tms_response;  /* 1 if from TMS cmd (left-justified), 0 if normal (right-justified) */
    int is_multi_byte;    /* 1 if copying multiple bytes (byte-mode), 0 otherwise */
} bit_copy_entry_t;

/* Maximum number of bit-copy operations per scan */
#define MAX_BIT_COPY_ENTRIES 4096

/**
 * Copy bits from TMS command response (left-justified/MSB-aligned)
 * MPSSE TMS command (0x6b) returns data left-justified in MSB.
 * For N bits read, data is in bits [7..(8-N)].
 * @param dst Destination buffer
 * @param dst_offset Bit offset in destination (0 = LSB of first byte)
 * @param src Source buffer (single byte containing left-justified bits)
 * @param bit_count Number of bits to copy
 */
static inline void bit_copy_tms(uint8_t *dst, unsigned int dst_offset,
                                 const uint8_t *src, unsigned int bit_count)
{
    uint8_t src_byte = src[0];
    
    /* Shift right to align LSB - same as pyftdi: byte >>= 8-bit_count */
    src_byte >>= (8 - bit_count);
    
    /* Advance to destination byte */
    dst += dst_offset / 8;
    dst_offset %= 8;
    
    /* Extract bits one by one from the shifted byte */
    for (unsigned int i = 0; i < bit_count; i++) {
        unsigned int di = dst_offset + i;
        int bit = (src_byte >> i) & 1;
        if (bit)
            dst[di / 8] |= (1 << (di % 8));
        else
            dst[di / 8] &= ~(1 << (di % 8));
    }
}

/**
 * Copy multiple bytes from byte-mode response
 * Byte-mode (0x39) returns N bytes directly in MSB-to-LSB order.
 * @param dst Destination buffer
 * @param dst_offset Bit offset in destination (0 = LSB of first byte)
 * @param src Source buffer (multiple bytes)
 * @param byte_count Number of bytes to copy
 */
static inline void byte_copy(uint8_t *dst, unsigned int dst_offset,
                               const uint8_t *src, unsigned int byte_count)
{
    unsigned int total_bits = byte_count * 8;
    
    for (unsigned int i = 0; i < total_bits; i++) {
        unsigned int src_bit = i;
        unsigned int dst_bit = dst_offset + i;
        
        int bit = (src[src_bit / 8] >> (src_bit % 8)) & 1;
        
        if (bit)
            dst[dst_bit / 8] |= (1 << (dst_bit % 8));
        else
            dst[dst_bit / 8] &= ~(1 << (dst_bit % 8));
    }
}

#endif /* XVC_BIT_COPY_H */
