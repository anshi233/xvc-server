/*
 * device_manager.h - Device discovery and management
 * XVC Server for Digilent HS2
 */

#ifndef XVC_DEVICE_MANAGER_H
#define XVC_DEVICE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* FTDI USB IDs for supported devices */
#define FTDI_VENDOR_ID      0x0403  /* FTDI */
#define FT2232H_PRODUCT_ID  0x6010  /* Digilent HS2 uses this */
#define FT232H_PRODUCT_ID   0x6014  /* Generic FT232H */

/* Legacy defines for compatibility */
#define HS2_VENDOR_ID   FTDI_VENDOR_ID
#define HS2_PRODUCT_ID  FT2232H_PRODUCT_ID

/* Maximum devices to track */
#define MAX_DEVICES     32

/* Device state */
typedef enum {
    DEVICE_STATE_UNKNOWN = 0,
    DEVICE_STATE_AVAILABLE,
    DEVICE_STATE_IN_USE,
    DEVICE_STATE_ERROR
} device_state_t;

/* HS2 device information */
typedef struct {
    /* USB identification */
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t bus_number;
    uint8_t device_address;
    
    /* Device strings */
    char serial[MAX_SERIAL_LEN];
    char manufacturer[64];
    char product[64];
    
    /* Computed identifiers */
    char bus_location[16];  /* "001-002" format */
    
    /* State */
    device_state_t state;
    int assigned_instance;  /* 0 if not assigned */
} hs2_device_t;

/* Device manager context */
typedef struct {
    hs2_device_t devices[MAX_DEVICES];
    int device_count;
    bool initialized;
} device_manager_t;

/* Device Manager API */

/**
 * Initialize device manager
 * @param mgr Device manager context
 * @return 0 on success, -1 on error
 */
int device_manager_init(device_manager_t *mgr);

/**
 * Shutdown device manager
 */
void device_manager_shutdown(device_manager_t *mgr);

/**
 * Scan for HS2 devices
 * @param mgr Device manager context
 * @return Number of devices found, or -1 on error
 */
int device_manager_scan(device_manager_t *mgr);

/**
 * Find device matching identifier
 * @param mgr Device manager context
 * @param id Device identifier
 * @return Pointer to device or NULL
 */
hs2_device_t* device_manager_find(device_manager_t *mgr, const device_id_t *id);

/**
 * Find first available device
 * @param mgr Device manager context
 * @return Pointer to device or NULL
 */
hs2_device_t* device_manager_find_available(device_manager_t *mgr);

/**
 * Mark device as in-use by instance
 * @param mgr Device manager context
 * @param device Device to mark
 * @param instance_id Instance ID
 * @return 0 on success, -1 on error
 */
int device_manager_assign(device_manager_t *mgr, hs2_device_t *device, int instance_id);

/**
 * Release device
 * @param mgr Device manager context
 * @param device Device to release
 */
void device_manager_release(device_manager_t *mgr, hs2_device_t *device);

/**
 * Get device by index
 * @param mgr Device manager context
 * @param index Device index (0-based)
 * @return Pointer to device or NULL
 */
hs2_device_t* device_manager_get(device_manager_t *mgr, int index);

/**
 * Print device list to stdout
 * @param mgr Device manager context
 * @param verbose Include detailed info
 */
void device_manager_print(const device_manager_t *mgr, bool verbose);

/**
 * Generate configuration for discovered devices
 * @param mgr Device manager context
 * @param config Output configuration
 * @param base_port Starting port number
 * @return 0 on success, -1 on error
 */
int device_manager_generate_config(const device_manager_t *mgr, 
                                    xvc_global_config_t *config,
                                    int base_port);

#endif /* XVC_DEVICE_MANAGER_H */
