/*
 * device_manager.c - Device Discovery and Management
 * XVC Server for Digilent HS2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>
#include "device_manager.h"
#include "logging.h"

/* libusb context */
static libusb_context *usb_ctx = NULL;

int device_manager_init(device_manager_t *mgr)
{
    if (!mgr) return -1;
    
    memset(mgr, 0, sizeof(device_manager_t));
    
    if (!usb_ctx) {
        int ret = libusb_init(&usb_ctx);
        if (ret < 0) {
            LOG_ERROR("Failed to initialize libusb: %s", libusb_strerror(ret));
            return -1;
        }
    }
    
    mgr->initialized = true;
    return 0;
}

void device_manager_shutdown(device_manager_t *mgr)
{
    if (!mgr) return;
    
    mgr->initialized = false;
    mgr->device_count = 0;
    
    if (usb_ctx) {
        libusb_exit(usb_ctx);
        usb_ctx = NULL;
    }
}

int device_manager_scan(device_manager_t *mgr)
{
    if (!mgr || !mgr->initialized) return -1;
    
    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(usb_ctx, &devs);
    
    if (cnt < 0) {
        LOG_ERROR("Failed to get device list: %s", libusb_strerror((int)cnt));
        return -1;
    }
    
    mgr->device_count = 0;
    
    for (ssize_t i = 0; i < cnt && mgr->device_count < MAX_DEVICES; i++) {
        struct libusb_device_descriptor desc;
        
        int ret = libusb_get_device_descriptor(devs[i], &desc);
        if (ret < 0) continue;
        
        /* Check for supported FTDI devices (FT2232H or FT232H) */
        if (desc.idVendor == FTDI_VENDOR_ID && 
            (desc.idProduct == FT2232H_PRODUCT_ID || desc.idProduct == FT232H_PRODUCT_ID)) {
            hs2_device_t *dev = &mgr->devices[mgr->device_count];
            memset(dev, 0, sizeof(hs2_device_t));
            
            dev->vendor_id = desc.idVendor;
            dev->product_id = desc.idProduct;
            dev->bus_number = libusb_get_bus_number(devs[i]);
            dev->device_address = libusb_get_device_address(devs[i]);
            dev->state = DEVICE_STATE_AVAILABLE;
            
            /* Format bus location */
            snprintf(dev->bus_location, sizeof(dev->bus_location), 
                     "%03d-%03d", dev->bus_number, dev->device_address);
            
            /* Try to get string descriptors */
            libusb_device_handle *handle;
            ret = libusb_open(devs[i], &handle);
            if (ret == 0) {
                if (desc.iManufacturer) {
                    libusb_get_string_descriptor_ascii(handle, desc.iManufacturer,
                        (unsigned char*)dev->manufacturer, sizeof(dev->manufacturer));
                }
                if (desc.iProduct) {
                    libusb_get_string_descriptor_ascii(handle, desc.iProduct,
                        (unsigned char*)dev->product, sizeof(dev->product));
                }
                if (desc.iSerialNumber) {
                    libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                        (unsigned char*)dev->serial, sizeof(dev->serial));
                }
                libusb_close(handle);
            } else {
                LOG_DBG("Cannot open device %s for descriptor read: %s",
                          dev->bus_location, libusb_strerror(ret));
                strcpy(dev->manufacturer, "Unknown");
                strcpy(dev->product, "FTDI Device");
            }
            
            mgr->device_count++;
            LOG_DBG("Found HS2 device: %s (SN: %s)", dev->product, dev->serial);
        }
    }
    
    libusb_free_device_list(devs, 1);
    
    LOG_INFO("Scan complete: found %d HS2 device(s)", mgr->device_count);
    return mgr->device_count;
}

hs2_device_t* device_manager_find(device_manager_t *mgr, const device_id_t *id)
{
    if (!mgr || !id) return NULL;
    
    for (int i = 0; i < mgr->device_count; i++) {
        hs2_device_t *dev = &mgr->devices[i];
        
        switch (id->type) {
            case DEVICE_ID_SERIAL:
                if (strcmp(dev->serial, id->value) == 0) return dev;
                break;
                
            case DEVICE_ID_BUS:
                if (strcmp(dev->bus_location, id->value) == 0) return dev;
                break;
                
            case DEVICE_ID_AUTO:
                if (dev->state == DEVICE_STATE_AVAILABLE) return dev;
                break;
                
            default:
                break;
        }
    }
    
    return NULL;
}

hs2_device_t* device_manager_find_available(device_manager_t *mgr)
{
    if (!mgr) return NULL;
    
    for (int i = 0; i < mgr->device_count; i++) {
        if (mgr->devices[i].state == DEVICE_STATE_AVAILABLE) {
            return &mgr->devices[i];
        }
    }
    return NULL;
}

int device_manager_assign(device_manager_t *mgr, hs2_device_t *device, int instance_id)
{
    if (!mgr || !device) return -1;
    
    if (device->state != DEVICE_STATE_AVAILABLE) {
        LOG_WARN("Device %s already in use by instance %d",
                 device->serial, device->assigned_instance);
        return -1;
    }
    
    device->state = DEVICE_STATE_IN_USE;
    device->assigned_instance = instance_id;
    
    LOG_INFO("Device %s assigned to instance %d", device->serial, instance_id);
    return 0;
}

void device_manager_release(device_manager_t *mgr, hs2_device_t *device)
{
    if (!mgr || !device) return;
    
    LOG_INFO("Device %s released from instance %d", 
             device->serial, device->assigned_instance);
    
    device->state = DEVICE_STATE_AVAILABLE;
    device->assigned_instance = 0;
}

hs2_device_t* device_manager_get(device_manager_t *mgr, int index)
{
    if (!mgr || index < 0 || index >= mgr->device_count) return NULL;
    return &mgr->devices[index];
}

void device_manager_print(const device_manager_t *mgr, bool verbose)
{
    if (!mgr) return;
    
    printf("Detected %d Digilent HS2 device(s):\n\n", mgr->device_count);
    
    for (int i = 0; i < mgr->device_count; i++) {
        const hs2_device_t *dev = &mgr->devices[i];
        
        printf("Device #%d:\n", i + 1);
        printf("  Manufacturer: %s\n", dev->manufacturer[0] ? dev->manufacturer : "Unknown");
        printf("  Product: %s\n", dev->product[0] ? dev->product : "Unknown");
        printf("  Serial Number: %s\n", dev->serial[0] ? dev->serial : "N/A");
        printf("  USB Bus: %03d, Device: %03d\n", dev->bus_number, dev->device_address);
        printf("  Suggested Instance ID: %d\n", i + 1);
        printf("  Suggested Port: %d\n", DEFAULT_BASE_PORT + i);
        
        if (verbose) {
            printf("  Vendor ID: 0x%04X\n", dev->vendor_id);
            printf("  Product ID: 0x%04X\n", dev->product_id);
            printf("  State: %s\n", 
                   dev->state == DEVICE_STATE_AVAILABLE ? "Available" :
                   dev->state == DEVICE_STATE_IN_USE ? "In Use" : "Unknown");
        }
        printf("\n");
    }
}

int device_manager_generate_config(const device_manager_t *mgr, 
                                    xvc_global_config_t *config,
                                    int base_port)
{
    if (!mgr || !config) return -1;
    
    config_init(config);
    config->base_port = base_port;
    
    for (int i = 0; i < mgr->device_count && i < MAX_INSTANCES; i++) {
        const hs2_device_t *dev = &mgr->devices[i];
        xvc_instance_config_t *inst = &config->instances[i];
        
        inst->instance_id = i + 1;
        inst->port = base_port + i;
        inst->enabled = true;
        inst->frequency = DEFAULT_FREQUENCY;
        inst->latency_timer = DEFAULT_LATENCY;
        
        /* Use serial number if available, otherwise bus location */
        if (dev->serial[0]) {
            inst->device_id.type = DEVICE_ID_SERIAL;
            strncpy(inst->device_id.value, dev->serial, MAX_SERIAL_LEN - 1);
            snprintf(inst->alias, MAX_ALIAS_LEN, "HS2-%s", dev->serial);
        } else {
            inst->device_id.type = DEVICE_ID_BUS;
            strncpy(inst->device_id.value, dev->bus_location, MAX_SERIAL_LEN - 1);
            snprintf(inst->alias, MAX_ALIAS_LEN, "HS2-BUS%s", dev->bus_location);
        }
        
        config->instance_count++;
    }
    
    return 0;
}
