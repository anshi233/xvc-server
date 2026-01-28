/*
 * whitelist.h - IP Whitelist
 * XVC Server for Digilent HS2
 */

#ifndef XVC_WHITELIST_H
#define XVC_WHITELIST_H

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>
#include "config.h"

/* Whitelist check result */
typedef enum {
    WHITELIST_ALLOWED = 0,
    WHITELIST_BLOCKED,
    WHITELIST_LOGGED      /* Allowed but logged (permissive mode) */
} whitelist_result_t;

/* Whitelist context */
typedef struct {
    whitelist_mode_t mode;
    whitelist_entry_t entries[MAX_WHITELIST_ENTRIES];
    int entry_count;
} whitelist_t;

/* Whitelist API */

/**
 * Initialize whitelist
 * @param wl Whitelist context
 * @param mode Whitelist mode
 */
void whitelist_init(whitelist_t *wl, whitelist_mode_t mode);

/**
 * Add entry to whitelist
 * @param wl Whitelist context
 * @param ip IP address or CIDR
 * @param is_block true for blocklist entry
 * @return 0 on success, -1 on error
 */
int whitelist_add(whitelist_t *wl, const char *ip, bool is_block);

/**
 * Check if IP is allowed
 * @param wl Whitelist context
 * @param addr Socket address
 * @return Whitelist result
 */
whitelist_result_t whitelist_check(const whitelist_t *wl, const struct sockaddr *addr);

/**
 * Check if IPv4 address is allowed
 * @param wl Whitelist context
 * @param ip IPv4 address in network byte order
 * @return Whitelist result
 */
whitelist_result_t whitelist_check_ipv4(const whitelist_t *wl, uint32_t ip);

/**
 * Parse CIDR notation
 * @param cidr CIDR string (e.g., "192.168.1.0/24")
 * @param ip Output IP address (network byte order)
 * @param prefix_len Output prefix length
 * @return 0 on success, -1 on error
 */
int whitelist_parse_cidr(const char *cidr, uint32_t *ip, int *prefix_len);

/**
 * Load whitelist from instance config
 * @param wl Whitelist context
 * @param instance Instance configuration
 * @return 0 on success, -1 on error
 */
int whitelist_load(whitelist_t *wl, const xvc_instance_config_t *instance);

/**
 * Get result name
 */
const char* whitelist_result_name(whitelist_result_t result);

#endif /* XVC_WHITELIST_H */
