/*
 * whitelist.c - IP Whitelist
 * XVC Server for Digilent HS2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include "whitelist.h"
#include "logging.h"

void whitelist_init(whitelist_t *wl, whitelist_mode_t mode)
{
    if (!wl) return;
    
    memset(wl, 0, sizeof(whitelist_t));
    wl->mode = mode;
}

int whitelist_parse_cidr(const char *cidr, uint32_t *ip, int *prefix_len)
{
    if (!cidr || !ip || !prefix_len) return -1;
    
    char buf[64];
    strncpy(buf, cidr, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    
    /* Look for CIDR prefix */
    char *slash = strchr(buf, '/');
    if (slash) {
        *slash = '\0';
        *prefix_len = atoi(slash + 1);
        if (*prefix_len < 0 || *prefix_len > 32) {
            return -1;
        }
    } else {
        *prefix_len = 32;  /* Single IP */
    }
    
    /* Parse IP address */
    struct in_addr addr;
    if (inet_pton(AF_INET, buf, &addr) != 1) {
        return -1;
    }
    
    *ip = addr.s_addr;
    return 0;
}

int whitelist_add(whitelist_t *wl, const char *ip, bool is_block)
{
    if (!wl || !ip) return -1;
    
    if (wl->entry_count >= MAX_WHITELIST_ENTRIES) {
        LOG_ERROR("Whitelist full");
        return -1;
    }
    
    whitelist_entry_t *entry = &wl->entries[wl->entry_count];
    
    /* Parse CIDR */
    uint32_t parsed_ip;
    int prefix_len;
    if (whitelist_parse_cidr(ip, &parsed_ip, &prefix_len) < 0) {
        LOG_ERROR("Invalid IP/CIDR: %s", ip);
        return -1;
    }
    
    strncpy(entry->ip, ip, MAX_IP_LEN - 1);
    entry->prefix_len = prefix_len;
    entry->is_block = is_block;
    
    wl->entry_count++;
    LOG_DBG("Added whitelist entry: %s (block=%d)", ip, is_block);
    
    return 0;
}

static bool ip_matches_entry(uint32_t client_ip, const whitelist_entry_t *entry)
{
    uint32_t entry_ip;
    int prefix_len;
    
    if (whitelist_parse_cidr(entry->ip, &entry_ip, &prefix_len) < 0) {
        return false;
    }
    
    if (prefix_len == 32) {
        return client_ip == entry_ip;
    }
    
    /* Create mask */
    uint32_t mask = htonl(0xFFFFFFFF << (32 - prefix_len));
    return (client_ip & mask) == (entry_ip & mask);
}

whitelist_result_t whitelist_check_ipv4(const whitelist_t *wl, uint32_t ip)
{
    if (!wl) return WHITELIST_ALLOWED;
    
    if (wl->mode == WHITELIST_OFF) {
        return WHITELIST_ALLOWED;
    }
    
    /* Check blocklist first */
    for (int i = 0; i < wl->entry_count; i++) {
        if (wl->entries[i].is_block && ip_matches_entry(ip, &wl->entries[i])) {
            return WHITELIST_BLOCKED;
        }
    }
    
    /* Check allowlist */
    bool in_allowlist = false;
    for (int i = 0; i < wl->entry_count; i++) {
        if (!wl->entries[i].is_block && ip_matches_entry(ip, &wl->entries[i])) {
            in_allowlist = true;
            break;
        }
    }
    
    if (in_allowlist) {
        return WHITELIST_ALLOWED;
    }
    
    /* Not in allowlist */
    if (wl->mode == WHITELIST_STRICT) {
        return WHITELIST_BLOCKED;
    }
    
    /* Permissive mode - allow but log */
    return WHITELIST_LOGGED;
}

whitelist_result_t whitelist_check(const whitelist_t *wl, const struct sockaddr *addr)
{
    if (!wl || !addr) return WHITELIST_ALLOWED;
    
    if (addr->sa_family == AF_INET) {
        const struct sockaddr_in *sin = (const struct sockaddr_in*)addr;
        return whitelist_check_ipv4(wl, sin->sin_addr.s_addr);
    }
    
    /* TODO: IPv6 support */
    return WHITELIST_ALLOWED;
}

int whitelist_load(whitelist_t *wl, const xvc_instance_config_t *instance)
{
    if (!wl || !instance) return -1;
    
    whitelist_init(wl, instance->whitelist_mode);
    
    for (int i = 0; i < instance->whitelist_count; i++) {
        const whitelist_entry_t *src = &instance->whitelist[i];
        if (whitelist_add(wl, src->ip, src->is_block) < 0) {
            return -1;
        }
    }
    
    LOG_DBG("Loaded whitelist: mode=%d, entries=%d", 
              wl->mode, wl->entry_count);
    return 0;
}

const char* whitelist_result_name(whitelist_result_t result)
{
    switch (result) {
        case WHITELIST_ALLOWED: return "ALLOWED";
        case WHITELIST_BLOCKED: return "BLOCKED";
        case WHITELIST_LOGGED:  return "LOGGED";
        default: return "UNKNOWN";
    }
}
