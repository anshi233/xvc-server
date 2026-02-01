/*
 * config.c - Configuration Parser
 * XVC Server for Digilent HS2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "logging.h"

/* INI parser state */
typedef enum {
    SECTION_NONE = 0,
    SECTION_INSTANCE_MANAGEMENT,
    SECTION_INSTANCE_MAPPINGS,
    SECTION_INSTANCE_SETTINGS,
    SECTION_INSTANCE_ALIASES,
    SECTION_IP_WHITELIST
} section_t;

/* Helper: trim whitespace */
static char* trim(char *str)
{
    if (!str) return NULL;
    
    while (isspace(*str)) str++;
    if (*str == 0) return str;
    
    char *end = str + strlen(str) - 1;
    while (end > str && isspace(*end)) end--;
    end[1] = '\0';
    
    return str;
}

/* Helper: parse section name */
static section_t parse_section(const char *name)
{
    if (strcmp(name, "instance_management") == 0) return SECTION_INSTANCE_MANAGEMENT;
    if (strcmp(name, "instance_mappings") == 0) return SECTION_INSTANCE_MAPPINGS;
    if (strcmp(name, "instance_settings") == 0) return SECTION_INSTANCE_SETTINGS;
    if (strcmp(name, "instance_aliases") == 0) return SECTION_INSTANCE_ALIASES;
    if (strcmp(name, "ip_whitelist_per_instance") == 0) return SECTION_IP_WHITELIST;
    return SECTION_NONE;
}

void config_init(xvc_global_config_t *config)
{
    memset(config, 0, sizeof(xvc_global_config_t));
    
    config->instance_mgmt_enabled = true;
    config->base_port = DEFAULT_BASE_PORT;
    config->max_instances = MAX_INSTANCES;
    config->log_level = XVC_LOG_INFO;
    
    /* Initialize all instances with defaults */
    for (int i = 0; i < MAX_INSTANCES; i++) {
        config->instances[i].instance_id = i + 1;
        config->instances[i].port = DEFAULT_BASE_PORT + i;
        config->instances[i].frequency = DEFAULT_FREQUENCY;
        config->instances[i].latency_timer = DEFAULT_LATENCY;
        config->instances[i].async_mode = false;
        config->instances[i].jtag_mode = JTAG_MODE_MPSSE;  /* Default to fast MPSSE mode */
        config->instances[i].xvc_buffer_size = DEFAULT_XVC_BUFFER_SIZE;
        config->instances[i].whitelist_mode = WHITELIST_OFF;
        config->instances[i].enabled = false;
    }
}

int config_parse_device_id(const char *str, device_id_t *id)
{
    if (!str || !id) return -1;
    
    memset(id, 0, sizeof(device_id_t));
    
    if (strncmp(str, "SN:", 3) == 0) {
        id->type = DEVICE_ID_SERIAL;
        strncpy(id->value, str + 3, MAX_SERIAL_LEN - 1);
    } else if (strncmp(str, "BUS:", 4) == 0) {
        id->type = DEVICE_ID_BUS;
        strncpy(id->value, str + 4, MAX_SERIAL_LEN - 1);
    } else if (strncmp(str, "CUSTOM:", 7) == 0) {
        id->type = DEVICE_ID_CUSTOM;
        strncpy(id->value, str + 7, MAX_SERIAL_LEN - 1);
    } else if (strcmp(str, "auto") == 0) {
        id->type = DEVICE_ID_AUTO;
    } else {
        return -1;
    }
    
    return 0;
}

int config_format_device_id(const device_id_t *id, char *buf, size_t len)
{
    if (!id || !buf || len == 0) return -1;
    
    switch (id->type) {
        case DEVICE_ID_SERIAL:
            return snprintf(buf, len, "SN:%s", id->value);
        case DEVICE_ID_BUS:
            return snprintf(buf, len, "BUS:%s", id->value);
        case DEVICE_ID_CUSTOM:
            return snprintf(buf, len, "CUSTOM:%s", id->value);
        case DEVICE_ID_AUTO:
            return snprintf(buf, len, "auto");
        default:
            return snprintf(buf, len, "none");
    }
}

int config_load(xvc_global_config_t *config, const char *path)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        LOG_ERROR("Cannot open config file: %s", path);
        return -1;
    }
    
    config_init(config);
    
    char line[512];
    section_t section = SECTION_NONE;
    int line_num = 0;
    
    while (fgets(line, sizeof(line), fp)) {
        line_num++;
        
        char *ptr = trim(line);
        
        /* Skip empty lines and comments */
        if (!*ptr || *ptr == '#' || *ptr == ';') continue;
        
        /* Convert trailing comments to null terminator */
        char *comment = strchr(ptr, '#');
        if (comment) *comment = '\0';
        comment = strchr(ptr, ';');
        if (comment) *comment = '\0';
        
        /* Re-trim after stripping comment */
        ptr = trim(ptr);
        if (!*ptr) continue;
        
        /* Section header */
        if (*ptr == '[') {
            char *end = strchr(ptr, ']');
            if (!end) {
                LOG_WARN("Config line %d: malformed section header", line_num);
                continue;
            }
            *end = '\0';
            section = parse_section(ptr + 1);
            continue;
        }
        
        /* Key=Value */
        char *eq = strchr(ptr, '=');
        if (!eq) {
            LOG_WARN("Config line %d: missing = in key/value", line_num);
            continue;
        }
        
        *eq = '\0';
        char *key = trim(ptr);
        char *value = trim(eq + 1);
        
        switch (section) {
            case SECTION_INSTANCE_MANAGEMENT:
                if (strcmp(key, "enabled") == 0) {
                    config->instance_mgmt_enabled = (strcmp(value, "true") == 0 || strcmp(value, "1") == 0);
                } else if (strcmp(key, "base_port") == 0) {
                    config->base_port = atoi(value);
                } else if (strcmp(key, "max_instances") == 0) {
                    config->max_instances = atoi(value);
                }
                break;
                
            case SECTION_INSTANCE_MAPPINGS: {
                int id = atoi(key);
                if (id >= 1 && id <= MAX_INSTANCES) {
                    xvc_instance_config_t *inst = &config->instances[id - 1];
                    inst->enabled = true;
                    inst->port = config->base_port + id - 1;
                    config_parse_device_id(value, &inst->device_id);
                    if (id > config->instance_count) {
                        config->instance_count = id;
                    }
                }
                break;
            }
            
            case SECTION_INSTANCE_SETTINGS: {
                /* Parse "id:setting" format */
                char *colon = strchr(key, ':');
                if (colon) {
                    *colon = '\0';
                    int id = atoi(key);
                    char *setting = colon + 1;
                    
                    if (id >= 1 && id <= MAX_INSTANCES) {
                        xvc_instance_config_t *inst = &config->instances[id - 1];
                        
                        if (strcmp(setting, "frequency") == 0) {
                            inst->frequency = strtoul(value, NULL, 0);
                        } else if (strcmp(setting, "latency_timer") == 0) {
                            inst->latency_timer = atoi(value);
                        } else if (strcmp(setting, "async") == 0) {
                            inst->async_mode = (strcmp(value, "true") == 0 || strcmp(value, "1") == 0);
                        } else if (strcmp(setting, "jtag_mode") == 0) {
                            if (strcmp(value, "bitbang") == 0) {
                                inst->jtag_mode = JTAG_MODE_BITBANG;
                            } else {
                                inst->jtag_mode = JTAG_MODE_MPSSE;  /* default */
                            }
                        } else if (strcmp(setting, "xvc_buffer_size") == 0) {
                            int size = atoi(value);
                            if (size >= 2048 && size <= MAX_XVC_BUFFER_SIZE) {
                                inst->xvc_buffer_size = size;
                            } else {
                                LOG_WARN("Invalid xvc_buffer_size %d, using default %d",
                                         size, DEFAULT_XVC_BUFFER_SIZE);
                                inst->xvc_buffer_size = DEFAULT_XVC_BUFFER_SIZE;
                            }
                        }
                    }
                }
                break;
            }
            
            case SECTION_INSTANCE_ALIASES: {
                int id = atoi(key);
                if (id >= 1 && id <= MAX_INSTANCES) {
                    strncpy(config->instances[id - 1].alias, value, MAX_ALIAS_LEN - 1);
                }
                break;
            }
            
            case SECTION_IP_WHITELIST: {
                /* Parse "id:mode" or "id:allow_N" or "id:block_N" */
                char *colon = strchr(key, ':');
                if (colon) {
                    *colon = '\0';
                    int id = atoi(key);
                    char *setting = colon + 1;
                    
                    if (id >= 1 && id <= MAX_INSTANCES) {
                        xvc_instance_config_t *inst = &config->instances[id - 1];
                        
                        if (strcmp(setting, "mode") == 0) {
                            if (strcmp(value, "strict") == 0) {
                                inst->whitelist_mode = WHITELIST_STRICT;
                            } else if (strcmp(value, "permissive") == 0) {
                                inst->whitelist_mode = WHITELIST_PERMISSIVE;
                            } else {
                                inst->whitelist_mode = WHITELIST_OFF;
                            }
                        } else if (strncmp(setting, "allow_", 6) == 0 || strncmp(setting, "block_", 6) == 0) {
                            if (inst->whitelist_count < MAX_WHITELIST_ENTRIES) {
                                whitelist_entry_t *entry = &inst->whitelist[inst->whitelist_count++];
                                strncpy(entry->ip, value, MAX_IP_LEN - 1);
                                entry->is_block = (setting[0] == 'b');
                            }
                        }
                    }
                }
                break;
            }
            
            default:
                break;
        }
    }
    
    fclose(fp);
    LOG_INFO("Loaded config from %s: %d instances", path, config->instance_count);
    return 0;
}

int config_save(const xvc_global_config_t *config, const char *path)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        LOG_ERROR("Cannot create config file: %s", path);
        return -1;
    }
    
    fprintf(fp, "# XVC Server Configuration\n");
    fprintf(fp, "# Generated automatically\n\n");
    
    /* Instance management section */
    fprintf(fp, "[instance_management]\n");
    fprintf(fp, "enabled = %s\n", config->instance_mgmt_enabled ? "true" : "false");
    fprintf(fp, "base_port = %d\n", config->base_port);
    fprintf(fp, "max_instances = %d\n\n", config->max_instances);
    
    /* Instance mappings section */
    fprintf(fp, "[instance_mappings]\n");
    fprintf(fp, "# Format: instance_id = device_id\n");
    fprintf(fp, "# device_id: SN:serial, BUS:bus-dev, CUSTOM:name, or auto\n");
    
    for (int i = 0; i < config->instance_count; i++) {
        const xvc_instance_config_t *inst = &config->instances[i];
        if (inst->enabled) {
            char id_str[128];
            config_format_device_id(&inst->device_id, id_str, sizeof(id_str));
            fprintf(fp, "%d = %s\n", inst->instance_id, id_str);
        }
    }
    fprintf(fp, "\n");
    
    /* Instance settings section */
    fprintf(fp, "[instance_settings]\n");
    fprintf(fp, "# Format: instance_id:setting = value\n");
    
    for (int i = 0; i < config->instance_count; i++) {
        const xvc_instance_config_t *inst = &config->instances[i];
        if (inst->enabled) {
            fprintf(fp, "%d:frequency = %u\n", inst->instance_id, inst->frequency);
            if (inst->latency_timer != DEFAULT_LATENCY) {
                fprintf(fp, "%d:latency_timer = %d\n", inst->instance_id, inst->latency_timer);
            }
            if (inst->xvc_buffer_size != DEFAULT_XVC_BUFFER_SIZE) {
                fprintf(fp, "%d:xvc_buffer_size = %d\n", inst->instance_id, inst->xvc_buffer_size);
            }
        }
    }
    fprintf(fp, "\n");
    
    /* Instance aliases section */
    fprintf(fp, "[instance_aliases]\n");
    for (int i = 0; i < config->instance_count; i++) {
        const xvc_instance_config_t *inst = &config->instances[i];
        if (inst->enabled && inst->alias[0]) {
            fprintf(fp, "%d = %s\n", inst->instance_id, inst->alias);
        }
    }
    fprintf(fp, "\n");
    
    fclose(fp);
    LOG_INFO("Saved config to %s", path);
    return 0;
}

void config_free(xvc_global_config_t *config)
{
    /* Nothing to free currently */
    (void)config;
}

xvc_instance_config_t* config_get_instance(xvc_global_config_t *config, int instance_id)
{
    if (instance_id < 1 || instance_id > MAX_INSTANCES) {
        return NULL;
    }
    return &config->instances[instance_id - 1];
}
