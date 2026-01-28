/*
 * config.h - Configuration structures and parser API
 * XVC Server for Digilent HS2
 */

#ifndef XVC_CONFIG_H
#define XVC_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum limits */
#define MAX_INSTANCES       32
#define MAX_SERIAL_LEN      64
#define MAX_ALIAS_LEN       64
#define MAX_PATH_LEN        256
#define MAX_WHITELIST_ENTRIES 64
#define MAX_IP_LEN          46  /* IPv6 max length */

/* Default values */
#define DEFAULT_BASE_PORT   2542
#define DEFAULT_FREQUENCY   10000000  /* 10 MHz */
#define DEFAULT_LATENCY     2         /* ms */

/* Device ID types */
typedef enum {
    DEVICE_ID_NONE = 0,
    DEVICE_ID_SERIAL,       /* SN:ABC12345 */
    DEVICE_ID_BUS,          /* BUS:001-002 */
    DEVICE_ID_CUSTOM,       /* CUSTOM:name */
    DEVICE_ID_AUTO          /* auto */
} device_id_type_t;

/* Whitelist mode */
typedef enum {
    WHITELIST_OFF = 0,      /* All IPs allowed */
    WHITELIST_PERMISSIVE,   /* Log non-whitelisted but allow */
    WHITELIST_STRICT        /* Block non-whitelisted */
} whitelist_mode_t;

/* JTAG adapter mode */
typedef enum {
    JTAG_MODE_MPSSE = 0,    /* MPSSE mode (default, fast, up to 30MHz) */
    JTAG_MODE_BITBANG       /* Bit-bang mode (legacy, slow, max 3MHz) */
} jtag_mode_t;

/* Device identifier */
typedef struct {
    device_id_type_t type;
    char value[MAX_SERIAL_LEN];
} device_id_t;

/* Whitelist entry */
typedef struct {
    char ip[MAX_IP_LEN];
    int prefix_len;         /* CIDR prefix length, -1 for single IP */
    bool is_block;          /* true = blocklist, false = allowlist */
} whitelist_entry_t;

/* Per-instance configuration */
typedef struct {
    int instance_id;
    int port;
    device_id_t device_id;
    char alias[MAX_ALIAS_LEN];
    
    /* Device settings */
    uint32_t frequency;     /* TCK frequency in Hz */
    int latency_timer;      /* FTDI latency timer in ms */
    bool async_mode;        /* Use async FTDI operations */
    jtag_mode_t jtag_mode;  /* JTAG adapter mode (MPSSE or bitbang) */
    
    /* Whitelist settings */
    whitelist_mode_t whitelist_mode;
    whitelist_entry_t whitelist[MAX_WHITELIST_ENTRIES];
    int whitelist_count;
    
    /* Runtime state */
    bool enabled;
    pid_t pid;              /* Child process PID */
} xvc_instance_config_t;

/* Global configuration */
typedef struct {
    bool instance_mgmt_enabled;
    int base_port;
    int max_instances;
    
    /* Instance configurations */
    xvc_instance_config_t instances[MAX_INSTANCES];
    int instance_count;
    
    /* Global settings */
    char log_file[MAX_PATH_LEN];
    int log_level;
    bool daemonize;
} xvc_global_config_t;

/* Configuration API */

/**
 * Initialize configuration with defaults
 */
void config_init(xvc_global_config_t *config);

/**
 * Load configuration from INI file
 * @param config Configuration structure to populate
 * @param path Path to INI file
 * @return 0 on success, -1 on error
 */
int config_load(xvc_global_config_t *config, const char *path);

/**
 * Save configuration to INI file
 * @param config Configuration structure
 * @param path Path to output file
 * @return 0 on success, -1 on error
 */
int config_save(const xvc_global_config_t *config, const char *path);

/**
 * Free configuration resources
 */
void config_free(xvc_global_config_t *config);

/**
 * Parse device ID string (e.g., "SN:ABC12345")
 * @param str Input string
 * @param id Output device ID structure
 * @return 0 on success, -1 on error
 */
int config_parse_device_id(const char *str, device_id_t *id);

/**
 * Format device ID to string
 * @param id Device ID structure
 * @param buf Output buffer
 * @param len Buffer length
 * @return Number of characters written
 */
int config_format_device_id(const device_id_t *id, char *buf, size_t len);

/**
 * Get instance by ID
 * @param config Global configuration
 * @param instance_id Instance ID (1-based)
 * @return Pointer to instance config or NULL
 */
xvc_instance_config_t* config_get_instance(xvc_global_config_t *config, int instance_id);

#endif /* XVC_CONFIG_H */
