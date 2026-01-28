/*
 * logging.h - Logging infrastructure
 * XVC Server for Digilent HS2
 */

#ifndef XVC_LOGGING_H
#define XVC_LOGGING_H

#include <stdarg.h>
#include <stdbool.h>

/* Log levels - prefixed with XVC_ to avoid syslog.h conflicts */
typedef enum {
    XVC_LOG_TRACE = 0,
    XVC_LOG_DEBUG,
    XVC_LOG_INFO,
    XVC_LOG_WARN,
    XVC_LOG_ERROR,
    XVC_LOG_FATAL
} log_level_t;

/* Log output targets */
typedef enum {
    LOG_TARGET_STDOUT = 1,
    LOG_TARGET_STDERR = 2,
    LOG_TARGET_FILE   = 4,
    LOG_TARGET_SYSLOG = 8
} log_target_t;

/* Logging configuration */
typedef struct {
    log_level_t level;
    int targets;            /* Bitmask of log_target_t */
    char log_file[256];
    bool include_timestamp;
    bool include_level;
    bool include_source;
    int instance_id;        /* 0 for main process, >0 for instances */
} log_config_t;

/* Logging API */

/**
 * Initialize logging system
 * @param config Logging configuration
 * @return 0 on success, -1 on error
 */
int log_init(const log_config_t *config);

/**
 * Shutdown logging system
 */
void log_shutdown(void);

/**
 * Set log level
 */
void log_set_level(log_level_t level);

/**
 * Check if logging is enabled for level
 */
bool log_enabled(log_level_t level);

/**
 * Set instance ID for log prefixing
 */
void log_set_instance(int instance_id);

/**
 * Log a message
 * @param level Log level
 * @param file Source file name
 * @param line Source line number
 * @param fmt Printf-style format string
 */
void log_msg(log_level_t level, const char *file, int line, const char *fmt, ...);

/**
 * Log a message with va_list
 */
void log_vmsg(log_level_t level, const char *file, int line, const char *fmt, va_list args);

/* Convenience macros */
#define LOG_TRACE(fmt, ...) log_msg(XVC_LOG_TRACE, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_DBG(fmt, ...)   log_msg(XVC_LOG_DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  log_msg(XVC_LOG_INFO,  __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  log_msg(XVC_LOG_WARN,  __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) log_msg(XVC_LOG_ERROR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) log_msg(XVC_LOG_FATAL, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

/* Get log level name */
const char* log_level_name(log_level_t level);

/* Parse log level from string */
log_level_t log_level_from_string(const char *str);

#endif /* XVC_LOGGING_H */
