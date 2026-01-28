/*
 * logging.c - Logging Implementation
 * XVC Server for Digilent HS2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <pthread.h>

/* Include syslog BEFORE logging.h to get system constants first */
#include <syslog.h>

/* Undefine syslog macros we'll override */
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARNING
#undef LOG_ERR
#undef LOG_CRIT

/* Now include our logging header */
#include "logging.h"

/* Global logging state */
static struct {
    log_config_t config;
    FILE *log_file;
    pthread_mutex_t mutex;
    bool initialized;
} g_log = {0};

/* Log level names */
static const char *level_names[] = {
    "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
};

/* Syslog priority mapping */
static const int syslog_priority[] = {
    7, 7, 6, 4, 3, 2
};

int log_init(const log_config_t *config)
{
    if (g_log.initialized) {
        return 0;
    }
    
    memcpy(&g_log.config, config, sizeof(log_config_t));
    
    if (pthread_mutex_init(&g_log.mutex, NULL) != 0) {
        return -1;
    }
    
    /* Open log file if specified */
    if ((config->targets & LOG_TARGET_FILE) && config->log_file[0]) {
        g_log.log_file = fopen(config->log_file, "a");
        if (!g_log.log_file) {
            perror("Failed to open log file");
            return -1;
        }
        setbuf(g_log.log_file, NULL);  /* Unbuffered */
    }
    
    /* Open syslog if specified */
    if (config->targets & LOG_TARGET_SYSLOG) {
        openlog("xvc-server", LOG_PID | LOG_NDELAY, LOG_DAEMON);
    }
    
    g_log.initialized = true;
    return 0;
}

void log_shutdown(void)
{
    if (!g_log.initialized) {
        return;
    }
    
    pthread_mutex_lock(&g_log.mutex);
    
    if (g_log.log_file) {
        fclose(g_log.log_file);
        g_log.log_file = NULL;
    }
    
    if (g_log.config.targets & LOG_TARGET_SYSLOG) {
        closelog();
    }
    
    pthread_mutex_unlock(&g_log.mutex);
    pthread_mutex_destroy(&g_log.mutex);
    
    g_log.initialized = false;
}

void log_set_level(log_level_t level)
{
    g_log.config.level = level;
}

void log_set_instance(int instance_id)
{
    g_log.config.instance_id = instance_id;
}

bool log_enabled(log_level_t level)
{
    return level >= g_log.config.level;
}

void log_msg(log_level_t level, const char *file, int line, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_vmsg(level, file, line, fmt, args);
    va_end(args);
}

void log_vmsg(log_level_t level, const char *file, int line, const char *fmt, va_list args)
{
    if (level < g_log.config.level) {
        return;
    }
    
    char timestamp[32] = "";
    char prefix[128] = "";
    char message[1024] = "";
    
    /* Format timestamp */
    if (g_log.config.include_timestamp) {
        time_t now = time(NULL);
        struct tm *tm = localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm);
    }
    
    /* Build prefix */
    int pos = 0;
    
    if (g_log.config.include_timestamp) {
        pos += snprintf(prefix + pos, sizeof(prefix) - pos, "[%s] ", timestamp);
    }
    
    if (g_log.config.instance_id > 0) {
        pos += snprintf(prefix + pos, sizeof(prefix) - pos, "[I%d] ", g_log.config.instance_id);
    }
    
    if (g_log.config.include_level) {
        pos += snprintf(prefix + pos, sizeof(prefix) - pos, "[%s] ", level_names[level]);
    }
    
    if (g_log.config.include_source && file) {
        const char *basename = strrchr(file, '/');
        basename = basename ? basename + 1 : file;
        pos += snprintf(prefix + pos, sizeof(prefix) - pos, "%s:%d: ", basename, line);
    }
    
    /* Format message */
    vsnprintf(message, sizeof(message), fmt, args);
    
    /* Lock for thread-safe output */
    if (g_log.initialized) {
        pthread_mutex_lock(&g_log.mutex);
    }
    
    /* Output to targets */
    if (g_log.config.targets & LOG_TARGET_STDOUT) {
        printf("%s%s\n", prefix, message);
        fflush(stdout);
    }
    
    if (g_log.config.targets & LOG_TARGET_STDERR) {
        fprintf(stderr, "%s%s\n", prefix, message);
        fflush(stderr);
    }
    
    if ((g_log.config.targets & LOG_TARGET_FILE) && g_log.log_file) {
        fprintf(g_log.log_file, "%s%s\n", prefix, message);
    }
    
    if (g_log.config.targets & LOG_TARGET_SYSLOG) {
        syslog(syslog_priority[level], "%s", message);
    }
    
    if (g_log.initialized) {
        pthread_mutex_unlock(&g_log.mutex);
    }
    
    /* Exit on fatal */
    if (level == XVC_LOG_FATAL) {
        exit(1);
    }
}

const char* log_level_name(log_level_t level)
{
    if (level >= 0 && level < (int)(sizeof(level_names) / sizeof(level_names[0]))) {
        return level_names[level];
    }
    return "UNKNOWN";
}

log_level_t log_level_from_string(const char *str)
{
    if (!str) return XVC_LOG_INFO;
    
    if (strcasecmp(str, "debug") == 0) return XVC_LOG_DEBUG;
    if (strcasecmp(str, "info") == 0) return XVC_LOG_INFO;
    if (strcasecmp(str, "warn") == 0 || strcasecmp(str, "warning") == 0) return XVC_LOG_WARN;
    if (strcasecmp(str, "error") == 0) return XVC_LOG_ERROR;
    if (strcasecmp(str, "fatal") == 0) return XVC_LOG_FATAL;
    
    return XVC_LOG_INFO;
}
