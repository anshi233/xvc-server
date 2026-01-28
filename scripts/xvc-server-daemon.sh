#!/bin/bash
#
# xvc-server-daemon.sh - Daemon management script for XVC Server
#
# Usage:
#   xvc-server-daemon.sh {start|stop|restart|status|reload}
#

set -e

NAME="xvc-server"
DAEMON="/usr/local/bin/xvc-server"
CONFIG="/etc/xvc-server/xvc-server-multi.conf"
PIDFILE="/var/run/${NAME}.pid"
LOGFILE="/var/log/${NAME}.log"

# Check if running as root
check_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Error: This script must be run as root" >&2
        exit 1
    fi
}

# Check if daemon binary exists
check_binary() {
    if [ ! -x "$DAEMON" ]; then
        echo "Error: $DAEMON not found or not executable" >&2
        exit 1
    fi
}

# Check if config file exists
check_config() {
    if [ ! -f "$CONFIG" ]; then
        echo "Error: Configuration file $CONFIG not found" >&2
        echo "Run xvc-discover to generate configuration:" >&2
        echo "  sudo xvc-discover --output $CONFIG" >&2
        exit 1
    fi
}

# Get PID if running
get_pid() {
    if [ -f "$PIDFILE" ]; then
        cat "$PIDFILE"
    fi
}

# Check if running
is_running() {
    local pid
    pid=$(get_pid)
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        return 0
    fi
    return 1
}

# Start daemon
do_start() {
    check_root
    check_binary
    check_config

    if is_running; then
        echo "${NAME} is already running (PID $(get_pid))"
        return 0
    fi

    echo "Starting ${NAME}..."
    
    # Start daemon
    $DAEMON -d "$CONFIG"
    
    # Wait for daemon to start
    sleep 1
    
    if is_running; then
        echo "${NAME} started (PID $(get_pid))"
        return 0
    else
        echo "Failed to start ${NAME}" >&2
        return 1
    fi
}

# Stop daemon
do_stop() {
    check_root

    if ! is_running; then
        echo "${NAME} is not running"
        rm -f "$PIDFILE"
        return 0
    fi

    local pid
    pid=$(get_pid)
    
    echo "Stopping ${NAME} (PID $pid)..."
    kill -TERM "$pid"
    
    # Wait for graceful shutdown
    local count=0
    while is_running && [ $count -lt 30 ]; do
        sleep 1
        count=$((count + 1))
    done
    
    if is_running; then
        echo "Force killing ${NAME}..."
        kill -KILL "$pid" 2>/dev/null || true
        sleep 1
    fi
    
    rm -f "$PIDFILE"
    echo "${NAME} stopped"
}

# Restart daemon
do_restart() {
    do_stop
    sleep 2
    do_start
}

# Reload configuration
do_reload() {
    check_root

    if ! is_running; then
        echo "${NAME} is not running"
        return 1
    fi

    local pid
    pid=$(get_pid)
    
    echo "Reloading ${NAME} configuration..."
    kill -HUP "$pid"
    echo "Reload signal sent to PID $pid"
}

# Show status
do_status() {
    if is_running; then
        local pid
        pid=$(get_pid)
        echo "${NAME} is running (PID $pid)"
        
        # Show instance info if available
        if command -v pgrep >/dev/null; then
            local children
            children=$(pgrep -P "$pid" 2>/dev/null | wc -l)
            echo "  Active instances: $children"
        fi
        
        # Show ports in use
        if command -v ss >/dev/null; then
            echo "  Listening on:"
            ss -tlnp 2>/dev/null | grep xvc-server | awk '{print "    " $4}' || true
        fi
        
        return 0
    else
        echo "${NAME} is not running"
        return 1
    fi
}

# Main
case "${1:-}" in
    start)
        do_start
        ;;
    stop)
        do_stop
        ;;
    restart)
        do_restart
        ;;
    reload|force-reload)
        do_reload
        ;;
    status)
        do_status
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|reload|status}" >&2
        exit 1
        ;;
esac

exit 0
