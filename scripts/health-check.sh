#!/bin/bash
#
# health-check.sh - Health check script for XVC Server instances
#
# Usage:
#   health-check.sh [instance_id]
#
# Examples:
#   health-check.sh         # Check all instances
#   health-check.sh 1       # Check instance 1 only
#

set -e

BASE_PORT=2542
MAX_PORT=2573

# Check single port
check_port() {
    local port=$1
    local instance=$2
    
    if nc -z -w2 localhost "$port" 2>/dev/null; then
        echo "[OK] Instance $instance: Port $port is listening"
        return 0
    else
        echo "[FAIL] Instance $instance: Port $port is not responding"
        return 1
    fi
}

# Check all instances
check_all() {
    local failed=0
    
    echo "XVC Server Health Check"
    echo "========================"
    
    # Find which ports are in use
    for port in $(seq $BASE_PORT $MAX_PORT); do
        instance=$((port - BASE_PORT + 1))
        if ss -tlnp 2>/dev/null | grep -q ":$port "; then
            if ! check_port "$port" "$instance"; then
                failed=$((failed + 1))
            fi
        fi
    done
    
    if [ $failed -eq 0 ]; then
        echo ""
        echo "All instances healthy"
        return 0
    else
        echo ""
        echo "$failed instance(s) failed health check"
        return 1
    fi
}

# Check specific instance
check_instance() {
    local instance=$1
    local port=$((BASE_PORT + instance - 1))
    
    echo "Checking instance $instance (port $port)..."
    check_port "$port" "$instance"
}

# Main
if [ -n "${1:-}" ]; then
    check_instance "$1"
else
    check_all
fi
