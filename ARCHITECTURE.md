# XVC Server Architecture - Multi-Instance Design

## Overview
Each HS2 device gets its own dedicated XVC server instance running on a unique TCP port, enabling concurrent access without device conflicts.

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────────┐
│                         Host System (ARM64 Linux)                │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │              Instance Manager (Main Process)                   │    │
│  │  - Spawns/monitors XVC server instances                │    │
│  │  - Load instance configuration                                     │    │
│  │  - Health monitoring across all instances                        │    │
│  │  - Log aggregation                                               │    │
│  │  - Signal distribution                                           │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                          ↓                                          │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐   │
│  │   Instance 1  │   Instance 2  │   Instance 3  │   Instance N  │   │
│  │  Port:2542  │  Port:2543  │  Port:2544  │  Port:2542+N│   │
│  └──────┬──────┘   └──────┬──────┘   └──────┬──────┘   │
│         ↓                   ↓                   ↓              ↓            │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐            │
│  │ XVC Protocol  │   │ XVC Protocol  │   │ XVC Protocol  │            │
│  │   Handler       │   │   Handler       │   │   Handler       │            │
│  └────────┬────────┘   └────────┬────────┘   └────────┬────────┘            │
│         ↓                      ↓                      ↓                           │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐            │
│  │  Device Mgr    │   │  Device Mgr    │   │  Device Mgr    │            │
│  └────────┬────────┘   └────────┬────────┘   └────────┬────────┘            │
│         ↓                      ↓                      ↓                           │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐            │
│  │  FTDI Adapter  │   │  FTDI Adapter  │   │  FTDI Adapter  │            │
│  └────────┬────────┘   └────────┬────────┘   └────────┬────────┘            │
│         ↓                      ↓                      ↓                           │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐            │
│  │  Digilent HS2 #1│   │  Digilent HS2 #2│   │  Digilent HS2 #3│            │
│  │  (SN:ABC12345) │   │  (SN:DEF67890) │   │  (BUS:001-002)│            │
│  └─────────────────┘   └─────────────────┘   └─────────────────┘            │
└───────────────────────────────────────────────────────────────────────────┘
                          ↓
                ┌─────────────────────────────────────┐
                │         Vivado Clients              │
                │  - Client 1 → Instance 1 (Port 2542)│
                │  - Client 2 → Instance 2 (Port 2543)│
                │  - Client 3 → Instance 3 (Port 2544)│
                │  - Client N → Instance N (Port 2542+N)│
                └─────────────────────────────────────┘
```

## Instance Management

### Instance Lifecycle

```
Startup:
  1. Load instance configuration
  2. Spawn XVC server instance
  3. Assign device to instance
  4. Monitor instance health
  5. Log to instance-specific file

Runtime:
  1. Handle client connections
  2. Process XVC commands
  3. Communicate with HS2 device
  4. Update health statistics
  5. Rotate logs

Shutdown:
  1. Accept no new connections
  2. Drain existing connections
  3. Close FTDI device
  4. Save instance state
  5. Cleanup resources

Restart:
  1. Graceful shutdown
  2. Reload configuration
  3. Restart instance
  4. Reconnect to device
```

### Instance Configuration

Each instance has its own configuration section:

```ini
[instance_1]
port = 2542
device = SN:ABC12345
listen_ip = 0.0.0.0
max_connections = 4

[instance_2]
port = 2543
device = BUS:001-002
listen_ip = 0.0.0.0
max_connections = 4

[instance_3]
port = 2544
device = CUSTOM:PROD01
listen_ip = 0.0.0.0
max_connections = 4
```

## Port Allocation

### Automatic Port Calculation

```
base_port = 2542
instance_id = 1..32
instance_port = base_port + instance_id - 1

Examples:
  Instance 1: Port 2542 (2542 + 1 - 1 = 2542)
  Instance 2: Port 2543 (2542 + 2 - 1 = 2543)
  Instance 3: Port 2544 (2542 + 3 - 1 = 2544)
  Instance 32: Port 2573 (2542 + 32 - 1 = 2573)
```

### Manual Port Assignment

Override automatic port calculation:

```ini
[instance_management]
base_port = 2542
auto_port = false

[instance_1]
port = 3000  # Manual assignment

[instance_2]
port = 3001  # Manual assignment
```

## Device Assignment

### Static Assignment

Device assigned to specific instance:

```ini
[instance_mappings]
1 = SN:ABC12345
2 = BUS:001-002
3 = CUSTOM:PROD01
```

### Dynamic Assignment

Instance auto-detects available device:

```ini
[instance_mappings]
1 = auto
2 = auto
3 = auto
4 = auto
```

### Priority-Based Assignment

Devices assigned based on priority rules:

```ini
[device_priorities]
SN:ABC12345 = 1
SN:DEF67890 = 2
BUS:001-002 = 3

[instance_mappings]
1 = auto  # Uses highest priority device
2 = auto  # Uses second highest
3 = auto  # Uses third highest
```

## Network Configuration

### Per-Instance Network Settings

Each instance has independent network configuration:

```ini
[network_per_instance]
1:listen_ip = 0.0.0.0
1:max_connections = 4
1:tcp_nodelay = true
1:tcp_keepalive = true

2:listen_ip = 0.0.0.0
2:max_connections = 4
2:tcp_nodelay = true
2:tcp_keepalive = true
```

### IP Whitelist Per Instance

```ini
[ip_whitelist_per_instance]
1:mode = strict
1:allow_1 = 192.168.1.0/24

2:mode = permissive
2:allow_1 = 10.0.0.0/8

3:mode = off
# No restrictions
```

## Resource Management

### Memory Allocation

```
Per Instance:
  - Stack: 8MB
  - Heap: 64MB
  - Buffers: 32MB
  - Total: ~104MB per instance

Total for 32 instances:
  - ~3.3GB
  - Plus overhead: ~500MB
  - Total: ~3.8GB
```

### File Descriptors

```
Per Instance:
  - TCP listen socket: 1
  - Client connections: 4
  - FTDI device: 1
  - Log file: 1
  - Total: 7 FDs per instance

Total for 32 instances:
  - 224 FDs
  - Plus system overhead: 16
  - Total: 240 FDs

System limits:
  ulimit -n 4096  # Recommended
```

### Thread Allocation

```
Per Instance:
  - Main thread: 1
  - Client handler threads: 4
  - Device I/O thread: 1
  - Health monitor thread: 1
  - Total: 7 threads per instance

Total for 32 instances:
  - 224 threads
  - Plus manager threads: 8
  - Total: 232 threads
```

## Communication Flow

### Client Connection Flow

```
1. Client connects to instance_port
2. Instance accepts connection
3. IP whitelist check (if enabled)
4. Client sends 'getinfo:' command
5. Instance responds with version info
6. Client sends 'settck:' command
7. Instance configures FTDI device
8. Client sends 'shift:' commands
9. Instance processes XVC commands
10. Instance responds with TDO data
11. Repeat steps 8-10
12. Client disconnects
```

### Device Assignment Flow

```
1. Instance starts
2. Load device mapping from config
3. If static assignment:
   a. Open specific FTDI device
4. If dynamic assignment:
   a. Scan for available HS2 devices
   b. Select based on priority/rules
   c. Open selected device
5. Initialize device (latency timer, baudrate, etc.)
6. Monitor device health
7. On timeout/error:
   a. Log incident
   b. Attempt reconnection
   c. If max attempts: mark offline
8. On device reconnect:
   a. Resume operation
   b. Log recovery
```

## Health Monitoring

### Instance Health Checks

```
Per-Instance Metrics:
  - Connection count
  - Bytes transferred
  - JTAG operations count
  - Device errors
  - Uptime
  - Last client disconnect time

Health States:
  - HEALTHY: All systems operational
  - DEGRADED: Device issues, accepting connections
  - OFFLINE: Device not responding
  - ERROR: Critical failure

Health Actions:
  - DEGRADED: Attempt device recovery
  - OFFLINE: Restart instance after delay
  - ERROR: Send alert and stop instance
```

### Aggregate Health

Manager monitors all instances:

```
Health Dashboard:
  - Total instances: 32
  - Healthy: 28
  - Degraded: 3
  - Offline: 1
  - Total connections: 112
  - Total bandwidth: ~1.2 Gbps

Alerts:
  - Multiple instances degraded
  - High error rate
  - Resource exhaustion
  - Service availability drops below 99%
```

## Signal Handling

### Instance-Level Signals

```
SIGHUP:  Reload configuration
SIGTERM: Graceful shutdown
SIGINT:  Interrupted shutdown
SIGUSR1: Force device rediscovery
SIGUSR2: Trigger log rotation
```

### Manager-Level Signals

```
SIGHUP: Reload all instance configurations
SIGTERM: Shutdown all instances gracefully
SIGINT: Shutdown all instances immediately
SIGUSR1: Restart specific instance
SIGUSR2: Restart all instances
```

## Log Management

### Instance Logs

```
Location: /var/log/xvc-server-instance-{id}.log

Format:
  [TIMESTAMP] [LEVEL] [INSTANCE-{id}] message

Examples:
  [2024-01-26 10:15:23] [INFO] [INSTANCE-1] Client connected from 192.168.1.100
  [2024-01-26 10:15:24] [DEBUG] [INSTANCE-1] Received: getinfo:
  [2024-01-26 10:15:25] [DEBUG] [INSTANCE-1] Sent: xvcServer_v1.0:2048
  [2024-01-26 10:15:26] [WARN] [INSTANCE-1] Device timeout, attempting reconnection
```

### Aggregate Logs

```
Location: /var/log/xvc-server-manager.log

Format:
  [TIMESTAMP] [LEVEL] [MANAGER] message

Examples:
  [2024-01-26 10:00:00] [INFO] [MANAGER] Starting 32 instances
  [2024-01-26 10:00:05] [INFO] [MANAGER] Instance 1 started (port 2542)
  [2024-01-26 10:00:10] [INFO] [MANAGER] Instance 2 started (port 2543)
  [2024-01-26 10:00:15] [WARN] [MANAGER] Instance 3 failed to start: device not found
```

## Performance Considerations

### Concurrent Instance Load

```
Optimal: 8-16 active instances
Maximum: 32 instances (with sufficient resources)

Resource Scaling:
  CPU: Linear with active instances
  Memory: Linear with active instances
  Network: Linear with active instances
  I/O: Limited by USB bandwidth

Recommendations:
  - Monitor instance health
  - Set appropriate max_connections per instance
  - Use IP whitelisting to limit access
  - Implement connection pooling
```

### USB Bandwidth Management

```
Total USB 2.0 Bandwidth: 480 Mbps
Per HS2 Device: ~12 Mbps (maximum)
Maximum Concurrent HS2: ~40 devices
Recommended Concurrent: 16-24 devices
With XVC Overhead: 8-16 devices

Bandwidth Distribution:
  - Each HS2: ~12 Mbps
  - USB protocol overhead: ~20%
  - Actual data: ~10 Mbps per device
  - Total with 16 devices: ~160 Mbps
```

## Failure Recovery

### Instance Failure

```
Detection:
  - Health check timeout
  - Process crash
  - Device disconnection

Recovery:
  1. Log failure
  2. Mark instance as offline
  3. Notify manager
  4. Attempt restart (if auto-restart enabled)
  5. Limit restart attempts
  6. Alert if max attempts exceeded

Backoff Strategy:
  - Attempt 1: Immediate
  - Attempt 2: Wait 5 seconds
  - Attempt 3: Wait 30 seconds
  - Attempt 4+: Wait 60 seconds
```

### Manager Failure

```
Detection:
  - Manager process crash
  - System resource exhaustion

Recovery:
  1. Systemd restarts manager
  2. Manager reloads configuration
  3. Manager spawns all instances
  4. Each instance reconnects to device

Failover:
  - If manager cannot start: Systemd retries
  - Max retries: 5
  - Retry delay: Exponential backoff
  - Critical alert after max retries
```

## Security Considerations

### Instance Isolation

```
Process Isolation:
  - Separate process per instance
  - No shared memory between instances
  - Independent device access
  - Independent log files

Network Isolation:
  - Separate ports per instance
  - Independent IP whitelist per instance
  - Separate connection limits
  - Independent TCP settings
```

### Access Control

```
Per-Instance Access:
  - IP whitelist (strict/permissive/off)
  - Max connections limit
  - Connection timeout
  - Client authentication (future)

Manager Access:
  - Only root can control instances
  - Socket-based management API (future)
  - Protected configuration files
  - Secure log rotation
```

## Deployment

### Systemd Integration

```ini
# Main manager service
[Unit]
Description=XVC Server Manager
After=network.target

[Service]
Type=forking
PIDFile=/var/run/xvc-server-manager.pid
ExecStart=/usr/local/bin/xvc-server-manager
ExecStop=/usr/local/bin/xvc-server-manager stop
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Instance Services

```bash
# Instances managed by manager, not systemd
# Manager spawns instances as child processes
# Systemd only manages the main manager process
```

## Monitoring

### Health Dashboard (Future)

```
Web Interface: http://localhost:8080
Features:
  - Instance status overview
  - Real-time connection graphs
  - Device health monitoring
  - Log viewing
  - Configuration management
  - Instance control (start/stop/restart)
```

### Statistics API (Future)

```
REST API: http://localhost:8081/api
Endpoints:
  GET /api/instances - List all instances
  GET /api/instances/{id} - Get instance details
  GET /api/stats - Get aggregate statistics
  GET /api/health - Get health status
  POST /api/instances/{id}/restart - Restart instance
```

## Migration Path

### Single to Multi-Instance

```
1. Backup existing configuration
2. Install multi-instance version
3. Import device mappings
4. Configure instance settings
5. Start manager service
6. Verify all instances
7. Update Vivado connections (new ports)
```

### Device Migration

```
When adding new HS2 device:
  1. Identify device (lsusb)
  2. Add to device mapping
  3. Assign to instance ID
  4. Configure instance (if needed)
  5. Reload manager: systemctl reload xvc-server
  6. Instance auto-connects to new device
```
