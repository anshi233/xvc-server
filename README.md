# XVC Server - Multi-Instance Digilent HS2 Support

## Overview
Production-grade Xilinx Virtual Cable (XVC) server with multi-instance architecture. Each Digilent HS2 JTAG adapter gets its own dedicated XVC server instance on a unique TCP port, enabling concurrent access with process isolation, persistent configuration, IP whitelisting, and 24/7 enterprise-grade stability.

## Two-Binary Architecture

The system consists of two separate programs:

1. **xvc-discover**: Discovery tool that scans for connected HS2 devices and generates configuration templates
2. **xvc-server**: XVC server that reads configuration and runs multi-instance XVC service

## Key Features
- **Two-Binary Architecture**: Discovery tool generates config, server reads and runs config
- **Multi-Instance Architecture**: Each HS2 device gets its own XVC server instance
- **Dedicated TCP Ports**: Each instance listens on a unique port (2542 + instance_id - 1)
- **Process Isolation**: Faults in one instance don't affect others
- **Support up to 32 simultaneous instances**
- Device identification via USB serial, port location, or custom ID
- Per-instance IP whitelisting (strict/permissive/off modes)
- ARM64 Linux compatible with cross-compilation support
- 24/7 stability with health monitoring and auto-recovery
- Graceful shutdown and configuration hot-reloading
- Systemd service integration
- Comprehensive logging and statistics

## Features

- Multi-HS2 device support (up to 32 simultaneous devices)
- Device identification via USB serial, port location, or custom ID
- Persistent device mapping and tracking
- Source IP whitelisting (strict/permissive/off modes)
- ARM64 Linux compatible with cross-compilation support
- 24/7 stability with health monitoring and auto-recovery
- Graceful shutdown and configuration hot-reloading
- Systemd service integration
- Comprehensive logging and statistics

## Requirements

### Hardware
- Digilent HS2 JTAG debugger (FT2232H-based)
- ARM64 Linux system (e.g., Raspberry Pi, Jetson, ARM server)
- USB 2.0 ports

### Software Dependencies
- Linux kernel with USB support
- pthread library
- systemd (for service management)
- **Build tools** (for building vendor libraries):
  - gcc (or cross-compiler for ARM64)
  - make
  - cmake
  - build-essential

### External Libraries
**All external libraries are bundled locally** in `vendor/` directory. No system library installations required.

**Vendor Libraries:**
- `libftdi1` (version 1.5): FTDI library for HS2 communication
- `libusb` (version 1.0.26): USB library for device discovery

**System Dependencies Only:**
- C standard library (libc, libm, libpthread)
- Standard system headers

**Benefits of Local Libraries:**
- Reproducible builds across all platforms
- Version-controlled library versions
- No system-wide installations required
- Works consistently for cross-compilation

## Installation

### From Source
```bash
# Clone repository
git clone <repository-url>
cd xvc-server

# Build vendor libraries (required first time)
make vendor-lib

# Build both binaries (vendor libs + binaries)
make

# Or build individually
make xvc-discover
make xvc-server

# Install both binaries
sudo make install

# Install individually
sudo make install-discover
sudo make install-server

# Enable systemd service
sudo systemctl enable xvc-server
```

### First-Time Setup

Vendor libraries need to be built before compiling binaries:

```bash
# Step 1: Build vendor libraries
make vendor-lib

# Step 2: Build binaries
make

# Or do both in one command (make automatically builds vendor libs first)
make all
```

### Cross-Compilation for ARM64

```bash
# Build vendor libraries for ARM64
make vendor-lib ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

# Build binaries for ARM64
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

# Or combine
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
```

### Pre-built Packages (Future)
```bash
# Debian/Ubuntu
sudo dpkg -i xvc-server_1.0.0_arm64.deb

# RPM-based
sudo rpm -ivh xvc-server-1.0.0-1.aarch64.rpm
```

## Quick Start

### Step 1: Discover Devices
Run the discovery tool to scan for connected HS2 devices and generate configuration:

```bash
# Basic discovery (output to stdout)
sudo xvc-discover

# Generate configuration file
sudo xvc-discover --output /etc/xvc-server/xvc-server-multi.conf

# With verbose output
sudo xvc-discover --verbose --output /etc/xvc-server/xvc-server-multi.conf

# JSON format
sudo xvc-discover --format json --output devices.json
```

**Example Output:**
```
Detected 3 Digilent HS2 devices:

Device #1:
  Manufacturer: Digilent
  Product: JTAG-HS2
  Serial Number: ABC12345
  USB Bus: 001, Device: 002
  Suggested Instance ID: 1
  Suggested Port: 2542

Device #2:
  Manufacturer: Digilent
  Product: JTAG-HS2
  Serial Number: DEF67890
  USB Bus: 001, Device: 003
  Suggested Instance ID: 2
  Suggested Port: 2543

Device #3:
  Manufacturer: Digilent
  Product: JTAG-HS2
  Serial Number: GHI13579
  USB Bus: 002, Device: 001
  Suggested Instance ID: 3
  Suggested Port: 2544

Configuration written to: /etc/xvc-server/xvc-server-multi.conf
Edit the configuration file to customize settings, then run:
  sudo xvc-server /etc/xvc-server/xvc-server-multi.conf
```

### Step 2: Customize Configuration (Optional)
Edit the generated configuration file:

```bash
sudo nano /etc/xvc-server/xvc-server-multi.conf
```

Customize:
- Port assignments
- Instance settings (frequency, latency, etc.)
- Instance aliases
- IP whitelist settings
- Performance tuning

### Step 3: Start Server
Start the server with the configuration file:

```bash
# Start manually
sudo xvc-server /etc/xvc-server/xvc-server-multi.conf

# Or use systemd
sudo systemctl start xvc-server

# Check status
sudo systemctl status xvc-server

# View logs
sudo journalctl -u xvc-server -f
```

## Configuration

### xvc-discover Tool

The discovery tool scans for connected Digilent HS2 devices and generates configuration templates.

**Command Line Options:**
```
Usage: xvc-discover [OPTIONS]

Options:
  -o, --output FILE      Output configuration file (default: stdout)
  -f, --format FORMAT    Output format: ini, json, yaml (default: ini)
  -b, --base-port PORT   Base port for instance numbering (default: 2542)
  -v, --verbose          Enable verbose output
  -q, --quiet            Suppress normal output
  -h, --help             Display this help message
  -V, --version          Display version information
```

**Usage Examples:**

Basic discovery (output to stdout):
```bash
sudo xvc-discover
```

Generate configuration file:
```bash
sudo xvc-discover --output /etc/xvc-server/xvc-server-multi.conf
```

JSON format output:
```bash
sudo xvc-discover --format json --output devices.json
```

Detailed information:
```bash
sudo xvc-discover --verbose
```

Custom base port:
```bash
sudo xvc-discover --base-port 3000 --output /etc/xvc-server/xvc-server-multi.conf
```

**Discovery Workflow:**

1. **Scan USB Bus:**
   - Use libusb to scan all USB devices
   - Filter for FTDI devices with VID:0x0403, PID:0x6010
   - Identify Digilent HS2 devices

2. **Gather Device Information:**
   - Read USB descriptor (manufacturer, product, serial)
   - Get USB bus/device location
   - Test device accessibility

3. **Assign Instance IDs:**
   - Assign sequential instance IDs starting from 1
   - Calculate TCP port: base_port + instance_id - 1
   - Generate default aliases

4. **Generate Configuration:**
   - Create INI file with detected devices
   - Add default settings for each instance
   - Write to output file or stdout

5. **Output Summary:**
   - Display detected devices
   - Show instance mappings
   - Provide next steps

### Multi-Instance Configuration
Location: `/etc/xvc-server/xvc-server-multi.conf`

**Note:** This file is typically generated by `xvc-discover`. Edit as needed.

```ini
[instance_management]
enabled = true
base_port = 2542
max_instances = 32

[instance_mappings]
# Map instance IDs to specific HS2 devices
# Format: instance_id = device_id
# device_id can be: SN:serial, BUS:bus-dev, CUSTOM:name, or auto

1 = SN:ABC12345     # Instance 1 on port 2542 handles HS2 with serial ABC12345
2 = BUS:001-002     # Instance 2 on port 2543 handles HS2 on bus 001 device 002
3 = CUSTOM:PROD01   # Instance 3 on port 2544 handles custom device PROD01
4 = auto            # Instance 4 auto-detects device
5 = auto            # Instance 5 auto-detects device

[instance_settings]
# Per-instance settings
1:frequency = 10000000      # TCK frequency for instance 1
2:latency_timer = 2         # FTDI latency for instance 2
3:async = false             # Disable async for instance 3

[instance_aliases]
1 = Production Board A
2 = Production Board B
3 = Development Board
4 = Test Board A
5 = Test Board B
```

### Port Allocation

```ini
# Automatic port calculation
base_port = 2542
instance_id = 1..32
instance_port = base_port + instance_id - 1

Examples:
  Instance 1: Port 2542 (2542 + 1 - 1 = 2542)
  Instance 2: Port 2543 (2542 + 2 - 1 = 2543)
  Instance 3: Port 2544 (2542 + 3 - 1 = 2544)
  Instance 32: Port 2573 (2542 + 32 - 1 = 2573)
```

### Per-Instance IP Whitelist

```ini
[ip_whitelist_per_instance]
# Per-instance IP whitelist

# Format: instance_id:mode = mode_name
# mode: strict, permissive, off

# Example: Instance 1 uses strict whitelist
1:mode = strict

# Example: Instance 2 uses permissive whitelist
2:mode = permissive

# Example: Instance 3 allows all IPs
3:mode = off

# Allowlist per instance (format: instance_id:allow_N = IP/CIDR)
1:allow_1 = 192.168.1.0/24
1:allow_2 = 10.0.0.0/8

# Blocklist per instance (format: instance_id:block_N = IP/CIDR)
1:block_1 = 192.168.1.250
2:block_1 = 0.0.0.0/8
```

## Usage

### Complete Workflow

**1. Discover Devices:**
```bash
sudo xvc-discover --output /etc/xvc-server/xvc-server-multi.conf
```

**2. Edit Configuration (Optional):**
```bash
sudo nano /etc/xvc-server/xvc-server-multi.conf
```

**3. Start Server:**
```bash
# Manual start
sudo xvc-server /etc/xvc-server/xvc-server-multi.conf

# Or use systemd
sudo systemctl start xvc-server
```

**4. Verify Status:**
```bash
sudo systemctl status xvc-server
sudo journalctl -u xvc-server -f
```

### Starting Server

**Manual Start:**
```bash
sudo xvc-server /etc/xvc-server/xvc-server-multi.conf
```

**Systemd Service:**
```bash
# Enable service (start on boot)
sudo systemctl enable xvc-server

# Start service
sudo systemctl start xvc-server

# Stop service
sudo systemctl stop xvc-server

# Restart service
sudo systemctl restart xvc-server

# Check status
sudo systemctl status xvc-server

# View logs
sudo journalctl -u xvc-server -f
```

### Connecting from Vivado

Each instance has its own port, so Vivado connects to the specific instance for the target device:

**For Instance 1 (Port 2542):**
1. Open Vivado Hardware Manager
2. Click "Add Xilinx Virtual Cable"
3. Enter server details:
   - Host: `<server-ip>`
   - Port: `2542`
4. Click "OK" to connect

**For Instance 2 (Port 2543):**
1. Open Vivado Hardware Manager
2. Click "Add Xilinx Virtual Cable"
3. Enter server details:
   - Host: `<server-ip>`
   - Port: `2543`
4. Click "OK" to connect

**For Instance N (Port 2542+N-1):**
1. Open Vivado Hardware Manager
2. Click "Add Xilinx Virtual Cable"
3. Enter server details:
   - Host: `<server-ip>`
   - Port: `2542+N-1`
4. Click "OK" to connect

### Device Discovery

When multiple HS2 devices are connected:
- Server automatically discovers devices on startup
- Devices are identified using configured method (serial/port/custom)
- Persistent mappings are loaded from configuration
- Clients are automatically assigned to available devices

### Configuration Reload
```bash
# Reload without restart
sudo systemctl reload xvc-server

# Or send signal
sudo kill -HUP $(cat /var/run/xvc-server.pid)
```

### Force Device Discovery
```bash
# Trigger device re-discovery
sudo kill -USR1 $(cat /var/run/xvc-server.pid)
```

## Device Identification Methods

### 1. USB Serial Number (Recommended)
Most reliable method. Each FTDI chip has a unique serial number programmed at factory.

**Pros:**
- Unique and persistent
- Survives USB reconnection
- Survives system reboot
- Manufacturer-assigned

**Cons:**
- Requires reading FTDI EEPROM
- Cannot be changed without hardware programming

### 2. USB Port Location (Bus:Device)
Uses physical USB bus and device address for identification.

**Pros:**
- Always consistent for same USB port
- Easy to identify physically
- No programming required

**Cons:**
- Changes if USB topology changes
- May change after reboot on some systems

### 3. Custom Device ID
User-defined identifier for manual assignment.

**Pros:**
- Fully controlled
- Flexible naming
- Can use any naming scheme

**Cons:**
- Requires initial setup
- Manual configuration

## IP Whitelisting

### Strict Mode
Only allowlisted IPs can connect. All others are blocked.

```ini
[ip_whitelist]
enabled = true
mode = strict
allow_1 = 192.168.1.0/24
```

### Permissive Mode
Allowlisted IPs are allowed. Others are logged but not blocked.

```ini
[ip_whitelist]
enabled = true
mode = permissive
allow_1 = 192.168.1.0/24
```

### Off Mode
All IPs allowed (not recommended for production).

```ini
[ip_whitelist]
enabled = false
mode = off
```

## Troubleshooting

### Device Not Detected
```bash
# Check FTDI devices
lsusb -d 0403:6010

# Check server logs
sudo journalctl -u xvc-server -n 50

# Force device discovery
sudo kill -USR1 $(cat /var/run/xvc-server.pid)
```

### Connection Refused
```bash
# Check if server is running
sudo systemctl status xvc-server

# Check firewall
sudo ufw status
sudo iptables -L -n

# Check IP whitelist
grep "ip_whitelist" /etc/xvc-server/xvc-server.conf
```

### Device Connection Issues
```bash
# Check device permissions
sudo ls -l /dev/bus/usb/

# Check FTDI library
ldd /usr/local/bin/xvc-server | grep ftdi

# Test FTDI device directly
sudo ftdi_eeprom --read --flash-size
```

## Performance Optimization

### JTAG Speed
The JTAG speed is controlled by Vivado via the `settck:` command. Server responds with actual period used.

**Recommended settings for different FTDI devices:**
- FT2232H (Digilent HS2): Up to 12 Mbps
- FT4232H: Up to 12 Mbps
- FT232H: Up to 6 Mbps

### Network Optimization
```ini
[network]
tcp_nodelay = true
tcp_keepalive = true
keepalive_idle = 30
keepalive_cnt = 5
```

## Security

### Best Practices
1. Always use strict IP whitelist in production
2. Use firewall rules in addition to whitelist
3. Monitor logs for suspicious activity
4. Use VPN or secure tunnel for remote access
5. Regularly update device mappings
6. Use strong authentication for remote management

### Firewall Configuration
```bash
# Allow only XVC port
sudo ufw allow 2542/tcp

# Block all other incoming (after establishing SSH)
sudo ufw default deny incoming
sudo ufw allow from 192.168.1.0/24
```

## Architecture

### Component Diagram
```
Vivado Client
       ↓
TCP/IP Connection (Port 2542)
       ↓
┌─────────────────────┐
│  XVC Protocol     │
│  Handler          │
└─────────────────────┘
       ↓
┌─────────────────────┐
│  Device Manager    │
│  - Multi-HS2      │
│  - Mapping         │
│  - Tracking        │
└─────────────────────┘
       ↓
┌─────────────────────┐
│  FTDI Adapter    │
│  - libftdi1       │
│  - HS2 Control    │
└─────────────────────┘
       ↓
┌─────────────────────┐
│  Digilent HS2 #1 │
│  Digilent HS2 #2 │
│  Digilent HS2 #3 │
└─────────────────────┘
```

## Monitoring

### Health Check
```bash
# Check service health
./scripts/health-check.sh

# Expected output:
✓ Service is running
✓ Devices: 3 active
✓ Connections: 2 active
✓ Uptime: 5 days, 3 hours
```

### Statistics
The server maintains statistics:
- Total connections handled
- Failed connections
- Bytes transferred
- Active connections
- Device errors
- Uptime

View statistics via logs or management API (future feature).

## Support

### Logs
```bash
# Live logs
sudo journalctl -u xvc-server -f

# Historical logs
sudo less /var/log/xvc-server.log
sudo zcat /var/log/xvc-server.log.1.gz
```

### Issue Reporting
When reporting issues, include:
1. Server version: `xvc-server --version`
2. System architecture: `uname -m`
3. Linux kernel: `uname -r`
4. libftdi version: `dpkg -l | grep libftdi1`
5. Configuration files: `/etc/xvc-server/*`
6. Relevant log excerpts
7. Steps to reproduce

## License

This project is licensed under [LICENSE NAME] - [LICENSE DESCRIPTION]

## Credits

Based on xvcd by tmbinc (CC0 1.0 Public Domain)
Xilinx Virtual Cable protocol documentation by Xilinx Inc.

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Changelog

### Version 1.0.0 (Planned)
- Initial release
- Multi-HS2 device support
- IP whitelisting
- Persistent configuration
- 24/7 stability features
- ARM64 support
- Systemd integration
