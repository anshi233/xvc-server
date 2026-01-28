# Background Information - XVC Server for Digilent HS2

## What is XVC?

Xilinx Virtual Cable (XVC) is a TCP/IP-based protocol that allows remote debugging and programming of Xilinx FPGAs over a network connection. It implements the same functionality as a local JTAG connection but over Ethernet.

### XVC Protocol Overview

The XVC protocol consists of three main commands:

1. **getinfo:** - Client requests server information
   - Request: `getinfo:`
   - Response: `xvcServer_v1.0:2048`

2. **settck:** - Client sets JTAG clock frequency
   - Request: `settck:<period>`
   - Response: `<period>` (echo back)

3. **shift:** - Client performs JTAG operations
   - Request: `shift:<num_bits>:<tms_bits>:<tdi_bits>:<tms_length>:<tdi_length>`
   - Response: `<tdo_bits>`

### Why Use XVC?

- **Remote Debugging**: Access FPGAs from anywhere on the network
- **Shared Debuggers**: Multiple developers share expensive debug hardware
- **Automation**: Integrate FPGA debugging into CI/CD pipelines
- **Production Access**: Debug production systems without physical access
- **Hardware Savings**: Centralize expensive JTAG debuggers

## What is Digilent HS2?

The Digilent JTAG-HS2 is a high-speed USB-to-JTAG adapter based on the FTDI FT2232H chip.

### Technical Specifications

| Specification | Value |
|---------------|-------|
| USB Interface | USB 2.0 High Speed (480 Mbps) |
| FTDI Chip | FT2232H |
| Max JTAG Speed | ~12 Mbps |
| Vendor ID | 0x0403 (FTDI) |
| Product ID | 0x6010 (FT2232H) |
| JTAG Voltage | 1.2V - 3.3V (auto-detect) |
| USB Cable Length | Up to 3 meters |
| Supported JTAG Modes | TMS/TDI/TDO/TCK, SWD |

### Why HS2 is Suitable

1. **High Speed**: Up to 12 Mbps JTAG for fast bitstream downloads
2. **USB 2.0**: Standard USB interface, works on any Linux system
3. **Open Drivers**: libftdi provides open-source driver support
4. **Affordable**: Reasonably priced compared to enterprise debuggers
5. **Widely Available**: Commonly used in FPGA development
6. **FTDI Chip**: Well-documented, stable drivers

## Why Build a Custom XVC Server?

### Limitations of Existing Solutions

1. **XilinxOfficial XVC Server**:
   - Only runs on Xilinx SoCs (Zynq, Versal)
   - Not suitable for external USB adapters
   - Different use case (on-chip vs external)

2. **Standard xvcd** (tmbinc):
   - Single device support
   - Basic configuration
   - Limited enterprise features
   - No multi-device management

3. **Commercial Solutions**:
   - Expensive licensing
   - Proprietary hardware requirements
   - Vendor lock-in
   - Limited customization

### Our Solution: Multi-Instance XVC Server

Our implementation addresses these limitations with:

1. **Multi-Device Support**: Up to 32 HS2 devices simultaneously
2. **Dedicated Ports**: Each device gets unique TCP port
3. **Process Isolation**: Faults don't affect other instances
4. **Enterprise Features**: IP whitelisting, health monitoring, logging
5. **Production Ready**: 24/7 operation, auto-recovery, systemd integration
6. **Open Source**: CC0 license, fully customizable
7. **ARM64 Support**: Runs on ARM servers, Raspberry Pi, Jetson, etc.

## Use Cases

### Development Environment

```
Team of 10 developers, 5 HS2 debuggers:

Developer 1 → Instance 1 (Port 2542) → HS2 #1
Developer 2 → Instance 2 (Port 2543) → HS2 #2
Developer 3 → Instance 3 (Port 2544) → HS2 #3
Developer 4 → Instance 4 (Port 2545) → HS2 #4
Developer 5 → Instance 5 (Port 2546) → HS2 #5
Developer 6 → Instance 6 (Port 2547) → HS2 #1 (when free)
...

Benefits:
- Share expensive hardware
- Remote debugging from anywhere
- No hardware swapping
- Parallel debugging
```

### Production Debugging

```
Production rack with 20 FPGAs, 5 HS2 debuggers:

FPGA 1-4 → Instance 1-4 (Ports 2542-2545) → HS2 #1 (via JTAG switch)
FPGA 5-8 → Instance 5-8 (Ports 2546-2549) → HS2 #2 (via JTAG switch)
FPGA 9-12 → Instance 9-12 (Ports 2550-2553) → HS2 #3 (via JTAG switch)
FPGA 13-16 → Instance 13-16 (Ports 2554-2557) → HS2 #4 (via JTAG switch)
FPGA 17-20 → Instance 17-20 (Ports 2558-2561) → HS2 #5 (via JTAG switch)

Benefits:
- Remote production debugging
- 24/7 monitoring
- Automated debugging
- Reduced downtime
```

### CI/CD Pipeline

```
Continuous Integration server with 2 HS2 debuggers:

Build #1 → Instance 1 (Port 2542) → HS2 #1 → FPGA Board A
Build #2 → Instance 2 (Port 2543) → HS2 #2 → FPGA Board B
Build #3 → Instance 1 (Port 2542) → HS2 #1 → FPGA Board A (when free)
...

Benefits:
- Automated testing
- Parallel builds
- Bitstream download verification
- Debug waveform capture
- Hardware-in-the-loop testing
```

### Educational Lab

```
University lab with 30 students, 10 HS2 debuggers:

Student 1 → Instance 1 (Port 2542) → HS2 #1
Student 2 → Instance 2 (Port 2543) → HS2 #2
...
Student 10 → Instance 10 (Port 2551) → HS2 #10
Student 11 → Instance 1 (Port 2542) → HS2 #1 (when free)
...

Benefits:
- Share limited hardware
- Remote lab access
- Fair resource allocation
- Reduced hardware costs
- Flexible scheduling
```

## Technical Background

### FTDI FT2232H Architecture

```
FT2232H USB Chip
├── USB 2.0 Interface (480 Mbps)
├── Dual Channel UART/JTAG
│   ├── Channel A: JTAG (HS2 uses this)
│   └── Channel B: UART (optional)
├── Multi-Protocol Synchronous Serial Engine (MPSSE)
│   ├── JTAG mode
│   ├── SPI mode
│   ├── I2C mode
│   └── Bitbang mode
└── EEPROM Configuration
    ├── Serial Number
    ├── Vendor/Product IDs
    └── Custom descriptors
```

### libftdi1 Library

The libftdi1 library provides USB communication with FTDI devices.

**Key Functions:**
- `ftdi_init()`: Initialize FTDI context
- `ftdi_usb_open()`: Open specific FTDI device
- `ftdi_set_bitmode()`: Set operating mode (bitbang, MPSSE, etc.)
- `ftdi_read_data()`: Read data from device
- `ftdi_write_data()`: Write data to device
- `ftdi_usb_close()`: Close device

**Operating Modes:**
- **Bitbang**: Direct GPIO control (slow, flexible)
- **MPSSE**: Multi-Protocol Synchronous Serial (fast, specialized)
- **Sync FIFO**: Synchronous FIFO mode (very fast)

### JTAG Protocol Overview

JTAG (Joint Test Action Group) is a standard for testing and debugging printed circuit boards.

**JTAG TAP Controller States:**
```
Test-Logic-Reset → Run-Test/Idle → Select-DR-Scan → Capture-DR → Shift-DR → Exit1-DR → Pause-DR → Exit2-DR → Update-DR
                   ↺                                                                                     ↓
                   ←──────────────────────────────────────────────────────────────────────────────────────┘
```

**JTAG Signals:**
- **TCK**: Test Clock
- **TMS**: Test Mode Select
- **TDI**: Test Data In
- **TDO**: Test Data Out
- **TRST**: Test Reset (optional)

**JTAG Operations:**
1. **IDCODE Read**: Read device ID
2. **BYPASS**: Shift data through all devices
3. **EXTEST**: Control I/O pins
4. **SAMPLE**: Capture I/O states
5. **INTEST**: Test internal logic

### Network Performance Considerations

**XVC Over TCP/IP:**
- Protocol overhead: ~20 bytes per command
- Typical packet size: 2-4 KB
- Latency: 1-10 ms (LAN), 50-100 ms (WAN)
- Throughput: Limited by JTAG speed, not network

**Optimization Techniques:**
- TCP_NODELAY: Disable Nagle's algorithm
- TCP_KEEPALIVE: Detect dead connections
- Buffer tuning: Match JTAG vector size
- Connection pooling: Reuse connections

## ARM64 Linux Compatibility

### Why ARM64?

1. **Low Power**: ARM64 servers consume less power than x86_64
2. **Cost Effective**: ARM hardware is cheaper
3. **Embedded Friendly**: Same architecture as many embedded systems
4. **Growing Ecosystem**: Increasing software support
5. **Cloud Native**: Many cloud providers offer ARM64 instances

### Popular ARM64 Platforms

| Platform | Use Case | Typical HS2 Capacity |
|----------|----------|---------------------|
| Raspberry Pi 4 | Development/Lab | 4-6 instances |
| NVIDIA Jetson | Edge Computing | 8-12 instances |
| Ampere Altra | Production Server | 32 instances |
| AWS Graviton2 | Cloud Server | 32 instances |
| Rockchip RK3588 | SBC | 6-8 instances |

### ARM64-Specific Considerations

**Advantages:**
- Lower power consumption
- Lower cooling requirements
- Often includes multiple USB controllers
- Can be embedded in target system

**Challenges:**
- USB bandwidth limitations on some platforms
- Different USB controller drivers
- Limited USB ports (may need USB hubs)
- Different kernel versions

### Cross-Compilation

When developing on x86_64 for ARM64 deployment:

```bash
# Install cross-compiler
sudo apt install gcc-aarch64-linux-gnu

# Build for ARM64
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

# Test on ARM64 device
scp bin/xvc-server arm64-server:/usr/local/bin/
```

## Security Considerations

### Network Security

**Risks:**
- Unauthorized access to FPGAs
- Man-in-the-middle attacks
- Denial-of-service attacks
- Information leakage

**Mitigations:**
- IP whitelisting (strict mode)
- Firewall rules
- VPN for remote access
- Network segmentation
- TLS encryption (future feature)

### Device Security

**Risks:**
- Unauthorized bitstream upload
- JTAG manipulation
- Side-channel attacks
- Physical tampering

**Mitigations:**
- FPGA bitstream encryption
- JTAG disable in production
- Physical security
- Access logging
- Audit trails

### System Security

**Risks:**
- Privilege escalation
- Resource exhaustion
- Service disruption
- Data corruption

**Mitigations:**
- Run as non-root user
- Resource limits (ulimit)
- Systemd isolation
- File permissions
- Regular updates

## Performance Benchmarks

### JTAG Speed Comparison

| Operation | Speed | Notes |
|-----------|-------|-------|
| Bitstream Download | 10-12 Mbps | HS2 max, depends on FPGA |
| Debug Waveform Capture | 8-10 Mbps | Continuous capture |
| IDCODE Read | 1-2 Mbps | Single operation |
| Short Shifts | 5-8 Mbps | Many small operations |

### Network Performance

| Scenario | Latency | Throughput |
|----------|---------|------------|
| LAN (1 Gbps) | 1-2 ms | ~10 Mbps (JTAG limited) |
| LAN (100 Mbps) | 2-5 ms | ~10 Mbps (JTAG limited) |
| WAN (fiber) | 20-50 ms | ~10 Mbps (JTAG limited) |
| VPN | 30-100 ms | ~10 Mbps (JTAG limited) |

### System Resource Usage

**Per Instance:**
- CPU: 5-15% (single core)
- Memory: ~100 MB
- File Descriptors: 7
- Threads: 7

**For 32 Instances:**
- CPU: ~160-480% (1.6-4.8 cores)
- Memory: ~3.2 GB
- File Descriptors: 224
- Threads: 224

## Troubleshooting Guide

### Common Issues

**1. Device Not Detected**
- Check USB connection: `lsusb -d 0403:6010`
- Verify libftdi installation: `ldd /usr/local/bin/xvc-server`
- Check USB permissions: `ls -l /dev/bus/usb/`

**2. Connection Refused**
- Verify instance running: `systemctl status xvc-server`
- Check port availability: `netstat -tuln | grep 2542`
- Review firewall rules: `iptables -L -n`

**3. Slow Performance**
- Check USB bandwidth: `lsusb -t`
- Adjust FTDI latency timer
- Verify network MTU: `ip link show`
- Check CPU usage: `top`

**4. Instance Crashes**
- Check logs: `journalctl -u xvc-server -n 50`
- Verify device health
- Check memory usage
- Review configuration

### Debug Tools

**USB Analysis:**
```bash
# List USB devices
lsusb -v

# Monitor USB traffic
sudo usbmon -i 0

# Check USB bandwidth
lsusb -t
```

**Network Analysis:**
```bash
# Capture XVC traffic
sudo tcpdump -i eth0 port 2542 -w xvc.pcap

# Test network latency
ping <server-ip>

# Test throughput
iperf3 -c <server-ip>
```

**System Monitoring:**
```bash
# Monitor CPU usage
top

# Monitor memory
free -h

# Monitor processes
htop

# Monitor file descriptors
lsof -p <pid>
```

## Future Enhancements

### Planned Features

1. **TLS/SSL Encryption**: Secure XVC connections
2. **Client Authentication**: Username/password support
3. **Web Management UI**: Browser-based control panel
4. **REST API**: Programmatic instance management
5. **Health Dashboard**: Real-time monitoring
6. **Auto-scaling**: Dynamic instance spawning
7. **Load Balancing**: Distribute connections across instances
8. **Device Pooling**: Share devices among instances
9. **USB Hub Support**: Manage multiple USB hubs
10. **SNMP Support**: Enterprise monitoring integration

### Experimental Features

1. **GPU Acceleration**: Offload JTAG processing
2. **RDMA Support**: High-speed networking
3. **Docker Integration**: Containerized deployment
4. **Kubernetes Integration**: Cloud-native deployment
5. **Multi-Vendor Support**: Support other JTAG adapters

## References

### Official Documentation
- [Xilinx Virtual Cable Protocol](https://github.com/Xilinx/XilinxVirtualCable)
- [FTDI FT2232H Datasheet](https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT2232H.pdf)
- [libftdi1 Documentation](https://www.intra2net.com/en/developer/libftdi/documentation/)

### Community Resources
- [xvcd (tmbinc)](https://github.com/tmbinc/xvcd)
- [OpenOCD](https://openocd.org/)
- [Digilent HS2 Product Page](https://digilent.com/reference/test-and-measurement/jtag-hs2/start)

### Standards
- [IEEE 1149.1 JTAG Standard](https://standards.ieee.org/)
- [USB 2.0 Specification](https://www.usb.org/)
- [TCP/IP Protocol Suite](https://tools.ietf.org/)
