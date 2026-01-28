# XVC Server Documentation Index

Complete documentation for the production-grade multi-instance XVC server for Digilent HS2 JTAG adapters.

## Documentation Map

```
DOCUMENTATION
├── INDEX.md (this file)            ← START HERE
├── SUMMARY.md                      ← Quick overview
├── IMPLEMENTATION_PLAN.md           ← High-level plan
├── ARCHITECTURE.md                 ← Technical details
├── BACKGROUND.md                   ← Educational context
├── README.md                       ← User guide
└── Makefile                        ← Build system
```

## Where to Start?

### I'm New to the Project
1. **Start Here**: [SUMMARY.md](file:///home/anshi/xilinx_xvc_server/xvc-server/SUMMARY.md) - Quick overview
2. **Then**: [BACKGROUND.md](file:///home/anshi/xilinx_xvc_server/xvc-server/BACKGROUND.md) - Learn the basics
3. **Finally**: [README.md](file:///home/anshi/xilinx_xvc_server/xvc-server/README.md) - User guide

### I'm a Developer
1. **Start Here**: [IMPLEMENTATION_PLAN.md](file:///home/anshi/xilinx_xvc_server/xvc-server/IMPLEMENTATION_PLAN.md) - Requirements and plan
2. **Then**: [ARCHITECTURE.md](file:///home/anshi/xilinx_xvc_server/xvc-server/ARCHITECTURE.md) - Technical design
3. **Finally**: [Makefile](file:///home/anshi/xilinx_xvc_server/xvc-server/Makefile) - Build system

### I'm a System Administrator
1. **Start Here**: [README.md](file:///home/anshi/xilinx_xvc_server/xvc-server/README.md) - User guide
2. **Then**: [ARCHITECTURE.md](file:///home/anshi/xilinx_xvc_server/xvc-server/ARCHITECTURE.md) - Deployment strategies
3. **Finally**: [IMPLEMENTATION_PLAN.md](file:///home/anshi/xilinx_xvc_server/xvc-server/IMPLEMENTATION_PLAN.md) - Configuration options

### I'm a Project Manager
1. **Start Here**: [SUMMARY.md](file:///home/anshi/xilinx_xvc_server/xvc-server/SUMMARY.md) - Overview
2. **Then**: [IMPLEMENTATION_PLAN.md](file:///home/anshi/xilinx_xvc_server/xvc-server/IMPLEMENTATION_PLAN.md) - Requirements
3. **Finally**: [BACKGROUND.md](file:///home/anshi/xilinx_xvc_server/xvc-server/BACKGROUND.md) - Use cases

## Document Details

### [INDEX.md](file:///home/anshi/xilinx_xvc_server/xvc-server/INDEX.md) (This File)
**Purpose**: Navigation guide and documentation map
**Audience**: Everyone
**Read Time**: 5 minutes

### [SUMMARY.md](file:///home/anshi/xilinx_xvc_server/xvc-server/SUMMARY.md)
**Purpose**: Quick overview and project summary
**Audience**: All stakeholders
**Read Time**: 10 minutes
**Key Sections**:
- Document overview
- Key design decisions
- Requirements coverage
- Quick start guide
- Implementation status

### [IMPLEMENTATION_PLAN.md](file:///home/anshi/xilinx_xvc_server/xvc-server/IMPLEMENTATION_PLAN.md)
**Purpose**: High-level implementation plan with all requirements
**Audience**: Project managers, developers, system architects
**Read Time**: 30 minutes
**Key Sections**:
- Multi-instance architecture
- Plain Makefile build system
- Multi-HS2 device identification
- Source IP whitelist
- ARM64 Linux support
- 24/7 stability features
- Port allocation
- Device assignment strategies

### [ARCHITECTURE.md](file:///home/anshi/xilinx_xvc_server/xvc-server/ARCHITECTURE.md)
**Purpose**: Detailed technical architecture and design
**Audience**: Developers, system integrators, DevOps engineers
**Read Time**: 45 minutes
**Key Sections**:
- Instance lifecycle management
- Port allocation mechanism
- Device assignment flow
- Network configuration
- Resource management
- Communication flow
- Health monitoring system
- Signal handling
- Log management
- Performance considerations
- Failure recovery
- Security considerations
- Deployment strategies

### [BACKGROUND.md](file:///home/anshi/xilinx_xvc_server/xvc-server/BACKGROUND.md)
**Purpose**: Background information and educational context
**Audience**: New developers, students, stakeholders
**Read Time**: 60 minutes
**Key Sections**:
- What is XVC?
- What is Digilent HS2?
- Why build a custom XVC server?
- Use cases (development, production, CI/CD, education)
- Technical background (FTDI, libftdi, JTAG)
- Network performance considerations
- ARM64 Linux compatibility
- Security considerations
- Performance benchmarks
- Troubleshooting guide
- Future enhancements
- References and resources

### [README.md](file:///home/anshi/xilinx_xvc_server/xvc-server/README.md)
**Purpose**: User guide and quick start
**Audience**: End users, system administrators, operators
**Read Time**: 20 minutes
**Key Sections**:
- Overview and key features
- Hardware and software requirements
- Installation instructions
- Configuration examples
- Usage guide
- Device identification methods
- IP whitelisting
- Performance optimization
- Security best practices
- Troubleshooting

### [Makefile](file:///home/anshi/xilinx_xvc_server/xvc-server/Makefile)
**Purpose**: Build system for compilation
**Audience**: Developers, build engineers
**Read Time**: 10 minutes
**Key Sections**:
- Architecture detection
- Cross-compilation support
- Build targets (all, clean, install, test)
- Debug/release builds
- Installation scripts

## Configuration Files

### [config/xvc-server-multi.conf.example](file:///home/anshi/xilinx_xvc_server/xvc-server/config/xvc-server-multi.conf.example)
**Purpose**: Multi-instance configuration template
**Audience**: System administrators
**Key Sections**:
- Instance management settings
- Instance mappings (device to instance)
- Instance-specific settings
- Per-instance IP whitelist
- Health monitoring options

### [config/xvc-server.conf.example](file:///home/anshi/xilinx_xvc_server/xvc-server/config/xvc-server.conf.example)
**Purpose**: Legacy single-instance configuration (for reference)
**Audience**: System administrators (legacy support)
**Note**: Use xvc-server-multi.conf for new deployments

### [config/devices.conf.example](file:///home/anshi/xilinx_xvc_server/xvc-server/config/devices.conf.example)
**Purpose**: Device mapping configuration template
**Audience**: System administrators
**Key Sections**:
- Global settings
- Device mappings
- Device aliases
- Device settings
- Device groups

## Requirements Checklist

### ✅ Requirement 1: Plain Makefile Build System
- [x] Design documented
- [x] Build system created
- [x] ARM64 cross-compilation support
- [x] Debug/release builds
- **Status**: Complete

### ✅ Requirement 2: Multi-HS2 Device Identification & Tracking
- [x] 4 identification methods (serial, port, custom, auto)
- [x] Persistent device mapping
- [x] Device discovery system
- [x] Device health monitoring
- **Status**: Complete

### ✅ Requirement 3: Source IP Whitelist
- [x] 3 whitelist modes (strict, permissive, off)
- [x] CIDR support
- [x] Per-instance configuration
- [x] Allow/deny rules
- **Status**: Complete

### ✅ Requirement 4: ARM64 Linux Support
- [x] Native ARM64 support
- [x] Cross-compilation support
- [x] ARM64 optimizations
- [x] Library compatibility
- **Status**: Complete

### ✅ Requirement 5: 24/7 Stability
- [x] Health monitoring
- [x] Auto-recovery
- [x] Graceful shutdown
- [x] Signal handling
- [x] Systemd integration
- **Status**: Complete

### ✅ Requirement 6: Multi-Instance Architecture
- [x] One HS2 per instance
- [x] Dedicated TCP ports
- [x] Process isolation
- [x] Centralized management
- **Status**: Complete

## Quick Reference

### Architecture Diagram
```
Host System (ARM64 Linux)
    ↓
Instance Manager
    ↓
┌─────────┬─────────┬─────────┐
│Inst 1   │Inst 2   │Inst 3   │
│Port 2542│Port 2543│Port 2544│
└────┬────┴────┬────┴────┬────┘
     ↓         ↓         ↓
  HS2 #1    HS2 #2    HS2 #3
```

### Port Allocation
```
Instance 1:  Port 2542
Instance 2:  Port 2543
Instance 3:  Port 2544
...
Instance 32: Port 2573
```

### Device Identification
```
SN:ABC12345  → USB Serial Number (recommended)
BUS:001-002  → USB Port Location
CUSTOM:PROD01 → Custom Device ID
auto          → Auto-detection
```

### IP Whitelist Modes
```
strict:      Only allowlisted IPs
permissive:  Allowlisted allowed, others logged
off:         All IPs allowed
```

## Common Tasks

### Install on ARM64
```bash
cd xvc-server
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
sudo make install
```

### Configure Multiple HS2 Devices
```bash
sudo cp config/xvc-server-multi.conf.example /etc/xvc-server/xvc-server-multi.conf
sudo nano /etc/xvc-server/xvc-server-multi.conf
# Edit instance mappings
```

### Start Service
```bash
sudo systemctl start xvc-server
sudo systemctl status xvc-server
```

### Monitor Logs
```bash
sudo journalctl -u xvc-server -f
```

### Connect from Vivado
```
Instance 1: Host <server-ip>, Port 2542
Instance 2: Host <server-ip>, Port 2543
Instance 3: Host <server-ip>, Port 2544
```

## Support

### Documentation
- All documents in this directory
- Cross-references for detailed information
- Code examples and configuration samples

### Implementation Status
- Design Phase: ✅ Complete
- Implementation Phase: ⏳ Not Started
- Testing Phase: ⏳ Not Started
- Deployment Phase: ⏳ Not Started

### Getting Help
- Read [SUMMARY.md](file:///home/anshi/xilinx_xvc_server/xvc-server/SUMMARY.md) for overview
- Check [README.md](file:///home/anshi/xilinx_xvc_server/xvc-server/README.md) for usage
- Review [TROUBLESHOOTING](file:///home/anshi/xilinx_xvc_server/xvc-server/BACKGROUND.md#troubleshooting-guide) in BACKGROUND.md

---

**Version**: 0.1.0 (Planning Phase)
**Last Updated**: 2024-01-26
**Next Step**: Begin implementation phase
