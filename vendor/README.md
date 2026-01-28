# Vendor Libraries

This directory contains all external libraries required by the XVC server.

## Philosophy
All external dependencies are **bundled locally** in this `vendor/` directory. This ensures:

- **Reproducible builds**: No dependency on system library versions
- **Version control**: Specific library versions are tracked in repository
- **Cross-compilation**: Works consistently across different platforms
- **Self-contained**: No system-wide installations required

## Only System Dependencies

The only dependencies taken from the system are:
- **C standard library** (libc, libm, libpthread)
- **Standard system headers**

All other libraries are built locally from source.

## Directory Structure

```
vendor/
├── README.md                 # This file
├── libftdi1/              # FTDI library for HS2 communication
│   ├── src/                # Source code
│   ├── build/              # Build output (generated)
│   └── Makefile            # Build configuration
└── libusb/                 # USB library for device discovery
    ├── src/                # Source code
    ├── build/              # Build output (generated)
    └── Makefile            # Build configuration
```

## libftdi1

**Purpose**: Library for communicating with FTDI USB devices (Digilent HS2)

**Version**: 1.5 (or latest stable release)

**Source**: https://www.intra2net.com/en/developer/libftdi/download-libftdi

**Used by**: `xvc-server` (for HS2 communication)

**Build Instructions**:
```bash
cd vendor/libftdi1
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$(PWD)/..
make
```

**API Used**:
- `ftdi_init()` - Initialize FTDI context
- `ftdi_usb_open()` - Open FTDI device
- `ftdi_set_bitmode()` - Set operating mode
- `ftdi_read_data()` - Read data from device
- `ftdi_write_data()` - Write data to device
- `ftdi_usb_close()` - Close device

## libusb

**Purpose**: Low-level USB library for device discovery

**Version**: 1.0.26 (or latest stable release)

**Source**: https://github.com/libusb/libusb

**Used by**: `xvc-discover` (for device scanning)

**Build Instructions**:
```bash
cd vendor/libusb
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$(PWD)/..
make
```

**API Used**:
- `libusb_init()` - Initialize library
- `libusb_get_device_list()` - Get list of USB devices
- `libusb_open_device_with_vid_pid()` - Open device by VID/PID
- `libusb_get_device_descriptor()` - Get device information
- `libusb_free_device_list()` - Free device list

## Building Vendor Libraries

The main Makefile automatically builds vendor libraries:

```bash
# Build only vendor libraries
make vendor-lib

# Clean only vendor libraries
make vendor-clean
```

Vendor libraries are also built automatically when running `make all`.

## Cross-Compilation

Vendor libraries can be cross-compiled for ARM64:

```bash
# Build for ARM64
make vendor-lib ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

# The vendor library Makefiles will use the cross-compiler
```

## Version Pinning

To ensure reproducible builds, library versions are pinned:

**libftdi1**: Version 1.5
**libusb**: Version 1.0.26

To update a library:
1. Download new version
2. Extract to `vendor/<library>/`
3. Update version in this README
4. Run `make vendor-clean && make vendor-lib`

## Security Considerations

### Trust
Vendor libraries are trusted because:
- They are from official sources
- Version-controlled in repository
- Built locally from source
- No binary downloads from untrusted sources

### Verification
To verify library integrity:

```bash
# Check git commits
cd vendor/libftdi1
git log --oneline -10

# Check file hashes
sha256sum vendor/libftdi1/src/ftdi.c
```

## Troubleshooting

### Build Failures

**libftdi1 build fails:**
```bash
# Check for required dependencies
sudo apt install build-essential cmake libusb-dev

# Clean and rebuild
make vendor-clean
make vendor-lib
```

**libusb build fails:**
```bash
# Check for required dependencies
sudo apt install build-essential cmake

# Clean and rebuild
make vendor-clean
make vendor-lib
```

### Cross-Compilation Issues

**ARM64 cross-compiler not found:**
```bash
# Install cross-compiler
sudo apt install gcc-aarch64-linux-gnu

# Verify installation
aarch64-linux-gnu-gcc --version
```

**Library architecture mismatch:**
```bash
# Check library architecture
file vendor/libftdi1/build/libftdi.so.1

# Should show: ELF 64-bit LSB shared object, ARM aarch64
```

## Maintenance

### Updating Libraries

1. **Check upstream**: Visit library websites for updates
2. **Download source**: Get new version tarball
3. **Replace files**: Extract to `vendor/<library>/`
4. **Test build**: Run `make vendor-clean && make vendor-lib`
5. **Test functionality**: Build and test xvc-server
6. **Commit changes**: Update repository with new version

### Removing Libraries

To remove a vendor library:
```bash
rm -rf vendor/<library-name>
# Then remove references from main Makefile
```

## References

- **libftdi1**: https://www.intra2net.com/en/developer/libftdi
- **libusb**: https://libusb.info/
- **FTDI Chip Documentation**: https://www.ftdichip.com/
- **Digilent HS2**: https://digilent.com/reference/test-and-measurement/jtag-hs2/start

## License

- **libftdi1**: LGPL v2.1
- **libusb**: LGPL v2.1

Both libraries are compatible with our project's open-source requirements.
