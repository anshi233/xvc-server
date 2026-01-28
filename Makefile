# XVC Server Makefile
# Supports: ARM64, x86_64, cross-compilation
# Two binaries: xvc-discover and xvc-server
# All external libraries are local (vendor/), only stdlib from system

# Detect architecture
ARCH ?= $(shell uname -m)
CROSS_COMPILE ?=

# Compiler and flags
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Wextra -O2 -g
CFLAGS += -I./include

# Use system libraries if vendor libs not present (native build)
# Set USE_SYSTEM_LIBS=1 to use system packages, 0 to use vendor libs
USE_SYSTEM_LIBS ?= 0

ifeq ($(USE_SYSTEM_LIBS),1)
    # Use system libftdi and libusb (pkg-config)
    FTDI_CFLAGS := $(shell pkg-config --cflags libftdi1 2>/dev/null || echo "-I/usr/include/libftdi1")
    FTDI_LIBS := $(shell pkg-config --libs libftdi1 2>/dev/null || echo "-lftdi1")
    USB_CFLAGS := $(shell pkg-config --cflags libusb-1.0 2>/dev/null || echo "-I/usr/include/libusb-1.0")  
    USB_LIBS := $(shell pkg-config --libs libusb-1.0 2>/dev/null || echo "-lusb-1.0")
    CFLAGS += $(FTDI_CFLAGS) $(USB_CFLAGS)
    LDFLAGS_SERVER = $(FTDI_LIBS) $(USB_LIBS) -lpthread -lm
    LDFLAGS_DISCOVER = $(USB_LIBS) -lpthread
else
    # Use local vendor libraries (static linking)
    VENDOR_LIBUSB = ./vendor/libusb/install
    VENDOR_LIBFTDI = ./vendor/libftdi1/install
    CFLAGS += -I$(VENDOR_LIBFTDI)/include/libftdi1 -I$(VENDOR_LIBUSB)/include/libusb-1.0
    LDFLAGS_SERVER = $(VENDOR_LIBFTDI)/lib/libftdi1.a $(VENDOR_LIBUSB)/lib/libusb-1.0.a -lpthread -lm
    LDFLAGS_DISCOVER = $(VENDOR_LIBUSB)/lib/libusb-1.0.a -lpthread
endif

# Debug mode
DEBUG ?= 0
ifeq ($(DEBUG),1)
    CFLAGS += -DDEBUG -g
else
    CFLAGS += -DNDEBUG
endif

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
CONFIG_DIR = config
LOG_DIR = logs
SCRIPT_DIR = scripts
VENDOR_DIR = vendor
ABS_VENDOR_DIR = $(abspath $(VENDOR_DIR))

# xvc-server source files
SERVER_SOURCES = $(SRC_DIR)/main.c \
                  $(SRC_DIR)/tcp_server.c \
                  $(SRC_DIR)/device_manager.c \
                  $(SRC_DIR)/xvc_protocol.c \
                  $(SRC_DIR)/ftdi_adapter.c \
                  $(SRC_DIR)/mpsse_adapter.c \
                  $(SRC_DIR)/config.c \
                  $(SRC_DIR)/whitelist.c \
                  $(SRC_DIR)/logging.c

# xvc-discover source files
DISCOVER_SOURCES = $(SRC_DIR)/discover_main.c \
                    $(SRC_DIR)/device_manager.c \
                    $(SRC_DIR)/config.c \
                    $(SRC_DIR)/logging.c

# Object files
SERVER_OBJECTS = $(SERVER_SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
DISCOVER_OBJECTS = $(DISCOVER_SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR)/discover_%.o)

# Target binaries
TARGET_SERVER = $(BIN_DIR)/xvc-server
TARGET_DISCOVER = $(BIN_DIR)/xvc-discover

# Default target (build both)
ifeq ($(USE_SYSTEM_LIBS),1)
all: dirs $(TARGET_SERVER) $(TARGET_DISCOVER)
else
all: dirs vendor-lib $(TARGET_SERVER) $(TARGET_DISCOVER)
endif

# Create directories
dirs:
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(CONFIG_DIR)
	@mkdir -p $(LOG_DIR)
	@mkdir -p $(SCRIPT_DIR)
	@mkdir -p $(VENDOR_DIR)

# Build vendor libraries
# Build vendor libraries
vendor-lib: libusb-build libftdi-build

libusb-build:
	@echo "Building libusb..."
	@if [ ! -f $(VENDOR_DIR)/libusb/Makefile ]; then \
		echo "Configuring libusb..."; \
		cd $(VENDOR_DIR)/libusb && CFLAGS="-fPIC" ./configure --prefix=$(ABS_VENDOR_DIR)/libusb/install --enable-static --disable-shared --disable-udev; \
	fi
	@cd $(VENDOR_DIR)/libusb && $(MAKE) && $(MAKE) install

libftdi-build:
	@echo "Building libftdi1 (static only)..."
	@mkdir -p $(VENDOR_DIR)/libftdi1/build
	@cd $(VENDOR_DIR)/libftdi1/build && cmake .. \
		-DCMAKE_INSTALL_PREFIX=$(ABS_VENDOR_DIR)/libftdi1/install \
		-DLIBUSB_INCLUDE_DIR=$(ABS_VENDOR_DIR)/libusb/install/include/libusb-1.0 \
		-DLIBUSB_LIBRARIES=$(ABS_VENDOR_DIR)/libusb/install/lib/libusb-1.0.a \
		-DFTDI_EEPROM=OFF -DDOCUMENTATION=OFF -DEXAMPLES=OFF \
		-DPYTHON_BINDINGS=OFF -DFTDIPP=OFF -DBUILD_TESTS=OFF \
		-DSTATICLIBS=ON && \
	$(MAKE) ftdi1-static && \
	mkdir -p $(ABS_VENDOR_DIR)/libftdi1/install/lib && \
	mkdir -p $(ABS_VENDOR_DIR)/libftdi1/install/include/libftdi1 && \
	cp src/libftdi1.a $(ABS_VENDOR_DIR)/libftdi1/install/lib/ && \
	cp ../src/ftdi.h $(ABS_VENDOR_DIR)/libftdi1/install/include/libftdi1/

# Clean vendor libraries (delete directly to avoid path issues from cross-machine copies)
vendor-clean:
	@echo "Cleaning vendor libraries..."
	@rm -rf $(VENDOR_DIR)/libftdi1/build $(VENDOR_DIR)/libftdi1/install
	@rm -rf $(VENDOR_DIR)/libusb/install $(VENDOR_DIR)/libusb/.libs $(VENDOR_DIR)/libusb/libusb/.libs
	@rm -f $(VENDOR_DIR)/libusb/Makefile $(VENDOR_DIR)/libusb/config.status $(VENDOR_DIR)/libusb/config.log
	@rm -f $(VENDOR_DIR)/libusb/libtool $(VENDOR_DIR)/libusb/config.h $(VENDOR_DIR)/libusb/stamp-h1
	@rm -f $(VENDOR_DIR)/libusb/libusb/Makefile $(VENDOR_DIR)/libusb/examples/Makefile $(VENDOR_DIR)/libusb/tests/Makefile
	@echo "Vendor libraries cleaned"

# Link xvc-server
$(TARGET_SERVER): $(SERVER_OBJECTS)
	@echo "Linking xvc-server..."
	$(CC) $(SERVER_OBJECTS) -o $@ $(LDFLAGS_SERVER)
	@echo "Build complete: $@"

# Link xvc-discover
$(TARGET_DISCOVER): $(DISCOVER_OBJECTS)
	@echo "Linking xvc-discover..."
	$(CC) $(DISCOVER_OBJECTS) -o $@ $(LDFLAGS_DISCOVER)
	@echo "Build complete: $@"

# Compile xvc-server source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Compile xvc-discover source files (with prefix)
$(OBJ_DIR)/discover_%.o: $(SRC_DIR)/%.c
	@echo "Compiling $< (for xvc-discover)..."
	$(CC) $(CFLAGS) -c $< -o $@

# Build only xvc-server
xvc-server: dirs $(TARGET_SERVER)

# Build only xvc-discover
xvc-discover: dirs $(TARGET_DISCOVER)

# Install both binaries
install: install-server install-discover
	@echo "Installation complete"

# Install xvc-server only
install-server: $(TARGET_SERVER)
	@echo "Installing xvc-server..."
	install -d /usr/local/bin
	install -m 755 $(TARGET_SERVER) /usr/local/bin/xvc-server
	install -d /etc/xvc-server
	install -m 644 config/xvc-server-multi.conf.example /etc/xvc-server/xvc-server-multi.conf.example
	install -d /etc/xvc-server/scripts
	install -m 755 scripts/xvc-server-daemon.sh /etc/xvc-server/xvc-server-daemon.sh
	install -d /etc/systemd/system
	install -m 644 scripts/xvc-server.service /etc/systemd/system/xvc-server.service
	@systemctl daemon-reload
	@echo "xvc-server installed"

# Install xvc-discover only
install-discover: $(TARGET_DISCOVER)
	@echo "Installing xvc-discover..."
	install -d /usr/local/bin
	install -m 755 $(TARGET_DISCOVER) /usr/local/bin/xvc-discover
	@echo "xvc-discover installed"

# Install systemd service
install-systemd:
	@echo "Installing systemd service..."
	install -d /etc/systemd/system
	install -m 644 scripts/xvc-server.service /etc/systemd/system/xvc-server.service
	@systemctl daemon-reload
	@echo "systemd service installed"

# Uninstall
uninstall:
	@echo "Uninstalling xvc-server and xvc-discover..."
	systemctl stop xvc-server 2>/dev/null || true
	rm -f /usr/local/bin/xvc-server
	rm -f /usr/local/bin/xvc-discover
	rm -rf /etc/xvc-server
	rm -f /etc/systemd/system/xvc-server.service
	systemctl daemon-reload
	@echo "Uninstall complete"

# Clean build artifacts and vendor configurations
clean: vendor-clean
	@echo "Cleaning..."
	rm -rf $(OBJ_DIR) $(BIN_DIR)
	@echo "Clean complete"

# Clean everything including config and log directories
distclean: clean
	rm -rf $(CONFIG_DIR) $(LOG_DIR)
	@echo "Full clean complete"

# Rebuild everything
rebuild: clean all

# Run tests (if implemented)
test: $(TARGET_SERVER) $(TARGET_DISCOVER)
	@echo "Running tests..."
	@echo "Testing xvc-discover..."
	$(TARGET_DISCOVER) --help
	@echo "Testing xvc-server..."
	$(TARGET_SERVER) --help
	@echo "Tests passed"

# Show help
help:
	@echo "XVC Server Makefile"
	@echo "Two binaries: xvc-discover and xvc-server"
	@echo "All external libraries are local (vendor/), only stdlib from system"
	@echo ""
	@echo "Targets:"
	@echo "  all              - Build vendor libs and both xvc-server and xvc-discover (default)"
	@echo "  vendor-lib       - Build vendor libraries only"
	@echo "  xvc-server       - Build xvc-server only"
	@echo "  xvc-discover     - Build xvc-discover only"
	@echo "  clean            - Remove build artifacts"
	@echo "  vendor-clean     - Clean vendor libraries only"
	@echo "  distclean        - Clean build artifacts and vendor libraries"
	@echo "  install          - Install both binaries to system"
	@echo "  install-server   - Install xvc-server only"
	@echo "  install-discover - Install xvc-discover only"
	@echo "  install-systemd  - Install systemd service only"
	@echo "  uninstall        - Remove xvc-server and xvc-discover from system"
	@echo "  test             - Run basic tests"
	@echo "  rebuild          - Clean and rebuild everything"
	@echo ""
	@echo "Options:"
	@echo "  DEBUG=1    - Build with debug symbols"
	@echo "  ARCH=arm64 - Set target architecture"
	@echo "  CROSS_COMPILE=aarch64-linux-gnu- - Cross-compile"
	@echo ""
	@echo "Usage Examples:"
	@echo "  make                    # Build everything (vendor libs + binaries)"
	@echo "  make vendor-lib         # Build vendor libraries only"
	@echo "  make xvc-discover       # Build discovery tool only"
	@echo "  make install-server     # Install xvc-server"
	@echo "  make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-"

.PHONY: all dirs vendor-lib vendor-clean xvc-server xvc-discover install install-server install-discover install-systemd uninstall clean distclean rebuild test help
