# Setup Guide for Ubuntu - SlimeVR Tracker nRF

This guide will help you set up the complete development environment for building SlimeVR Tracker firmware on Ubuntu.

## System Requirements

- Ubuntu 20.04, 22.04, or 24.04 (64-bit)
- At least 10 GB free disk space
- Stable internet connection

## Installation Steps

### Step 1: Install System Dependencies

```bash
# Update package lists
sudo apt update

# Install essential build tools
sudo apt install -y \
    git \
    cmake \
    ninja-build \
    gperf \
    ccache \
    dfu-util \
    device-tree-compiler \
    wget \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-tk \
    python3-wheel \
    xz-utils \
    file \
    make \
    gcc \
    gcc-multilib \
    g++-multilib \
    libsdl2-dev \
    libmagic1
```

### Step 2: Install Zephyr Python Dependencies

```bash
# Upgrade pip
python3 -m pip install --upgrade pip

# Install west (Zephyr's meta-tool)
pip3 install --user west

# Add pip user binaries to PATH (add this to ~/.bashrc or ~/.zshrc)
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc

# Verify west installation
west --version
```

### Step 3: Install Zephyr SDK (ARM Toolchain)

```bash
cd ~

# Download Zephyr SDK (version 0.16.5-1 - adjust if needed)
ZEPHYR_SDK_VERSION=0.16.5
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz

# Extract SDK
tar xvf zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz

# Run SDK setup script
cd zephyr-sdk-${ZEPHYR_SDK_VERSION}
./setup.sh -t arm-zephyr-eabi

# Register CMake package
sudo cp ~/zephyr-sdk-${ZEPHYR_SDK_VERSION}/sysroots/x86_64-pokysdk-linux/usr/share/cmake/Zephyr-sdk/cmake/* /usr/local/share/cmake/Zephyr-sdk/ 2>/dev/null || true

# Clean up downloaded archive
cd ~
rm zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz
```

### Step 4: Install Nordic nRF Command Line Tools (Optional but Recommended)

```bash
cd ~

# Download nRF Command Line Tools
wget https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/nrf-command-line-tools_10.24.2_amd64.deb

# Install
sudo dpkg -i nrf-command-line-tools_10.24.2_amd64.deb

# Fix dependencies if needed
sudo apt-get install -f

# Verify installation
nrfjprog --version

# Clean up
rm nrf-command-line-tools_10.24.2_amd64.deb
```

### Step 5: Initialize West Workspace

```bash
# Navigate to your project
cd /home/madkoding/SlimeVR-Tracker-nRF

# Initialize west if not already done
west init -l .

# Update dependencies
west update

# Install Python requirements from Zephyr
pip3 install --user -r ../zephyr/scripts/requirements.txt
```

### Step 6: Set Up udev Rules for nRF Devices (Required for Flashing)

```bash
# Create udev rules file
sudo tee /etc/udev/rules.d/99-nrf.rules > /dev/null <<'EOF'
# Nordic Semiconductor nRF devices
SUBSYSTEM=="usb", ATTR{idVendor}=="1915", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1366", MODE="0666"

# Adafruit bootloader (for UF2 flashing)
SUBSYSTEM=="usb", ATTR{idVendor}=="239a", MODE="0666"

# SEGGER J-Link
SUBSYSTEM=="usb", ATTR{idVendor}=="1366", MODE="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add your user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

echo "⚠️  You need to log out and log back in for group changes to take effect!"
```

### Step 7: Verify Installation

```bash
cd /home/madkoding/SlimeVR-Tracker-nRF

# Check west version
west --version

# List available boards
west boards | grep nrf

# Check Zephyr version
west list
```

## Building the Firmware

### Basic Build Command

```bash
cd /home/madkoding/SlimeVR-Tracker-nRF

# Build for a specific board (example: xiao_ble)
west build -b xiao_ble_nrf52840 -p

# Or for another board
west build -b nrf52840dk_nrf52840 -p
```

### Common Boards

- `xiao_ble_nrf52840` - Seeed XIAO BLE nRF52840
- `nrf52840dk_nrf52840` - Nordic nRF52840 Development Kit
- `nrf52840dongle_nrf52840` - Nordic nRF52840 Dongle

### Build with Specific Configuration

```bash
# Clean build
west build -b xiao_ble_nrf52840 -p

# Build without cleaning (faster)
west build -b xiao_ble_nrf52840

# Build with verbose output
west build -b xiao_ble_nrf52840 -v
```

## Flashing the Firmware

### Method 1: Using J-Link (Nordic DK or external J-Link)

```bash
# Flash using west
west flash

# Or using nrfjprog directly
nrfjprog --program build/zephyr/zephyr.hex --chiperase --verify --reset
```

### Method 2: Using UF2 Bootloader (Adafruit bootloader)

```bash
# Build will create .uf2 file in build/zephyr/

# 1. Double-press reset button on your board
# 2. Board should appear as USB drive
# 3. Copy the .uf2 file to the drive

cp build/zephyr/zephyr.uf2 /media/$USER/*/
```

### Method 3: Using DFU over USB

```bash
# Enter DFU mode on device
# Then flash with:
west flash --runner dfu-util
```

## Troubleshooting

### Problem: "west: command not found"

```bash
# Make sure ~/.local/bin is in PATH
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

### Problem: "Zephyr SDK not found"

```bash
# Set environment variable
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5
echo 'export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5' >> ~/.zshrc
```

### Problem: "Permission denied" when flashing

```bash
# Make sure udev rules are installed and user is in dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### Problem: Build fails with "module not found"

```bash
# Update all dependencies
cd /home/madkoding/SlimeVR-Tracker-nRF
west update
pip3 install --user -r ../zephyr/scripts/requirements.txt
```

### Problem: "CMake Error: Could not find toolchain"

```bash
# Re-run SDK setup
cd ~/zephyr-sdk-0.16.5
./setup.sh -t arm-zephyr-eabi
```

## Useful Commands

### Clean Build Directory

```bash
rm -rf build
```

### View Build Configuration

```bash
west build -t menuconfig
```

### List All Build Targets

```bash
west build -t help
```

### View Board Configuration

```bash
west build -b xiao_ble_nrf52840 -t guiconfig
```

## Environment Variables (Add to ~/.zshrc)

```bash
# Zephyr SDK
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5

# West workspace
export ZEPHYR_BASE=/home/madkoding/zephyr

# Add local bin to PATH
export PATH="$HOME/.local/bin:$PATH"

# Optional: Enable ccache for faster rebuilds
export USE_CCACHE=1
```

## Next Steps

1. **Verify installation**: Run `west --version` and `west boards`
2. **Build firmware**: Try building for your specific board
3. **Flash and test**: Flash the freeze-fix branch and test
4. **Monitor logs**: Use serial console to view debug output

## Serial Console Monitoring

To view debug logs from the device:

```bash
# Install screen or minicom
sudo apt install screen

# Connect to serial port (adjust ttyACM0 as needed)
screen /dev/ttyACM0 115200

# Or use minicom
minicom -D /dev/ttyACM0 -b 115200
```

To exit screen: Press `Ctrl+A` then `K` then `Y`

## Additional Resources

- [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
- [Nordic nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/index.html)
- [West (Zephyr's meta-tool)](https://docs.zephyrproject.org/latest/develop/west/index.html)

---

**Installation Date**: 2025-10-02  
**For**: SlimeVR-Tracker-nRF freeze-fix branch
