#!/bin/bash
# Automated setup script for SlimeVR-Tracker-nRF on Ubuntu
# Run with: bash setup.sh

set -e  # Exit on error

echo "=========================================="
echo "SlimeVR-Tracker-nRF Setup for Ubuntu"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo -e "${YELLOW}Warning: This script is designed for Ubuntu. It may not work on other distributions.${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Step 1: Install system dependencies
echo -e "${GREEN}Step 1: Installing system dependencies...${NC}"
sudo apt update
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
    libmagic1 \
    screen

echo -e "${GREEN}✓ System dependencies installed${NC}"
echo ""

# Step 2: Install Python dependencies
echo -e "${GREEN}Step 2: Installing Python dependencies...${NC}"
python3 -m pip install --upgrade pip --user
pip3 install --user west

# Add to PATH
if ! grep -q ".local/bin" ~/.zshrc 2>/dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
    echo -e "${YELLOW}Added ~/.local/bin to PATH in ~/.zshrc${NC}"
fi

# Source for current session
export PATH="$HOME/.local/bin:$PATH"

echo -e "${GREEN}✓ Python dependencies installed${NC}"
echo ""

# Verify west
if ! command -v west &> /dev/null; then
    echo -e "${RED}Error: west command not found. Make sure ~/.local/bin is in your PATH${NC}"
    echo "Run: export PATH=\"\$HOME/.local/bin:\$PATH\""
    exit 1
fi

echo "West version: $(west --version)"
echo ""

# Step 3: Install Zephyr SDK
echo -e "${GREEN}Step 3: Installing Zephyr SDK...${NC}"
ZEPHYR_SDK_VERSION="0.16.5"
SDK_DIR="$HOME/zephyr-sdk-${ZEPHYR_SDK_VERSION}"

if [ -d "$SDK_DIR" ]; then
    echo -e "${YELLOW}Zephyr SDK already exists at $SDK_DIR${NC}"
    read -p "Reinstall? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$SDK_DIR"
    else
        echo "Skipping SDK installation"
    fi
fi

if [ ! -d "$SDK_DIR" ]; then
    cd ~
    SDK_FILE="zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz"
    
    if [ ! -f "$SDK_FILE" ]; then
        echo "Downloading Zephyr SDK..."
        wget "https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/${SDK_FILE}"
    fi
    
    echo "Extracting Zephyr SDK..."
    tar xf "$SDK_FILE"
    
    echo "Setting up Zephyr SDK..."
    cd "$SDK_DIR"
    ./setup.sh -t arm-zephyr-eabi -h
    
    # Clean up
    rm -f ~/"$SDK_FILE"
    
    echo -e "${GREEN}✓ Zephyr SDK installed${NC}"
else
    echo -e "${GREEN}✓ Zephyr SDK already installed${NC}"
fi

# Add SDK to environment
if ! grep -q "ZEPHYR_SDK_INSTALL_DIR" ~/.zshrc 2>/dev/null; then
    echo "export ZEPHYR_SDK_INSTALL_DIR=$SDK_DIR" >> ~/.zshrc
    echo -e "${YELLOW}Added ZEPHYR_SDK_INSTALL_DIR to ~/.zshrc${NC}"
fi

export ZEPHYR_SDK_INSTALL_DIR="$SDK_DIR"
echo ""

# Step 4: Install nRF Command Line Tools (optional)
echo -e "${GREEN}Step 4: Installing Nordic nRF Command Line Tools (optional)...${NC}"
read -p "Install nRF Command Line Tools? (needed for J-Link flashing) (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd ~
    NRF_TOOLS_VERSION="10.24.2"
    NRF_TOOLS_DEB="nrf-command-line-tools_${NRF_TOOLS_VERSION}_amd64.deb"
    
    if [ ! -f "$NRF_TOOLS_DEB" ]; then
        echo "Downloading nRF Command Line Tools..."
        wget "https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/${NRF_TOOLS_DEB}"
    fi
    
    echo "Installing nRF Command Line Tools..."
    sudo dpkg -i "$NRF_TOOLS_DEB" || sudo apt-get install -f -y
    
    rm -f "$NRF_TOOLS_DEB"
    echo -e "${GREEN}✓ nRF Command Line Tools installed${NC}"
else
    echo "Skipping nRF Command Line Tools installation"
fi
echo ""

# Step 5: Initialize west workspace
echo -e "${GREEN}Step 5: Initializing west workspace...${NC}"
cd /home/madkoding/SlimeVR-Tracker-nRF

# Check if already initialized
if [ ! -f .west/config ]; then
    echo "Initializing west..."
    west init -l .
else
    echo -e "${YELLOW}West workspace already initialized${NC}"
fi

echo "Updating west dependencies (this may take a while)..."
west update

# Install Zephyr Python requirements
if [ -f ../zephyr/scripts/requirements.txt ]; then
    echo "Installing Zephyr Python requirements..."
    pip3 install --user -r ../zephyr/scripts/requirements.txt
fi

echo -e "${GREEN}✓ West workspace initialized${NC}"
echo ""

# Step 6: Set up udev rules
echo -e "${GREEN}Step 6: Setting up udev rules...${NC}"
sudo tee /etc/udev/rules.d/99-nrf.rules > /dev/null <<'EOF'
# Nordic Semiconductor nRF devices
SUBSYSTEM=="usb", ATTR{idVendor}=="1915", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1366", MODE="0666"

# Adafruit bootloader (for UF2 flashing)
SUBSYSTEM=="usb", ATTR{idVendor}=="239a", MODE="0666"

# SEGGER J-Link
SUBSYSTEM=="usb", ATTR{idVendor}=="1366", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
if ! groups | grep -q dialout; then
    sudo usermod -a -G dialout $USER
    echo -e "${YELLOW}Added $USER to dialout group${NC}"
    echo -e "${YELLOW}⚠️  You need to log out and log back in for this to take effect!${NC}"
fi

echo -e "${GREEN}✓ udev rules configured${NC}"
echo ""

# Final verification
echo -e "${GREEN}=========================================="
echo "Installation Complete!"
echo "==========================================${NC}"
echo ""
echo "Verification:"
echo "  West version: $(west --version)"
echo "  SDK location: $ZEPHYR_SDK_INSTALL_DIR"
echo ""
echo -e "${YELLOW}Important: Source your shell configuration or restart terminal:${NC}"
echo "  source ~/.zshrc"
echo ""
echo -e "${YELLOW}If you were added to dialout group, log out and log back in!${NC}"
echo ""
echo "Next steps:"
echo "  1. Restart your terminal or run: source ~/.zshrc"
echo "  2. Verify installation: west boards | grep nrf"
echo "  3. Build firmware: west build -b xiao_ble_nrf52840 -p"
echo ""
echo "For detailed instructions, see: SETUP_UBUNTU.md"
echo ""
