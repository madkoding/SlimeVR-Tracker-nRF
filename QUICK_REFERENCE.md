# Quick Reference Guide - Building SlimeVR Firmware

## Quick Setup (Automated)

```bash
cd /home/madkoding/SlimeVR-Tracker-nRF
bash setup.sh
```

Then restart your terminal or run:
```bash
source ~/.zshrc
```

## Manual Setup (Step by Step)

If you prefer manual installation, follow `SETUP_UBUNTU.md`

## Building Firmware

### Available Boards

Check `pm_static_*.yml` files in the project root for available boards:
- `xiao_ble_nrf52840` - Seeed XIAO BLE
- `xiao_ble_nrf52840_sense` - Seeed XIAO BLE Sense
- `nrf52840dk_nrf52840` - Nordic nRF52840 DK
- `nrf52840dongle_nrf52840` - Nordic nRF52840 Dongle
- `promicro_uf2_nrf52840` - Pro Micro nRF52840
- `slimenrf_r3_nrf52840_uf2` - SlimeNRF R3
- `slimevrmini_p4_uf2_nrf52833` - SlimeVR Mini P4
- And many more...

### Basic Build Commands

```bash
# Build for XIAO BLE (most common)
west build -b xiao_ble_nrf52840 -p

# Build for specific board
west build -b BOARD_NAME -p

# Build without cleaning (faster)
west build -b xiao_ble_nrf52840

# Clean and rebuild
rm -rf build && west build -b xiao_ble_nrf52840
```

### Build Options

```bash
# Verbose output
west build -b xiao_ble_nrf52840 -v

# Build with pristine auto (clean if needed)
west build -b xiao_ble_nrf52840 --pristine=auto

# Build with specific overlay
west build -b xiao_ble_nrf52840 -- -DOVERLAY_CONFIG=overlay_i2c.conf
```

## Flashing Firmware

### Method 1: UF2 Bootloader (Easiest - for boards with Adafruit bootloader)

```bash
# 1. Build firmware
west build -b xiao_ble_nrf52840 -p

# 2. Double-press reset button on board
#    Board appears as USB drive (e.g., /media/$USER/XIAO-SENSE/)

# 3. Copy .uf2 file
cp build/zephyr/zephyr.uf2 /media/$USER/*/

# Board will automatically reset and run new firmware
```

### Method 2: J-Link (for Nordic DK or external J-Link)

```bash
# Build and flash in one command
west build -b nrf52840dk_nrf52840 -p
west flash

# Or manually with nrfjprog
nrfjprog --program build/zephyr/zephyr.hex --chiperase --verify --reset
```

### Method 3: DFU over USB

```bash
# Enter DFU mode on device first
west flash --runner dfu-util
```

## Viewing Debug Output

### Serial Console

```bash
# Using screen
screen /dev/ttyACM0 115200

# Exit screen: Ctrl+A, then K, then Y

# Using minicom
minicom -D /dev/ttyACM0 -b 115200

# Using pyserial-miniterm (if installed)
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

### Finding Serial Port

```bash
# List USB serial devices
ls /dev/ttyACM*
ls /dev/ttyUSB*

# Or use dmesg
dmesg | grep tty

# With more details
sudo dmesg -w
# Then plug in device to see which port it uses
```

## Common Issues & Solutions

### Issue: "west: command not found"
```bash
export PATH="$HOME/.local/bin:$PATH"
source ~/.zshrc
```

### Issue: "No module named 'elftools'"
```bash
pip3 install --user pyelftools
# Or install all Zephyr requirements:
pip3 install --user -r ../zephyr/scripts/requirements.txt
```

### Issue: "Zephyr SDK not found"
```bash
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5
# Add to ~/.zshrc to make permanent
echo 'export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5' >> ~/.zshrc
```

### Issue: Build fails with CMake errors
```bash
# Update west dependencies
west update

# Clean build directory
rm -rf build

# Try building again
west build -b xiao_ble_nrf52840 -p
```

### Issue: Permission denied when accessing /dev/ttyACM0
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in for this to take effect
```

### Issue: Device not appearing as USB drive
```bash
# Make sure to DOUBLE-PRESS the reset button quickly
# Check if device is detected:
lsusb | grep -i "adafruit\|nordic"

# Check dmesg for errors:
sudo dmesg | tail -20
```

## Testing the Freeze Fix

After building and flashing the freeze-fix branch:

### Monitor logs for protection messages:
```bash
screen /dev/ttyACM0 115200
```

Look for these messages (indicate protections are working):
- `"Timeout waiting for sensor scan, forcing exit"`
- `"Timeout waiting for sensor thread, forcing exit"`
- `"Sensor interrupt timeout #N"`
- `"Too many consecutive sensor timeouts, triggering recovery"`

### Test button responsiveness:
1. Let tracker run for several hours
2. Press SW0 button at random intervals
3. Verify button always responds
4. Check logs for any timeout messages

### Long-term stability test:
1. Leave tracker powered on for 24+ hours
2. Monitor serial output periodically
3. Test button responsiveness after extended runtime
4. Check for any freeze or unresponsive states

## Useful Development Commands

```bash
# View build configuration
west build -t menuconfig

# View all make targets
west build -t help

# Generate compile_commands.json (for IDE)
west build -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Check code size
west build -t rom_report
west build -t ram_report

# Run west topologies
west list

# Update a specific project
west update zephyr
```

## Environment Variables Reference

Add these to `~/.zshrc` for permanent configuration:

```bash
# Zephyr SDK location
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5

# Python user bin directory
export PATH="$HOME/.local/bin:$PATH"

# Enable ccache for faster rebuilds (optional)
export USE_CCACHE=1

# Zephyr base (usually auto-detected)
export ZEPHYR_BASE=/home/madkoding/zephyr
```

## Project Structure

```
SlimeVR-Tracker-nRF/
├── src/                    # Source code
│   ├── main.c             # Main application
│   ├── sensor/            # Sensor drivers
│   ├── system/            # System functions
│   └── connection/        # Communication
├── boards/                # Board definitions
├── build/                 # Build output (generated)
│   └── zephyr/
│       ├── zephyr.hex     # Flash with J-Link
│       ├── zephyr.uf2     # Copy to bootloader
│       └── zephyr.elf     # Debug symbols
├── CMakeLists.txt         # Build configuration
├── prj.conf              # Project Kconfig
├── west.yml              # West manifest
└── pm_static_*.yml       # Partition maps per board
```

## Next Steps After Installation

1. ✅ Verify installation: `west --version`
2. ✅ List boards: `west boards | grep nrf`
3. ✅ Build firmware: `west build -b xiao_ble_nrf52840 -p`
4. ✅ Flash to device: Use UF2 or J-Link
5. ✅ Monitor serial: `screen /dev/ttyACM0 115200`
6. ✅ Test freeze fixes: Press button, check logs

## Getting Help

- Project documentation: `README.md`
- Setup guide: `SETUP_UBUNTU.md`
- Freeze fix details: `FREEZE_FIX_COMPLETE.md`
- Zephyr docs: https://docs.zephyrproject.org
- Nordic docs: https://developer.nordicsemi.com

---
Last updated: 2025-10-02
