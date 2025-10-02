# How to Flash the Firmware

## Build Summary ✅

**Build Status:** SUCCESS  
**Firmware Size:** 217KB / 796KB Flash (26.7%), 39KB / 252KB RAM (15.3%)  
**Target Board:** xiao_ble (Seeed XIAO BLE nRF52840)  
**Output Files:**
- `build/SlimeVR-Tracker-nRF/zephyr/zephyr.uf2` (426 KB) - **Main firmware file**
- `build/SlimeVR-Tracker-nRF/zephyr/zephyr.hex` (598 KB) - Alternative format
- `build/SlimeVR-Tracker-nRF/zephyr/zephyr.elf` (6.4 MB) - Debug symbols

---

## Method 1: UF2 Bootloader (Easiest - Recommended)

### For Seeed XIAO BLE:

1. **Enter Bootloader Mode:**
   - Connect the XIAO BLE to your computer via USB
   - Double-click the RESET button quickly
   - A USB drive named "XIAO-SENSE" or "XIAO-nRF52" should appear

2. **Flash the Firmware:**
   ```bash
   # Copy the UF2 file to the mounted drive
   cp build/SlimeVR-Tracker-nRF/zephyr/zephyr.uf2 /media/$USER/XIAO*/
   
   # Or manually drag-and-drop zephyr.uf2 to the mounted drive
   ```

3. **Wait for Completion:**
   - The device will automatically reboot after flashing
   - The USB drive will disconnect
   - Your tracker should now be running the new firmware with freeze fixes

---

## Method 2: Using nrfjprog (Nordic Development Kit)

If you have an nRF52840 DK or J-Link programmer:

```bash
# Flash using nrfjprog
nrfjprog --program build/SlimeVR-Tracker-nRF/zephyr/zephyr.hex --chiperase --verify --reset
```

---

## Method 3: Using west flash

If the device is connected via USB and bootloader is active:

```bash
cd /home/madkoding/SlimeVR-Tracker-nRF
west flash
```

---

## After Flashing

### 1. Verify Serial Output (Optional)

Connect to the serial console to see logs:

```bash
# Find the device
ls /dev/ttyACM*

# Connect with screen (Ctrl+A, K to exit)
screen /dev/ttyACM0 115200

# Or use minicom
minicom -D /dev/ttyACM0 -b 115200
```

**Note:** You may need to log out and log back in for `dialout` group membership to take effect for serial access.

### 2. Test the Fixes

The firmware now includes these freeze fixes:

1. **Sensor timeout protection:** Max 10 consecutive sensor timeouts before recovery
2. **Communication error tracking:** Max 100 consecutive comm errors logged
3. **Thread wait timeouts:** 5-second timeouts on thread waits
4. **Scan request timeouts:** 10-second timeout on sensor scan requests
5. **Sensor suspension timeouts:** 1-second timeouts on sensor suspension loops
6. **Button stuck detection:** 30-second timeout on stuck button detection
7. **Concurrent scan timeout:** 5-second timeout on concurrent scan operations

### 3. Monitor for Issues

Watch the serial output for these messages:
- `"Timeout waiting for sensor scan to finish"` - sensor_scan timeout triggered
- `"consecutive sensor timeouts, forcing sensor resume"` - recovery triggered
- `"consecutive I2C/SPI/EXT errors"` - communication errors detected
- `"Button GPIO stuck"` - button stuck detection
- `"Timeout waiting for threads"` - thread wait timeout

### 4. Test Procedure

1. Power on the tracker
2. Leave it running for several hours
3. Test button SW0 responsiveness periodically
4. Check if the freeze issue is resolved
5. Monitor serial logs for timeout messages

---

## Rebuilding After Changes

If you modify the code:

```bash
cd /home/madkoding/SlimeVR-Tracker-nRF

# Quick rebuild (only changed files)
west build

# Full clean rebuild
west build -p

# Build for different board
west build -b <board_name> -p
```

Available boards:
- `xiao_ble` - Seeed XIAO BLE nRF52840
- `xiao_ble_nrf52840_sense` - Seeed XIAO BLE Sense
- `nrf52840dk_nrf52840` - Nordic nRF52840 DK
- And many others (see `boards/` directory)

---

## Troubleshooting

### USB Drive Doesn't Appear
- Try double-clicking RESET faster
- Ensure USB cable supports data transfer
- Try a different USB port
- Check cable connection

### Permission Denied on Serial Port
```bash
# Add yourself to dialout group (if not done already)
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
```

### Build Errors After Changes
```bash
# Clean and rebuild
rm -rf build
west build -b xiao_ble -p
```

### Device Not Responding
- Try resetting the device
- Re-flash the firmware
- Check battery charge
- Verify connections

---

## Files Generated

All build artifacts are in `build/SlimeVR-Tracker-nRF/zephyr/`:
- `zephyr.uf2` - Flashable firmware (UF2 format)
- `zephyr.hex` - Flashable firmware (Intel HEX format)
- `zephyr.elf` - Binary with debug symbols
- `zephyr.dts` - Compiled device tree
- `zephyr.map` - Memory map
- `.config` - Kconfig configuration

---

## Success! 🎉

Your firmware has been successfully compiled with all 7 freeze fixes applied. The tracker should now:
- ✅ Not freeze during sensor operations
- ✅ Recover from sensor timeouts automatically
- ✅ Handle communication errors gracefully
- ✅ Respond to button presses reliably
- ✅ Not get stuck in infinite loops

Good luck testing!
