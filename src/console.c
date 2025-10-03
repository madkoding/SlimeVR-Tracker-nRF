#include "globals.h"
#include "system/system.h"
#include "system/battery_tracker.h"
#include "system/power.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "connection/esb.h"
#include "build_defines.h"

#if CONFIG_USB_DEVICE_STACK
#define USB DT_NODELABEL(usbd)
#define USB_EXISTS (DT_NODE_HAS_STATUS(USB, okay) && CONFIG_UART_CONSOLE)
#endif

#if (USB_EXISTS || CONFIG_RTT_CONSOLE) && CONFIG_USE_SLIMENRF_CONSOLE

#if USB_EXISTS
#include <zephyr/console/console.h>
#include <zephyr/logging/log_ctrl.h>
#else
#include "system/rtt_console.h"
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
#if USB_EXISTS
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 1024); // TODO: larger stack size to handle reboot and print info
#else
K_THREAD_DEFINE(console_thread_id, 1024, console_thread, NULL, NULL, NULL, 6, 0, 0);
#endif

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#endif

static const char *meows[] = {
	"Mew",
	"Meww",
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"mew",
	"meww",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static const char *meow_suffixes[] = {
	" :3",
	" :3c",
	" ;3",
	" ;3c",
	" x3",
	" x3c",
	" X3",
	" X3c",
	" >:3",
	" >:3c",
	" >;3",
	" >;3c",
	""
};

void console_thread_create(void)
{
#if USB_EXISTS
	k_thread_create(&console_thread_id, console_thread_id_stack, K_THREAD_STACK_SIZEOF(console_thread_id_stack), (k_thread_entry_t)console_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
#endif
}

void console_thread_abort(void)
{
#if USB_EXISTS
	k_thread_abort(&console_thread_id);
#endif
}

// ANSI color codes
#define COLOR_RESET   "\033[0m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_BOLD    "\033[1m"

static void print_board(void)
{
	printk("\n");
	printk(COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗\n");
#if USB_EXISTS
	printk("║  " COLOR_BOLD "%-61s" COLOR_RESET COLOR_CYAN "║\n" COLOR_RESET, CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT);
#else
	printk("║  " COLOR_BOLD "%-61s" COLOR_RESET COLOR_CYAN "║\n" COLOR_RESET, "SlimeVR Tracker");
#endif
	printk(COLOR_CYAN "╠════════════════════════════════════════════════════════════════╣\n");
	
	// Print version and build info in two lines
	char version_line[64];
	char build_line[64];
	
	snprintf(version_line, sizeof(version_line), FW_NAME " " APP_VERSION_EXTENDED_STRING);
	snprintf(build_line, sizeof(build_line), "(Commit " TOSTRING(APP_BUILD_VERSION) ", Build %d-%02d-%02d %02d:%02d:%02d)",
		BUILD_YEAR, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN, BUILD_SEC);
	
	printk(COLOR_CYAN "║  " COLOR_RESET "%-62s" COLOR_CYAN "║\n", version_line);
	printk(COLOR_CYAN "║  " COLOR_RESET "%-62s" COLOR_CYAN "║\n", build_line);
	printk("╚════════════════════════════════════════════════════════════════╝\n" COLOR_RESET);

	printk("\n" COLOR_GREEN "┌─ HARDWARE INFO ────────────────────────────────────────────────┐\n");
	printk("│  " COLOR_RESET "Board:  " COLOR_YELLOW "%-51s" COLOR_RESET COLOR_GREEN " │\n", CONFIG_BOARD);
	printk("│  " COLOR_RESET "SOC:    " COLOR_YELLOW "%-51s" COLOR_RESET COLOR_GREEN " │\n", CONFIG_SOC);
	printk("│  " COLOR_RESET "Target: " COLOR_YELLOW "%-51s" COLOR_RESET COLOR_GREEN " │\n", CONFIG_BOARD_TARGET);
	printk("└────────────────────────────────────────────────────────────────┘\n" COLOR_RESET);
}

static void print_sensor(void)
{
	printk("\n" COLOR_BLUE "┌─ SENSOR CONFIGURATION ─────────────────────────────────────────┐\n");
	printk("│ " COLOR_RESET "IMU:         " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", (retained->imu_addr & 0x7F) != 0x7F ? sensor_get_sensor_imu_name() : "Not searching");
	if (retained->imu_reg != 0xFF)
		printk("│   " COLOR_RESET "Interface: " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", (retained->imu_reg & 0x80) ? "SPI" : "I2C");
	
	char addr_str[50];
	snprintf(addr_str, sizeof(addr_str), "0x%02X%02X", retained->imu_addr, retained->imu_reg);
	printk("│   " COLOR_RESET "Address:   " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", addr_str);
	
	// IMU Wake-up support
	bool wakeup_available = sys_imu_wakeup_available();
	printk("│   " COLOR_RESET "Wake-up:   " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", wakeup_available ? "Available" : "Not available");

#if SENSOR_MAG_EXISTS
	printk(COLOR_BLUE "│                                                                │\n");
	printk("│ " COLOR_RESET "Magnetometer: " COLOR_YELLOW "%-45s" COLOR_RESET COLOR_BLUE "│\n", (retained->mag_addr & 0x7F) != 0x7F ? sensor_get_sensor_mag_name() : "Not searching");
	if (retained->mag_reg != 0xFF) {
		char interface_str[50];
		snprintf(interface_str, sizeof(interface_str), "%s%s", 
			(retained->mag_reg & 0x80) ? "SPI" : "I2C", 
			(retained->mag_addr & 0x80) ? ", external" : "");
		printk("│   " COLOR_RESET "Interface: " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", interface_str);
	}
	snprintf(addr_str, sizeof(addr_str), "0x%02X%02X", retained->mag_addr, retained->mag_reg);
	printk("│   " COLOR_RESET "Address:   " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", addr_str);
#endif
	printk(COLOR_BLUE "│                                                                │\n");
	printk("│ " COLOR_RESET "Fusion:      " COLOR_YELLOW "%-47s" COLOR_RESET COLOR_BLUE "│\n", sensor_get_sensor_fusion_name());
	printk("└────────────────────────────────────────────────────────────────┘\n" COLOR_RESET);

	printk("\n" COLOR_MAGENTA "┌─ CALIBRATION DATA ─────────────────────────────────────────────┐\n");
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("│ " COLOR_RESET "Accelerometer matrix:%-38s" COLOR_MAGENTA "│\n", "");
	for (int i = 0; i < 3; i++) {
		char matrix_line[60];
		snprintf(matrix_line, sizeof(matrix_line), "%7.5f %7.5f %7.5f %7.5f", 
			(double)retained->accBAinv[0][i], (double)retained->accBAinv[1][i], 
			(double)retained->accBAinv[2][i], (double)retained->accBAinv[3][i]);
		printk("│   " COLOR_YELLOW "%-57s" COLOR_RESET COLOR_MAGENTA "│\n", matrix_line);
	}
#else
	printk("│ " COLOR_RESET "Accelerometer bias:%-41s" COLOR_MAGENTA "│\n", "");
	char accel_line[60];
	snprintf(accel_line, sizeof(accel_line), "X: %8.5f  Y: %8.5f  Z: %8.5f", 
		(double)retained->accelBias[0], (double)retained->accelBias[1], (double)retained->accelBias[2]);
	printk("│   " COLOR_YELLOW "%-57s" COLOR_RESET COLOR_MAGENTA "│\n", accel_line);
#endif
	printk(COLOR_MAGENTA "│                                                                │\n");
	printk("│ " COLOR_RESET "Gyroscope bias:%-45s" COLOR_MAGENTA "│\n", "");
	char gyro_line[60];
	snprintf(gyro_line, sizeof(gyro_line), "X: %8.5f  Y: %8.5f  Z: %8.5f", 
		(double)retained->gyroBias[0], (double)retained->gyroBias[1], (double)retained->gyroBias[2]);
	printk("│   " COLOR_YELLOW "%-57s" COLOR_RESET COLOR_MAGENTA "│\n", gyro_line);
#if SENSOR_MAG_EXISTS
	printk(COLOR_MAGENTA "│                                                                │\n");
	printk("│ " COLOR_RESET "Magnetometer matrix:%-40s" COLOR_MAGENTA "│\n", "");
	for (int i = 0; i < 3; i++) {
		char matrix_line[60];
		snprintf(matrix_line, sizeof(matrix_line), "%7.5f %7.5f %7.5f %7.5f", 
			(double)retained->magBAinv[0][i], (double)retained->magBAinv[1][i], 
			(double)retained->magBAinv[2][i], (double)retained->magBAinv[3][i]);
		printk("│   " COLOR_YELLOW "%-57s" COLOR_RESET COLOR_MAGENTA "│\n", matrix_line);
	}
#endif
	printk("└────────────────────────────────────────────────────────────────┘\n" COLOR_RESET);
}

static void print_connection(void)
{
	bool paired = retained->paired_addr[0];
	printk("\n" COLOR_CYAN "┌─ CONNECTION STATUS ────────────────────────────────────────────┐\n");
	if (paired) {
		char tracker_id[50];
		snprintf(tracker_id, sizeof(tracker_id), "%u", retained->paired_addr[1]);
		printk("│ " COLOR_RESET "Tracker ID:       " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_CYAN "│\n", tracker_id);
	} else {
		printk("│ " COLOR_RESET "Tracker ID:       " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_CYAN "│\n", "Not paired");
	}
	
	char device_addr[50];
	snprintf(device_addr, sizeof(device_addr), "%012llX", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
	printk("│ " COLOR_RESET "Device address:   " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_CYAN "│\n", device_addr);
	
	if (paired) {
		char receiver_addr[50];
		snprintf(receiver_addr, sizeof(receiver_addr), "%012llX", (*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);
		printk("│ " COLOR_RESET "Receiver address: " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_CYAN "│\n", receiver_addr);
	} else {
		printk("│ " COLOR_RESET "Receiver address: " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_CYAN "│\n", "None");
	}
	printk("└────────────────────────────────────────────────────────────────┘\n" COLOR_RESET);
}

static void print_battery(void)
{
	int battery_mV = sys_get_valid_battery_mV();
	int16_t calibrated_pptt = sys_get_calibrated_battery_pptt(sys_get_valid_battery_pptt());
	uint64_t unplugged_time = sys_get_last_unplugged_time();
	uint64_t remaining = sys_get_battery_remaining_time_estimate();
	uint64_t runtime = sys_get_battery_runtime_estimate();
	
	printk("\n" COLOR_GREEN "┌─ BATTERY STATUS ───────────────────────────────────────────────┐\n");
	
	if (battery_mV > 0)
	{
		unplugged_time = k_ticks_to_us_floor64(k_uptime_ticks() - unplugged_time);
		uint32_t hours = unplugged_time / 3600000000;
		unplugged_time %= 3600000000;
		uint8_t minutes = unplugged_time / 60000000;
		if (hours > 0 || minutes > 0) {
			char battery_str[50];
			snprintf(battery_str, sizeof(battery_str), "%.0f%% (Read %uh %umin ago)", 
				(double)calibrated_pptt / 100.0, hours, minutes);
			printk("│ " COLOR_RESET "Battery level:    " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", battery_str);
		} else {
			char battery_str[50];
			snprintf(battery_str, sizeof(battery_str), "%.0f%%", (double)calibrated_pptt / 100.0);
			printk("│ " COLOR_RESET "Battery level:    " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", battery_str);
		}
	}
	else if (unplugged_time == 0)
	{
		printk("│ " COLOR_RESET "Battery level:    " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", "Waiting for valid reading");
	}
	else
	{
		printk("│ " COLOR_RESET "Battery level:    " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", "None");
	}
	
	if (remaining > 0)
	{
		remaining = k_ticks_to_us_floor64(remaining);
		uint32_t hours = remaining / 3600000000;
		remaining %= 3600000000;
		uint8_t minutes = remaining / 60000000;
		char time_str[50];
		snprintf(time_str, sizeof(time_str), "%uh %umin", hours, minutes);
		printk("│ " COLOR_RESET "Remaining time:   " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", time_str);
	}
	else
	{
		printk("│ " COLOR_RESET "Remaining time:   " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", "Not available");
	}
	
	if (runtime > 0)
	{
		runtime = k_ticks_to_us_floor64(runtime);
		uint32_t hours = runtime / 3600000000;
		runtime %= 3600000000;
		uint8_t minutes = runtime / 60000000;
		char time_str[50];
		snprintf(time_str, sizeof(time_str), "%uh %umin", hours, minutes);
		printk("│ " COLOR_RESET "Full charge time: " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", time_str);
	}
	else
	{
		printk("│ " COLOR_RESET "Full charge time: " COLOR_YELLOW "%-42s" COLOR_RESET COLOR_GREEN "│\n", "Not available");
	}
	
	printk("└────────────────────────────────────────────────────────────────┘\n" COLOR_RESET);
}

static void print_info(void)
{
	print_board();
	print_sensor();
	print_connection();
	print_battery();
	printk("\n");
}

static void print_uptime(const uint64_t ticks, const char *name)
{
	uint64_t uptime = k_ticks_to_us_floor64(ticks);

	uint32_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime % 1000;

	printk("%s: %02u:%02u:%02u.%03u,%03u\n", name, hours, minutes, seconds, milliseconds, microseconds);
}

static void print_battery_tracker(void)
{
	int adc_mV = sys_get_battery_mV();
	printk("ADC: %d mV\n", adc_mV);

	int battery_mV = sys_get_valid_battery_mV();
	int16_t pptt = sys_get_valid_battery_pptt();
	int16_t calibrated_pptt = sys_get_calibrated_battery_pptt(pptt);
	uint64_t unplugged_time = sys_get_last_unplugged_time();
	if (battery_mV > 0)
		printk("\nBattery: %.2f%% (Raw %.2f%%, %d mV)\n", (double)calibrated_pptt / 100.0, (double)pptt / 100.0, battery_mV);
	else
		printk("\nBattery: None\n");
	if (unplugged_time > 0)
		print_uptime(k_uptime_ticks() - unplugged_time, "Last updated");
	else
		printk("Last updated: Never\n");

	uint64_t runtime = sys_get_battery_runtime_estimate();
	uint64_t runtime_min = sys_get_battery_runtime_min_estimate();
	uint64_t runtime_max = sys_get_battery_runtime_max_estimate();
	uint64_t remaining = sys_get_battery_remaining_time_estimate();
	if (remaining > 0)
		print_uptime(remaining, "\nRemaining runtime");
	else
		printk("Remaining runtime: Not available\n");
	if (runtime > 0)
		print_uptime(runtime, "Fully charged runtime");
	else
		printk("Fully charged runtime: Not available\n");
	if (runtime_min > 0)
		print_uptime(runtime_min, "Minimum runtime");
	else
		printk("Minimum runtime: Not available\n");
	if (runtime_max > 0)
		print_uptime(runtime_max, "Maximum runtime");
	else
		printk("Maximum runtime: Not available\n");

	int16_t last_min = sys_get_last_cycle_min_pptt();
	int16_t last_max = sys_get_last_cycle_max_pptt();
	int16_t last_calibrated_min = sys_get_calibrated_battery_pptt(last_min);
	int16_t last_calibrated_max = sys_get_calibrated_battery_pptt(last_max);
	uint64_t last_runtime = sys_get_last_cycle_runtime();
	if (last_min >= 0 && last_max >= 0 && last_runtime > 0)
	{
		printk("\nLast discharge cycle: %.2f%% -> %.2f%% (Raw %.2f%% -> %.2f%%)\n", (double)last_calibrated_max / 100.0, (double)last_calibrated_min / 100.0, (double)last_max / 100.0, (double)last_min / 100.0);
		print_uptime(last_runtime, "Last cycle runtime");
	}
	else
	{
		printk("\nLast cycle: Not available\n");
	}

	float coverage = sys_get_battery_calibration_coverage();
	int16_t min = sys_get_calibrated_battery_range_min_pptt();
	int16_t max = sys_get_calibrated_battery_range_max_pptt();
	if (min >= 0 && max >= 0)
		printk("\nCalibration: %.0f%% - %.0f%% (%.0f%% coverage)\n", (double)min / 100.0, (double)max / 100.0, (double)coverage * 100.0);
	else
		printk("\nCalibration: None\n");
	printk("Cycle count: ~%.2f\n", (double)sys_get_battery_cycles());
}

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void console_thread(void)
{
#if USB_EXISTS && DFU_EXISTS
	if (button_read()) // button held on usb connect, enter DFU
	{
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = 0x57;
		sys_request_system_reboot();
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
	}
#endif

#if USB_EXISTS
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
#endif
	printk(FW_STRING);
	printk("info                         Get device information\n");
	printk("uptime                       Get device uptime\n");
	printk("reboot                       Soft reset the device\n");
	printk("battery                      Get battery information\n");
	printk("scan                         Restart sensor scan\n");
	printk("calibrate                    Calibrate sensor ZRO\n");

	uint8_t command_info[] = "info";
	uint8_t command_uptime[] = "uptime";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_battery[] = "battery";
	uint8_t command_scan[] = "scan";
	uint8_t command_calibrate[] = "calibrate";

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	printk("6-side                       Calibrate 6-side accelerometer\n");

	uint8_t command_6_side[] = "6-side";
#endif

#if SENSOR_MAG_EXISTS
	printk("mag                          Clear magnetometer calibration\n");

	uint8_t command_mag[] = "mag";
#endif

	printk("set <address>                Manually set receiver\n");
	printk("pair                         Enter pairing mode\n");
	printk("clear                        Clear pairing data\n");

	uint8_t command_set[] = "set";
	uint8_t command_pair[] = "pair";
	uint8_t command_clear[] = "clear";

#if DFU_EXISTS
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_dfu[] = "dfu";
#endif

	printk("meow                         Meow!\n");

	uint8_t command_meow[] = "meow";

	// debug
	uint8_t command_reset[] = "reset";
	uint8_t command_reset_arg_zro[] = "zro";
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	uint8_t command_reset_arg_acc[] = "acc";
#endif
#if SENSOR_MAG_EXISTS
	uint8_t command_reset_arg_mag[] = "mag";
#endif
	uint8_t command_reset_arg_bat[] = "bat";
	uint8_t command_reset_arg_all[] = "all";

	while (1) {
#if USB_EXISTS
		uint8_t *line = console_getline();
#else
		uint8_t *line = rtt_console_getline();
#endif
		uint8_t *arg = NULL;
		for (uint8_t *p = line; *p; ++p)
		{
			*p = tolower(*p);
			if (*p == ' ' && !arg)
			{
				*p = 0;
				p++;
				*p = tolower(*p);
				if (*p)
					arg = p;
			}
		}

		if (memcmp(line, command_info, sizeof(command_info)) == 0)
		{
			print_info();
		}
		else if (memcmp(line, command_uptime, sizeof(command_uptime)) == 0)
		{
			uint64_t uptime = k_uptime_ticks();
			print_uptime(uptime, "Uptime");
			print_uptime(uptime - retained->uptime_latest + retained->uptime_sum, "Accumulated");
		}
		else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
		{
			sys_request_system_reboot();
		}
		else if (memcmp(line, command_battery, sizeof(command_battery)) == 0)
		{
			print_battery_tracker();
		}
		else if (memcmp(line, command_scan, sizeof(command_scan)) == 0)
		{
			sensor_request_scan(true);
		}
		else if (memcmp(line, command_calibrate, sizeof(command_calibrate)) == 0)
		{
			sensor_request_calibration();
		}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		else if (memcmp(line, command_6_side, sizeof(command_6_side)) == 0)
		{
			sensor_request_calibration_6_side();
		}
#endif
#if SENSOR_MAG_EXISTS
		else if (memcmp(line, command_mag, sizeof(command_mag)) == 0)
		{
			sensor_calibration_clear_mag(NULL, true);
		}
#endif
		else if (memcmp(line, command_set, sizeof(command_set)) == 0) 
		{
			uint64_t addr = strtoull(arg, NULL, 16);
			uint8_t buf[17];
			snprintk(buf, 17, "%016llx", addr);
			if (addr != 0 && memcmp(buf, arg, 17) == 0)
				esb_set_pair(addr);
			else
				printk("Invalid address\n");
		}
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) 
		{
			esb_reset_pair();
		}
		else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) 
		{
			esb_clear_pair();
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
#if ADAFRUIT_BOOTLOADER
			NRF_POWER->GPREGRET = 0x57;
			sys_request_system_reboot();
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) 
		{
			print_meow();
		}
		else if (memcmp(line, command_reset, sizeof(command_reset)) == 0)
		{
			if (arg && memcmp(arg, command_reset_arg_zro, sizeof(command_reset_arg_zro)) == 0)
			{
				sensor_calibration_clear(NULL, NULL, true);
			}
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
			else if (arg && memcmp(arg, command_reset_arg_acc, sizeof(command_reset_arg_acc)) == 0)
			{
				sensor_calibration_clear_6_side(NULL, true);
			}
#endif
#if SENSOR_MAG_EXISTS
			else if (arg && memcmp(arg, command_reset_arg_mag, sizeof(command_reset_arg_mag)) == 0)
			{
				sensor_calibration_clear_mag(NULL, true);
			}
#endif
			else if (arg && memcmp(arg, command_reset_arg_bat, sizeof(command_reset_arg_bat)) == 0)
			{
				sys_reset_battery_tracker();
			}
			else if (arg && memcmp(arg, command_reset_arg_all, sizeof(command_reset_arg_all)) == 0)
			{
				sys_clear();
			}
			else
			{
				printk("Invalid argument\n");
			}
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif