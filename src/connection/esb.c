/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "globals.h"
#include "system/system.h"
#include "connection.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/sys/crc.h>

#include "esb.h"

// =============================================================================
// ESB Configuration Constants
// =============================================================================

#define ESB_RETRANSMIT_DELAY_US     1000  // Delay between retransmission attempts
#define ESB_RETRANSMIT_COUNT        5     // Number of retransmission attempts
#define ESB_DEINIT_DELAY_MS         10    // Wait time before disabling ESB
#define ESB_PAIR_TX_DELAY_MS        2     // Delay between pairing packet transmissions
#define ESB_PAIR_CYCLE_DELAY_MS     996   // Delay between full pairing cycles
#define ESB_PAIR_COMPLETE_DELAY_MS  1600  // Wait time after pairing completes
#define ESB_THREAD_LOOP_TIMEOUT_MS  100   // Main thread loop timeout
#define CLOCK_FETCH_MAX_ATTEMPTS    10    // Maximum attempts to fetch clock result
#define TX_ERROR_THRESHOLD          100   // Consecutive TX errors before connection error

// Pairing packet indices
#define PAIR_PACKET_CHECKSUM_IDX    0
#define PAIR_PACKET_STATE_IDX       1
#define PAIR_PACKET_ADDR_START_IDX  2
#define PAIR_PACKET_ADDR_LENGTH     6

// Pairing states
#define PAIR_STATE_REQUEST          0
#define PAIR_STATE_ACK_DATA         1
#define PAIR_STATE_ACKNOWLEDGE      2

// =============================================================================
// Global Variables
// =============================================================================

// Semaphore to wake ESB thread when events occur
K_SEM_DEFINE(esb_event_sem, 0, 1);

// State tracking
static bool esb_initialized = false;
static bool esb_paired = false;
static bool clock_status = false;
static bool timer_state = false;
static bool send_data = false;

// Error tracking
static uint32_t tx_errors = 0;
static int64_t last_tx_success = 0;

// Timer/clock synchronization
static uint8_t last_reset = 0;
static uint16_t led_clock = 0;
static uint32_t led_clock_offset = 0;

// ESB payload structures
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
	0, 0, 0, 0, 0, 0, 0, 0);

// Pairing address storage (8 bytes)
static uint8_t paired_addr[8] = {0};

// Discovery mode addresses (randomly generated)
// TODO: Verify purpose and relationship to CONFIG_ESB_PIPE_COUNT
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

// Active address configuration
static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

// =============================================================================
// Logging
// =============================================================================

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

// =============================================================================
// Thread Definitions
// =============================================================================

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 512, esb_thread, NULL, NULL, NULL,
                ESB_THREAD_PRIORITY, 0, 0);

// =============================================================================
// Clock Control Functions
// =============================================================================

#if defined(CONFIG_CLOCK_CONTROL_NRF)

static struct onoff_manager *clk_mgr;

/**
 * Initialize the clock control manager
 * Called during system initialization
 */
static int clocks_init(void)
{
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENOTSUP;
	}

	return 0;
}

SYS_INIT(clocks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/**
 * Start the high-frequency clock
 * Required for radio operations
 *
 * @return 0 on success, negative error code on failure
 */
int clocks_start(void)
{
	if (clock_status) {
		return 0; // Already started
	}

	int err;
	int res;
	struct onoff_client clk_cli;

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	// Attempt to fetch result with exponential backoff
	err = sys_notify_fetch_result(&clk_cli.notify, &res);
	for (int attempt = 1; err && attempt <= CLOCK_FETCH_MAX_ATTEMPTS; attempt++) {
		k_usleep(100 * attempt); // Backoff: 100us, 200us, 300us...
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	}

	if (err) {
		LOG_WRN_ONCE("Unable to fetch Clock request result: %d", err);
		return err;
	}

#if defined(NRF54L15_XXAA)
	// Workaround for MLTPAN-20 errata
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_DBG("HF clock started");
	clock_status = true;
	return 0;
}

/**
 * Stop the high-frequency clock to save power
 */
void clocks_stop(void)
{
	if (!clock_status) {
		return; // Already stopped
	}

	clock_status = false;
	onoff_release(clk_mgr);
	LOG_DBG("HF clock stop request");
}

#else
BUILD_ASSERT(false, "No Clock Control driver available");
#endif

// =============================================================================
// Clock Control Thread Helpers
// =============================================================================

static struct k_thread clocks_thread_id;
static K_THREAD_STACK_DEFINE(clocks_thread_id_stack, 128);

/**
 * Request clock start in a separate thread after a delay
 *
 * @param delay_us Delay in microseconds before starting clock
 */
void clocks_request_start(uint32_t delay_us)
{
	k_thread_create(&clocks_thread_id, clocks_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_thread_id_stack),
		(k_thread_entry_t)clocks_start, NULL, NULL, NULL,
		CLOCKS_START_THREAD_PRIORITY, 0, K_USEC(delay_us));
}

static struct k_thread clocks_stop_thread_id;
static K_THREAD_STACK_DEFINE(clocks_stop_thread_id_stack, 128);

/**
 * Request clock stop in a separate thread after a delay
 *
 * @param delay_us Delay in microseconds before stopping clock
 */
void clocks_request_stop(uint32_t delay_us)
{
	k_thread_create(&clocks_stop_thread_id, clocks_stop_thread_id_stack,
		K_THREAD_STACK_SIZEOF(clocks_stop_thread_id),
		(k_thread_entry_t)clocks_stop, NULL, NULL, NULL,
		CLOCKS_STOP_THREAD_PRIORITY, 0, K_USEC(delay_us));
}

// =============================================================================
// ESB Event Handler
// =============================================================================

/**
 * Handle ESB radio events
 * Called from interrupt context
 *
 * @param event ESB event structure containing event type and data
 */
void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
		case ESB_EVENT_TX_SUCCESS:
			// Transmission successful - reset error counter
			tx_errors = 0;
			if (esb_paired) {
				clocks_stop();
			}
			k_sem_give(&esb_event_sem);
			break;

		case ESB_EVENT_TX_FAILED:
			// Track consecutive transmission failures
			if (tx_errors < UINT32_MAX) {
				tx_errors++;
			}

			// Record timestamp when error threshold is reached
			if (tx_errors == TX_ERROR_THRESHOLD) {
				last_tx_success = k_uptime_get();
			}

			LOG_DBG("TX FAILED");

			if (esb_paired) {
				clocks_stop();
			}
			k_sem_give(&esb_event_sem);
			break;

		case ESB_EVENT_RX_RECEIVED:
			// TODO: Read rx buffer until -ENODATA (or -EACCES/-EINVAL)
			if (esb_read_rx_payload(&rx_payload) == 0) { // Success

				if (!paired_addr[0]) {
					// Device not paired - handle pairing acknowledgment
					LOG_DBG("tx: %16llX rx: %16llX",
						*(uint64_t *)tx_payload_pair.data,
						*(uint64_t *)rx_payload.data);

					// Check if this is ACK to second packet in pairing burst
					if (rx_payload.length == 8 &&
					    tx_payload_pair.data[PAIR_PACKET_STATE_IDX] == PAIR_STATE_ACK_DATA) {
						memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
					}
				} else {
					// Device already paired - handle synchronization packet
					if (rx_payload.length == 4) {
						// TODO: Why do we receive packets when already paired?
						// This may be part of acknowledge protocol

						// Ensure timer is initialized
						if (timer_state == false) {
							timer_state = true;
							LOG_WRN("Timer not initialized");
							break;
						}

						// Reset timer and synchronize LED clock
						last_reset = 0;
						led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1];
						led_clock_offset = 0;
						LOG_DBG("RX, timer reset");
					}
				}
			}
			break;
	}
}

// =============================================================================
// ESB Initialization and Control
// =============================================================================

/**
 * Initialize ESB (Enhanced ShockBurst) protocol
 *
 * @param tx true for transmitter mode, false for receiver mode
 * @return 0 on success, negative error code on failure
 */
int esb_initialize(bool tx)
{
	int err;
	struct esb_config config = ESB_DEFAULT_CONFIG;

	// Common configuration for both TX and RX
	config.event_handler = event_handler;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.crc = ESB_CRC_16BIT;
	config.tx_output_power = CONFIG_RADIO_TX_POWER;
	config.retransmit_delay = ESB_RETRANSMIT_DELAY_US;
	config.retransmit_count = ESB_RETRANSMIT_COUNT;
	config.selective_auto_ack = true;

	// Set receiver mode if not transmitter
	if (!tx) {
		config.mode = ESB_MODE_PRX;
	}

	err = esb_init(&config);
	if (err) {
		goto error;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		goto error;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		goto error;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		goto error;
	}

	esb_initialized = true;
	return 0;

error:
	LOG_ERR("ESB initialization failed: %d", err);
	set_status(SYS_STATUS_CONNECTION_ERROR, true);
	return err;
}

/**
 * Deinitialize ESB and clean up resources
 */
void esb_deinitialize(void)
{
	if (esb_initialized) {
		esb_initialized = false;
		// Wait for pending transmissions to complete
		k_msleep(ESB_DEINIT_DELAY_MS);
		esb_disable();
	}
	esb_initialized = false;
}

// =============================================================================
// Address Configuration Functions
// =============================================================================

/**
 * Configure ESB to use discovery mode addresses
 * Used during pairing to find available receivers
 */
inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

/**
 * Configure ESB to use paired device addresses
 * Generates unique addresses based on pairing data
 */
inline void esb_set_addr_paired(void)
{
	uint8_t addr_buffer[16] = {0};

	// Recreate receiver address from paired data
	for (int i = 0; i < 4; i++) {
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}

	for (int i = 0; i < 8; i++) {
		addr_buffer[i + 8] = paired_addr[7] + i;
	}

	// Avoid invalid addresses (see nRF datasheet)
	for (int i = 0; i < 16; i++) {
		if (addr_buffer[i] == 0x00 ||
		    addr_buffer[i] == 0x55 ||
		    addr_buffer[i] == 0xAA) {
			addr_buffer[i] += 8;
		}
	}

	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

// =============================================================================
// Pairing Functions
// =============================================================================

/**
 * Calculate and validate checksum for device address
 *
 * @return Checksum value (never 0, offset to 8 if calculated as 0)
 */
static uint8_t calculate_device_checksum(void)
{
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	uint8_t buf[PAIR_PACKET_ADDR_LENGTH] = {0};

	memcpy(buf, device_addr, PAIR_PACKET_ADDR_LENGTH);
	uint8_t checksum = crc8_ccitt(0x07, buf, PAIR_PACKET_ADDR_LENGTH);

	// Ensure checksum is never 0
	if (checksum == 0) {
		checksum = 8;
	}

	return checksum;
}

/**
 * Set paired receiver address
 * Validates checksum before accepting pairing
 *
 * @param addr 64-bit pairing address received from receiver
 */
void esb_set_pair(uint64_t addr)
{
	uint8_t checksum = calculate_device_checksum();

	// Validate checksum matches
	if ((addr & 0xFF) != checksum) {
		LOG_INF("Incorrect checksum");
		return;
	}

	esb_reset_pair();
	memcpy(paired_addr, &addr, sizeof(paired_addr));
	LOG_INF("Paired");

	// Persist pairing data
	sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr));
}

/**
 * Prepare pairing payload with device address and checksum
 */
static void prepare_pairing_payload(void)
{
	uint64_t *device_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	uint8_t checksum = calculate_device_checksum();

	tx_payload_pair.noack = false;

	// Copy device address to payload
	memcpy(&tx_payload_pair.data[PAIR_PACKET_ADDR_START_IDX],
	       device_addr,
	       PAIR_PACKET_ADDR_LENGTH);

	LOG_INF("Device address: %012llX", *device_addr & 0xFFFFFFFFFFFF);
	LOG_INF("Checksum: %02X", checksum);

	// Store checksum to validate received packets
	tx_payload_pair.data[PAIR_PACKET_CHECKSUM_IDX] = checksum;
}

/**
 * Execute one pairing transmission cycle
 * Sends three packets: request, ack data, and acknowledge
 */
static void execute_pairing_cycle(void)
{
	esb_flush_rx();
	esb_flush_tx();

	// Packet 1: Send pairing request
	tx_payload_pair.data[PAIR_PACKET_STATE_IDX] = PAIR_STATE_REQUEST;
	esb_write_payload(&tx_payload_pair);
	esb_start_tx();
	k_msleep(ESB_PAIR_TX_DELAY_MS);

	// Packet 2: Receive acknowledgment data
	tx_payload_pair.data[PAIR_PACKET_STATE_IDX] = PAIR_STATE_ACK_DATA;
	esb_write_payload(&tx_payload_pair);
	esb_start_tx();
	k_msleep(ESB_PAIR_TX_DELAY_MS);

	// Packet 3: Acknowledge pairing from receiver
	tx_payload_pair.data[PAIR_PACKET_STATE_IDX] = PAIR_STATE_ACKNOWLEDGE;
	esb_write_payload(&tx_payload_pair);
	esb_start_tx();
	k_msleep(ESB_PAIR_CYCLE_DELAY_MS);
}

/**
 * Execute complete pairing process with receiver
 * Continuously attempts pairing until successful
 */
void esb_pair(void)
{
	// Clear any existing connection errors
	if (tx_errors >= TX_ERROR_THRESHOLD) {
		set_status(SYS_STATUS_CONNECTION_ERROR, false);
	}
	tx_errors = 0;

	// Check if device needs pairing
	if (!paired_addr[0]) {
		LOG_INF("Pairing");

		esb_set_addr_discovery();
		esb_initialize(true);

		prepare_pairing_payload();

		uint8_t expected_checksum = tx_payload_pair.data[PAIR_PACKET_CHECKSUM_IDX];
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);

		// Continue pairing attempts until successful
		while (paired_addr[0] != expected_checksum) {

			// Ensure ESB is initialized
			if (!esb_initialized) {
				esb_set_addr_discovery();
				esb_initialize(true);
			}

			// Ensure clock is running
			if (!clock_status) {
				clocks_start();
			}

			// Validate received checksum
			if (paired_addr[0]) {
				LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}

			execute_pairing_cycle();
		}

		// Pairing successful
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CONNECTION);
		LOG_INF("Paired");

		// Persist pairing data
		sys_write(PAIRED_ID, retained->paired_addr, paired_addr, sizeof(paired_addr));

		esb_deinitialize();
		k_msleep(ESB_PAIR_COMPLETE_DELAY_MS); // Wait for LED pattern to complete
	}

	LOG_INF("Tracker ID: %u", paired_addr[1]);
	LOG_INF("Receiver address: %012llX",
		(*(uint64_t *)&retained->paired_addr[0] >> 16) & 0xFFFFFFFFFFFF);

	connection_set_id(paired_addr[1]);

	esb_set_addr_paired();
	esb_paired = true;
	clocks_stop();
}

/**
 * Reset pairing state without clearing persistent storage
 * Used when initiating a new pairing
 */
void esb_reset_pair(void)
{
	if (paired_addr[0] || esb_paired) {
		esb_deinitialize(); // Ensure ESB is off
		esb_paired = false;
		memset(paired_addr, 0, sizeof(paired_addr));
		LOG_INF("Pairing requested");
	}
}

/**
 * Clear all pairing data including persistent storage
 * Forces device into pairing mode on next startup
 */
void esb_clear_pair(void)
{
	esb_reset_pair();
	// Write zeroes to persistent storage
	sys_write(PAIRED_ID, &retained->paired_addr, paired_addr, sizeof(paired_addr));
	LOG_INF("Pairing data reset");
}

// =============================================================================
// Data Transmission
// =============================================================================

/**
 * Write data packet to ESB transmit queue
 *
 * @param data Pointer to data buffer to transmit
 */
void esb_write(uint8_t *data)
{
	if (!esb_initialized || !esb_paired) {
		return; // Not ready to transmit
	}

	// Ensure clock is running for transmission
	if (!clock_status) {
		clocks_start();
	}

	tx_payload.noack = true;

	memcpy(tx_payload.data, data, tx_payload.length);

	// Clear any pending transmissions
	esb_flush_tx();

	// Add new transmission to queue
	esb_write_payload(&tx_payload);
	send_data = true;
}

/**
 * Check if ESB is ready for data transmission
 *
 * @return true if initialized and paired, false otherwise
 */
bool esb_ready(void)
{
	return esb_initialized && esb_paired;
}

// =============================================================================
// Main ESB Thread
// =============================================================================

/**
 * Main ESB management thread
 * Handles pairing, connection monitoring, and error recovery
 */
static void esb_thread(void)
{
#if CONFIG_CONNECTION_OVER_HID
	int64_t start_time = k_uptime_get();
#endif

	// Restore paired address from retained memory
	memcpy(paired_addr, retained->paired_addr, sizeof(paired_addr));

	while (1) {
		// Check if pairing is needed
#if CONFIG_CONNECTION_OVER_HID
		// Only auto-pair if not potentially communicating via USB
		bool should_pair = !esb_paired &&
		                   !get_status(SYS_STATUS_USB_CONNECTED) &&
		                   (k_uptime_get() - 750 > start_time);
#else
		bool should_pair = !esb_paired;
#endif

		if (should_pair) {
			esb_pair();
			esb_initialize(true);
		}

		// Handle connection errors
		if (tx_errors >= TX_ERROR_THRESHOLD) {
#if CONFIG_CONNECTION_OVER_HID
			// Only raise error if not potentially communicating via USB
			bool should_raise_error = !get_status(SYS_STATUS_CONNECTION_ERROR) &&
			                          !get_status(SYS_STATUS_USB_CONNECTED);
#else
			bool should_raise_error = !get_status(SYS_STATUS_CONNECTION_ERROR);
#endif

			if (should_raise_error) {
				set_status(SYS_STATUS_CONNECTION_ERROR, true);
			}

#if USER_SHUTDOWN_ENABLED
			// Shutdown if receiver not detected for extended period
			// TODO: Is shutdown necessary if USB is connected?
			if (k_uptime_get() - last_tx_success > CONFIG_CONNECTION_TIMEOUT_DELAY) {
				LOG_WRN("No response from receiver in %dm",
					CONFIG_CONNECTION_TIMEOUT_DELAY / 60000);
				sys_request_system_off(false);
			}
#endif
		}
		// Clear error status when connection is restored
		else if (tx_errors == 0 && get_status(SYS_STATUS_CONNECTION_ERROR)) {
			set_status(SYS_STATUS_CONNECTION_ERROR, false);
		}

		// Wait for events or timeout
		k_sem_take(&esb_event_sem, K_MSEC(ESB_THREAD_LOOP_TIMEOUT_MS));
	}
}
