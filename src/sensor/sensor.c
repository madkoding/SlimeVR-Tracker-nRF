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
#include "sensor.h"

#include <math.h>

#include "calibration.h"
#include "connection/connection.h"
#include "fusion/fusions.h"
#include "globals.h"
#include "sensors.h"
#include "system/system.h"
#include "util.h"

#define SPI_OP SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8)

// Thresholds for dance filtering
#define MAG_NORM_MIN 0.10f
#define MAG_NORM_MAX 1.20f
#define MAG_MAX_GYRO_DPS 800.0f
#define MAG_MAX_ADEV_G 0.40f

#define EMA_GYRO_ALPHA_MIN 0.20f
#define EMA_GYRO_ALPHA_MAX 0.85f
#define EMA_GYRO_K 0.0005f  // gain vs |g|
#define EMA_GYRO_STEP_MAX 900.0f  // dps/frame

#define EMA_ACCEL_ALPHA_MIN 0.20f
#define EMA_ACCEL_ALPHA_MAX 0.50f
#define EMA_ACCEL_STEP_MAX 6.0f  // g/frame

static inline void quat_normalize_inline(float q[4]) {
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n > 0.0f) {
		q[0] /= n;
		q[1] /= n;
		q[2] /= n;
		q[3] /= n;
	}
}

static void quat_smooth_lerp(
	const float qPrev[4],
	const float qNew[4],
	float alpha,
	float qOut[4]
) {
	float a[4] = {qPrev[0], qPrev[1], qPrev[2], qPrev[3]};
	float b[4] = {qNew[0], qNew[1], qNew[2], qNew[3]};
	quat_normalize_inline(a);
	quat_normalize_inline(b);
	float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
	if (dot < 0.0f) {
		b[0] = -b[0];
		b[1] = -b[1];
		b[2] = -b[2];
		b[3] = -b[3];
	}
	qOut[0] = (1.0f - alpha) * a[0] + alpha * b[0];
	qOut[1] = (1.0f - alpha) * a[1] + alpha * b[1];
	qOut[2] = (1.0f - alpha) * a[2] + alpha * b[2];
	qOut[3] = (1.0f - alpha) * a[3] + alpha * b[3];
	quat_normalize_inline(qOut);
}

static float q_smooth_prev[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// State for EMA initialization
static bool ema_g_inited = false;
static bool ema_a_inited = false;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu_spi), okay)
#define SENSOR_IMU_SPI_EXISTS true
#define SENSOR_IMU_SPI_NODE DT_NODELABEL(imu_spi)
static struct spi_dt_spec sensor_imu_spi_dev
	= SPI_DT_SPEC_GET(SENSOR_IMU_SPI_NODE, SPI_OP, 0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(imu), okay)
#define SENSOR_IMU_EXISTS true
#define SENSOR_IMU_NODE DT_NODELABEL(imu)
static struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
#else
static struct i2c_dt_spec sensor_imu_dev = {0};
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
#error "IMU node does not exist"
#endif
static uint8_t sensor_imu_dev_reg = 0xFF;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag_spi), okay)
#define SENSOR_MAG_SPI_EXISTS true
#define SENSOR_MAG_SPI_NODE DT_NODELABEL(mag_spi)
static struct spi_dt_spec sensor_mag_spi_dev
	= SPI_DT_SPEC_GET(SENSOR_MAG_SPI_NODE, SPI_OP, 0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mag), okay)
#define SENSOR_MAG_EXISTS true
#define SENSOR_MAG_NODE DT_NODELABEL(mag)
static struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);
#else
static struct i2c_dt_spec sensor_mag_dev = {0};
#endif
#if SENSOR_IMU_SPI_EXISTS  // might exist
#define SENSOR_MAG_EXT_EXISTS true
#endif
#if !SENSOR_MAG_SPI_EXISTS && !SENSOR_MAG_EXISTS && !SENSOR_MAG_EXT_EXISTS
#warning "Magnetometer node does not exist"
#endif
static uint8_t sensor_mag_dev_reg = 0xFF;

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
static float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion

static float ema_a_prev[3] = {0.0f, 0.0f, 0.0f};
static float ema_g_prev[3] = {0.0f, 0.0f, 0.0f};

static float q3[4] = {SENSOR_QUATERNION_CORRECTION};  // correction quaternion

static float last_lin_a[3] = {0};  // vector to hold last linear accelerometer

static int64_t last_suspend_attempt_time = 0;
static int64_t last_data_time;
static int64_t last_info_time;
static int64_t last_mag_time;

static float max_gyro_speed_square;
static bool mag_use_oneshot;
static bool mag_skip_oneshot;

static float accel_actual_time;
static float gyro_actual_time;
static float mag_actual_time;

static bool sensor_fusion_init;
static bool sensor_sensor_init;

static bool sensor_sensor_scanning;

static bool main_suspended;

static bool mag_available;
#if MAG_ENABLED
static bool mag_enabled = true;  // TODO: toggle from server
#else
static bool mag_enabled = false;
#endif

// Temperature compensation variables
static float temp_ref = 25.0f;  // Reference temperature (°C)
static float temp_current = 25.0f;  // Current temperature
static float gyro_temp_coeff[3]
	= {0.0f, 0.0f, 0.0f};  // Temperature coefficients (dps/°C)
static bool temp_calibration_active = false;

#if CONFIG_SENSOR_USE_XIOFUSION
static const sensor_fusion_t* sensor_fusion
	= &sensor_fusion_fusion;  // TODO: change from server
int fusion_id = FUSION_FUSION;
#elif CONFIG_SENSOR_USE_NXPSENSORFUSION
static const sensor_fusion_t* sensor_fusion
	= &sensor_fusion_motionsense;  // TODO: change from server
int fusion_id = FUSION_MOTIONSENSE;
#elif CONFIG_SENSOR_USE_VQF
static const sensor_fusion_t* sensor_fusion
	= &sensor_fusion_vqf;  // TODO: change from server
int fusion_id = FUSION_VQF;
#endif

static int sensor_imu_id = -1;
static int sensor_mag_id = -1;
static const sensor_imu_t* sensor_imu = &sensor_imu_none;
static const sensor_mag_t* sensor_mag = &sensor_mag_none;

// Temperature compensation function
static void apply_temperature_compensation(float temp, float gyro_raw[3]) {
	temp_current = temp;

	if (!temp_calibration_active) {
		return;  // Skip if temperature calibration is not active
	}

	// Apply linear temperature compensation: bias(T) = bias(Tref) + coeff * (T - Tref)
	float temp_delta = temp - temp_ref;
	for (int i = 0; i < 3; i++) {
		gyro_raw[i] -= gyro_temp_coeff[i] * temp_delta;
	}
}

// Function to update temperature coefficients (call during calibration)
static void update_temperature_calibration(float temp, const float gyro_bias[3]) {
	// Simple method: estimate temperature coefficient from current bias vs reference
	if (fabsf(temp - temp_ref)
		> 5.0f) {  // Only update if significant temperature difference
		float temp_delta = temp - temp_ref;
		for (int i = 0; i < 3; i++) {
			// Simple running average of temperature coefficient
			float coeff_new = gyro_bias[i] / temp_delta;
			gyro_temp_coeff[i]
				= gyro_temp_coeff[i] * 0.9f + coeff_new * 0.1f;  // EMA filter
		}
		temp_calibration_active = true;
		LOG_DBG(
			"Updated gyro temp coefficients: %.3f %.3f %.3f dps/°C",
			(double)gyro_temp_coeff[0],
			(double)gyro_temp_coeff[1],
			(double)gyro_temp_coeff[2]
		);
	}
}

// #define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);
#endif

static int sensor_scan(void);
static int sensor_init(void);
static void sensor_loop(void);
static struct k_thread sensor_thread_id;
static K_THREAD_STACK_DEFINE(sensor_thread_id_stack, 1024);

K_THREAD_DEFINE(
	sensor_init_thread_id,
	256,
	sensor_request_scan,
	true,
	NULL,
	NULL,
	7,
	0,
	0
);

const char* sensor_get_sensor_imu_name(void) {
	if (sensor_imu_id < 0) {
		return "None";
	}
	return dev_imu_names[sensor_imu_id];
}

const char* sensor_get_sensor_mag_name(void) {
	if (sensor_mag_id < 0) {
		return "None";
	}
	return dev_mag_names[sensor_mag_id];
}

const char* sensor_get_sensor_fusion_name(void) {
	if (fusion_id < 0) {
		return "None";
	}
	return fusion_names[fusion_id];
}

void sensor_scan_thread(void) {
	int err;
	sys_interface_resume();  // make sure interfaces are enabled
	err = sensor_scan();  // IMUs discovery
	if (err) {
		k_msleep(5);
		LOG_INF("Retrying sensor detection");

		// Reset address before retrying sensor detection
		sensor_imu_dev.addr = 0x00;

		err = sensor_scan();  // on POR, the sensor may not be ready yet
	}
	sys_interface_suspend();
	//	if (err)
	//		return err;
}

int sensor_scan(void) {
	while (sensor_sensor_scanning) {
		k_usleep(1);  // already scanning
	}
	if (sensor_sensor_init) {
		return 0;  // already initialized
	}
	sensor_sensor_scanning = true;

	sensor_scan_read();
	int imu_id = -1;
#if SENSOR_IMU_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver
	// initialization if needed
	sensor_imu_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for IMU");
	imu_id = sensor_scan_imu_spi(&sensor_imu_spi_dev, &sensor_imu_dev_reg);
	if (imu_id >= 0) {
		sensor_interface_register_sensor_imu_spi(&sensor_imu_spi_dev);
	}
#endif
#if SENSOR_IMU_EXISTS
	if (imu_id < 0) {
		LOG_INF("Scanning I2C bus for IMU");
		imu_id = sensor_scan_imu(&sensor_imu_dev, &sensor_imu_dev_reg);
		if (imu_id >= 0) {
			sensor_interface_register_sensor_imu_i2c(&sensor_imu_dev);
		}
	}
#endif
#if !SENSOR_IMU_SPI_EXISTS && !SENSOR_IMU_EXISTS
	LOG_ERR("IMU node does not exist");
#endif
	if (imu_id >= (int)ARRAY_SIZE(dev_imu_names)) {
		LOG_WRN("Found unknown device");
	} else if (imu_id < 0) {
		LOG_ERR("No IMU detected");
	} else {
		LOG_INF("Found %s", dev_imu_names[imu_id]);
	}
	if (imu_id >= 0) {
		if (imu_id >= (int)ARRAY_SIZE(sensor_imus) || sensor_imus[imu_id] == NULL
			|| sensor_imus[imu_id] == &sensor_imu_none) {
			sensor_scan_clear();  // clear invalid sensor data
			sensor_imu = &sensor_imu_none;
			sensor_sensor_scanning = false;  // done
			LOG_ERR("IMU not supported");
			set_status(SYS_STATUS_SENSOR_ERROR, true);
			return -1;  // an IMU was detected but not supported
		} else {
			sensor_imu = sensor_imus[imu_id];
		}
	} else {
		sensor_scan_clear();  // clear invalid sensor data
		sensor_imu = &sensor_imu_none;
		sensor_sensor_scanning = false;  // done
		set_status(SYS_STATUS_SENSOR_ERROR, true);
		return -1;  // no IMU detected! something is very wrong
	}

	int mag_id = -1;
#if SENSOR_MAG_SPI_EXISTS
	// for SPI scan, set frequency of 10MHz, it will be set later by the driver
	// initialization if needed
	sensor_mag_spi_dev.config.frequency = MHZ(10);
	LOG_INF("Scanning SPI bus for magnetometer");
	mag_id = sensor_scan_mag_spi(&sensor_mag_spi_dev, &sensor_mag_dev_reg);
	if (mag_id >= 0) {
		sensor_interface_register_sensor_mag_spi(&sensor_mag_spi_dev);
	}
#endif
#if SENSOR_MAG_EXISTS
	if (mag_id < 0) {
		LOG_INF("Scanning bus for magnetometer");
		mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
		if (mag_id >= 0) {
			sensor_interface_register_sensor_mag_i2c(&sensor_mag_dev);
		}
	}
	if (mag_id < 0 && !(sensor_imu_dev.addr & 0x80))  // I2C IMU
	{
		// IMU may support passthrough mode if the magnetometer is connected through the
		// IMU
		int err = sensor_imu->ext_passthrough(
			true
		);  // no need to disable, the imu will be reset later
		if (!err) {
			LOG_INF("Scanning bus for magnetometer through IMU passthrough");
			if (sensor_mag_dev.addr > 0x80)  // marked as external
			{
				sensor_mag_dev.addr &= 0x7F;
			} else {
				sensor_mag_dev.addr = 0x00;  // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag(&sensor_mag_dev, &sensor_mag_dev_reg);
			if (mag_id >= 0) {
				sensor_mag_dev.addr |= 0x80;  // mark as external
				sensor_interface_register_sensor_mag_i2c(
					&sensor_mag_dev
				);  // can register as i2c
			}
		}
	}
#endif
#if SENSOR_MAG_EXT_EXISTS
	if (mag_id < 0 && (sensor_imu_dev.addr & 0x80))  // SPI IMU
	{
		// IMU may support I2CM if the magnetometer is connected through the IMU
		int err = sensor_imu->ext_setup();
		if (!err) {
			LOG_INF("Scanning bus for magnetometer through IMU I2CM");
			if (sensor_mag_dev.addr > 0x80)  // marked as external
			{
				sensor_mag_dev.addr &= 0x7F;
			} else {
				sensor_mag_dev.addr = 0x00;  // reset magnetometer data
				sensor_mag_dev_reg = 0xFF;
			}
			mag_id = sensor_scan_mag_ext(
				sensor_interface_ext_get(),
				&sensor_mag_dev.addr,
				&sensor_mag_dev_reg
			);
			if (mag_id >= 0 && mag_id < (int)ARRAY_SIZE(sensor_mags)
				&& sensor_mags[mag_id] != NULL
				&& sensor_mags[mag_id] != &sensor_mag_none) {
				err = sensor_interface_register_sensor_mag_ext(
					sensor_mag_dev.addr,
					sensor_mags[mag_id]->ext_min_burst,
					sensor_mags[mag_id]->ext_burst
				);
				sensor_mag_dev.addr |= 0x80;  // mark as external
				if (err) {
					mag_id = -1;
					LOG_ERR("Failed to register magnetometer external interface");
				}
			}
		}
	}
#endif
#if !SENSOR_MAG_SPI_EXISTS && !SENSOR_MAG_EXISTS && !SENSOR_MAG_EXT_EXISTS
	LOG_WRN("Magnetometer node does not exist");
#endif
	if (mag_id >= (int)ARRAY_SIZE(dev_mag_names)) {
		LOG_WRN("Found unknown device");
	} else if (mag_id < 0) {
		LOG_WRN("No magnetometer detected");
	} else {
		LOG_INF("Found %s", dev_mag_names[mag_id]);
	}
	if (mag_id >= 0)  // if there is no magnetometer we do not care as much
	{
		if (mag_id >= (int)ARRAY_SIZE(sensor_mags) || sensor_mags[mag_id] == NULL
			|| sensor_mags[mag_id] == &sensor_mag_none) {
			sensor_mag = &sensor_mag_none;
			mag_available = false;
			LOG_ERR("Magnetometer not supported");
		} else {
			sensor_mag = sensor_mags[mag_id];
			mag_available = true;
		}
	} else {
		sensor_mag = &sensor_mag_none;
		mag_available = false;  // marked as not available
	}

	sensor_scan_write();
	connection_update_sensor_ids(imu_id, mag_id);
	sensor_imu_id = imu_id;
	sensor_mag_id = mag_id;

	sensor_sensor_init = true;  // successfully initialized
	sensor_sensor_scanning = false;  // done
	set_status(SYS_STATUS_SENSOR_ERROR, false);  // clear error
	return 0;
}

static bool main_running = false;

int sensor_request_scan(bool force) {
	if (sensor_sensor_init && !force) {
		return 0;  // already initialized
	}
	main_imu_suspend();
	k_thread_abort(
		&sensor_thread_id
	);  // stop the sensor thread // TODO: may need to handle fusion state
	LOG_INF("Aborted sensor thread");
	main_suspended = false;
	sensor_sensor_init = false;
	if (force) {
		sensor_imu_dev.addr = 0x00;
		sensor_mag_dev.addr = 0x00;
		sensor_imu_dev_reg = 0xFF;
		sensor_mag_dev_reg = 0xFF;
		LOG_INF("Requested sensor scan");
	}
	k_thread_create(
		&sensor_thread_id,
		sensor_thread_id_stack,
		K_THREAD_STACK_SIZEOF(sensor_thread_id_stack),
		(k_thread_entry_t)sensor_scan_thread,
		NULL,
		NULL,
		NULL,
		7,
		0,
		K_NO_WAIT
	);
	k_thread_join(&sensor_thread_id, K_FOREVER);  // wait for the thread to finish
	if (sensor_sensor_init && force) {
		k_thread_create(
			&sensor_thread_id,
			sensor_thread_id_stack,
			K_THREAD_STACK_SIZEOF(sensor_thread_id_stack),
			(k_thread_entry_t)sensor_loop,
			NULL,
			NULL,
			NULL,
			7,
			0,
			K_NO_WAIT
		);
		LOG_INF("Started sensor loop");
	}
	return !sensor_sensor_init;
}

void sensor_scan_read(void)  // TODO: move some of this to sys?
{
	if (retained->imu_addr != 0) {
		sensor_imu_dev.addr = retained->imu_addr;
		sensor_imu_dev_reg = retained->imu_reg;
	}
	if (retained->mag_addr != 0) {
		sensor_mag_dev.addr = retained->mag_addr;
		sensor_mag_dev_reg = retained->mag_reg;
	}
	LOG_INF(
		"IMU address: 0x%02X, register: 0x%02X",
		sensor_imu_dev.addr,
		sensor_imu_dev_reg
	);
	LOG_INF(
		"Magnetometer address: 0x%02X, register: 0x%02X",
		sensor_mag_dev.addr,
		sensor_mag_dev_reg
	);
}

void sensor_scan_write(void)  // TODO: move some of this to sys?
{
	retained->imu_addr = sensor_imu_dev.addr;
	retained->mag_addr = sensor_mag_dev.addr;
	retained->imu_reg = sensor_imu_dev_reg;
	retained->mag_reg = sensor_mag_dev_reg;
	retained_update();
}

void sensor_scan_clear(void)  // TODO: move some of this to sys?
{
	retained->imu_addr = 0x00;
	retained->mag_addr = 0x00;
	retained->imu_reg = 0xFF;
	retained->mag_reg = 0xFF;
	retained_update();
}

void sensor_retained_read(
	void
)  // TODO: move some of this to sys? or move to calibration?
{
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	LOG_INF("Accelerometer matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF(
			"%.5f %.5f %.5f %.5f",
			(double)retained->accBAinv[0][i],
			(double)retained->accBAinv[1][i],
			(double)retained->accBAinv[2][i],
			(double)retained->accBAinv[3][i]
		);
	}
#else
	LOG_INF(
		"Accelerometer bias: %.5f %.5f %.5f",
		(double)retained->accelBias[0],
		(double)retained->accelBias[1],
		(double)retained->accelBias[2]
	);
#endif
	LOG_INF(
		"Gyroscope bias: %.5f %.5f %.5f",
		(double)retained->gyroBias[0],
		(double)retained->gyroBias[1],
		(double)retained->gyroBias[2]
	);
	if (mag_available && mag_enabled) {
		//		LOG_INF("Magnetometer bridge offset: %.5f %.5f %.5f",
		//(double)retained->magBias[0], (double)retained->magBias[1],
		//(double)retained->magBias[2]);
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)retained->magBAinv[0][i],
				(double)retained->magBAinv[1][i],
				(double)retained->magBAinv[2][i],
				(double)retained->magBAinv[3][i]
			);
		}
	}
	if (retained->fusion_id) {
		LOG_INF("Fusion data recovered");
	}
}

void sensor_retained_write(void)  // TODO: move to sys?
{
	if (!sensor_fusion_init) {
		return;
	}
	//	memcpy(retained->magBias, sensor_calibration_get_magBias(),
	// sizeof(retained->magBias));
	sensor_fusion->save(retained->fusion_data);
	retained->fusion_id = fusion_id;
	retained_update();
}

void sensor_shutdown(void)  // Communicate all imus to shut down
{
	int err = sensor_request_scan(false);  // try initialization if possible
	if (mag_available || !err) {
		sys_interface_resume();
		if (mag_available) {  // try to shutdown magnetometer first (in case of
							  // passthrough)
			sensor_mag->shutdown();
		}
		if (!err) {
			sensor_imu->shutdown();
		}
		sys_interface_suspend();
	} else {
		LOG_ERR("Failed to shutdown sensors");
	}
}

uint8_t sensor_setup_WOM(void) {
	int err = sensor_request_scan(false);  // try initialization if possible
	if (!err) {
		sys_interface_resume();
		err = sensor_imu->setup_WOM();
		sys_interface_suspend();
		return err;
	} else {
		LOG_ERR("Failed to configure IMU wake up");
		return 0;
	}
}

void sensor_fusion_invalidate(void) {
	main_imu_restart();  // reinitialize fusion
	if (sensor_fusion_init) {  // clear fusion gyro offset
		float g_off[3] = {0};
		sensor_fusion->set_gyro_bias(g_off);
		// Reset quaternion smoothing
		q_smooth_prev[0] = 1.0f;
		q_smooth_prev[1] = 0.0f;
		q_smooth_prev[2] = 0.0f;
		q_smooth_prev[3] = 0.0f;
		// Reset EMA state
		ema_g_inited = false;
		ema_a_inited = false;
		sensor_retained_write();
	} else {  // TODO: always clearing the fusion?
		retained->fusion_id = 0;  // Invalidate retained fusion data
		retained_update();
	}
}

int sensor_update_time_ms = 6;

// TODO: get rid of it.. ?
static void set_update_time_ms(int time_ms) {
	sensor_update_time_ms = time_ms;  // TODO: terrible naming
}

int sensor_init(void) {
	int err;
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
	if (mag_available) {  // shutdown magnetometer first (in case of passthrough)
		sensor_mag->shutdown();  // TODO: is this needed?
	}
	sensor_imu->shutdown();  // TODO: is this needed?

	float clock_actual_rate = 0;
#if CONFIG_USE_SENSOR_CLOCK
	set_sensor_clock(
		true,
		32768,
		&clock_actual_rate
	);  // enable the clock source for IMU if present
#endif
	if (clock_actual_rate != 0) {
		LOG_INF("Sensor clock rate: %.2fHz", (double)clock_actual_rate);
	}

	// wait for sensor register reset // TODO: is this needed?
	k_usleep(250);

	// set FS/range
	float accel_range = CONFIG_SENSOR_ACCEL_FS;
	float gyro_range = CONFIG_SENSOR_GYRO_FS;
	float accel_actual_range, gyro_actual_range;
	sensor_imu
		->update_fs(accel_range, gyro_range, &accel_actual_range, &gyro_actual_range);
	LOG_INF("Accelerometer range: %.2fg", (double)accel_actual_range);
	LOG_INF("Gyroscope range: %.2fdps", (double)gyro_actual_range);

	// setup sensor, set ODR
	float accel_initial_time
		= 1.0 / CONFIG_SENSOR_ACCEL_ODR;  // configure with ~1000Hz ODR
	float gyro_initial_time
		= 1.0 / CONFIG_SENSOR_GYRO_ODR;  // configure with ~1000Hz ODR
	float mag_initial_time
		= sensor_update_time_ms / 1000.0;  // configure with ~200Hz ODR
	err = sensor_imu->init(
		clock_actual_rate,
		accel_initial_time,
		gyro_initial_time,
		&accel_actual_time,
		&gyro_actual_time
	);
#if SENSOR_IMU_SPI_EXISTS
	LOG_INF(
		"Requested SPI frequency: %.2fMHz",
		(double)sensor_imu_spi_dev.config.frequency / 1000000.0
	);
#endif
	LOG_INF("Accelerometer initial rate: %.2fHz", 1.0 / (double)accel_actual_time);
	LOG_INF("Gyrometer initial rate: %.2fHz", 1.0 / (double)gyro_actual_time);
	if (err < 0) {
		return err;
	}
	// 55-66ms to wait, get chip ids, and setup icm (50ms spent waiting for accel and
	// gyro to start)
	if (mag_available && mag_enabled) {
		// TODO: need to flag passthrough enabled
		//			sensor_imu->ext_passthrough(true); // reenable passthrough
		err = sensor_mag->init(
			mag_initial_time,
			&mag_actual_time
		);  // configure with ~200Hz ODR
#if SENSOR_MAG_SPI_EXISTS
		LOG_INF(
			"Requested SPI frequency: %.2fMHz",
			(double)sensor_mag_spi_dev.config.frequency / 1000000.0
		);
#endif
		LOG_INF("Magnetometer initial rate: %.2fHz", 1.0 / (double)mag_actual_time);
		if (err < 0) {
			return err;
		}
		// 0-1ms to setup mmc
	}
	LOG_INF("Initialized sensors");

	// Setup fusion
	sensor_retained_read();  // TODO: useless
	if (fusion_id == FUSION_VQF) {
		vqf_update_sensor_ids(sensor_imu_id);
	}
	if (retained->fusion_id == fusion_id)  // Check if the retained fusion data is valid
										   // and matches the selected fusion
	{  // Load state if the data is valid (fusion was initialized before)
		sensor_fusion->load(retained->fusion_data);
		retained->fusion_id = 0;  // Invalidate retained fusion data
		retained_update();
	} else {
		sensor_fusion->init(
			gyro_actual_time,
			accel_actual_time,
			mag_initial_time
		);  // TODO: using initial time since mag are not polled at the actual rate
		// Reset quaternion smoothing on fusion restart
		q_smooth_prev[0] = 1.0f;
		q_smooth_prev[1] = 0.0f;
		q_smooth_prev[2] = 0.0f;
		q_smooth_prev[3] = 0.0f;
		// Reset EMA state
		ema_g_inited = false;
		ema_a_inited = false;
	}

	sensor_calibration_update_sensor_ids(sensor_imu_id);
	if (sensor_imu == &sensor_imu_bmi270)  // bmi270 specific
	{
		LOG_INF("Applying gyroscope gain");
		bmi_gain_apply(sensor_calibration_get_sensor_data());
	}

	LOG_INF("Using %s", fusion_names[fusion_id]);
	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
	return 0;
}

enum sensor_sensor_mode {
	//	SENSOR_SENSOR_MODE_OFF,
	SENSOR_SENSOR_MODE_LOW_NOISE,
	SENSOR_SENSOR_MODE_LOW_POWER,
	SENSOR_SENSOR_MODE_LOW_POWER_2
};

static enum sensor_sensor_mode sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
static enum sensor_sensor_mode last_sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;

enum sensor_sensor_timeout {
	SENSOR_SENSOR_TIMEOUT_IMU,
	SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY,
	SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED,
};

static enum sensor_sensor_timeout sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;

static bool main_ok = false;
static bool send_info = false;

static int packet_errors = 0;

#define ACQUISITION_START_MS 1000
#define STATUS_INTERVAL_MS 5000

static int64_t last_status_time = 0;
static int64_t max_loop_time = 0;

#if DEBUG
static int64_t last_acquisition_time = INT64_MAX;
static uint64_t total_acquisition_time = 0;
static uint64_t total_read_packets = 0;
static uint64_t total_processed_packets = 0;
static uint64_t total_gyro_samples = 0;
static uint64_t total_accel_samples = 0;
#endif

void sensor_loop(void) {
	if (!sensor_sensor_init) {
		return;
	}
	main_running = true;
	sys_interface_resume();  // make sure interfaces are enabled
	int err = sensor_init();  // Initialize IMUs and Fusion // TODO: run as thread
							  // before loop
	// TODO: handle imu init error, maybe restart device?
	// TODO: on failure to init, disable sensor interface
	if (err) {
		set_status(
			SYS_STATUS_SENSOR_ERROR,
			true
		);  // TODO: only handles general init error
	} else {
		main_ok = true;
	}
	while (1) {
		int64_t time_begin = k_uptime_get();
		if (main_ok) {
			// Resume devices
			sys_interface_resume();

			// Trigger reconfig on sensor mode change
			bool reconfig = last_sensor_mode != sensor_mode;
			last_sensor_mode = sensor_mode;

			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33
			// samples, low power) Magneto sample will take ~400us Fusing data will take
			// between 100us (~7 samples, low noise) - 500us (~33 samples, low power)
			// for xiofusion
			// TODO: on any errors set main_ok false and skip (make functions return
			// nonzero)

			// At high speed, use oneshot mode to have synced magnetometer data
			// Call before FIFO and get the data after
			if (mag_available && mag_enabled && mag_use_oneshot) {
				sensor_mag->mag_oneshot();
			}

			// Read IMU temperature
			float temp = sensor_imu->temp_read();
			connection_update_sensor_temp(temp);

			// Read gyroscope (FIFO)
#if CONFIG_SENSOR_USE_LOW_POWER_2
			uint8_t* rawData = (uint8_t*)k_malloc(
				1900
			);  // Limit FIFO read to 2048 bytes (worst case is ICM 20 byte packet at
				// 1000Hz and 100ms update time)
			if (rawData == NULL) {
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets
				= sensor_imu->fifo_read(rawData, 1900);  // TODO: name this better?
#else
			uint8_t* rawData = (uint8_t*)k_malloc(
				1024
			);  // Limit FIFO read to 768 bytes (worst case is ICM 20 byte packet at
				// 1000Hz and 33ms update time)
			if (rawData == NULL) {
				LOG_ERR("Failed to allocate memory for FIFO buffer");
				set_status(SYS_STATUS_SENSOR_ERROR, true);
				main_ok = false;
			}
			uint16_t packets
				= sensor_imu->fifo_read(rawData, 1024);  // TODO: name this better?
#endif

			// Debug info
#if DEBUG
			int64_t acquisition_time = k_uptime_ticks();
			bool valid_acquisition
				= k_uptime_get() > ACQUISITION_START_MS
			   && last_acquisition_time
					  < acquisition_time;  // wait before beginning profiling
			if (valid_acquisition) {
				total_acquisition_time += acquisition_time - last_acquisition_time;
				total_read_packets += packets;
			}
			last_acquisition_time = acquisition_time;
#endif

			// Read magnetometer
			float raw_m[3];
			if (mag_available && mag_enabled) {
				sensor_mag->mag_read(
					raw_m
				);  // reading mag last, and it will be processed last
			}

			if (reconfig)  // TODO: get rid of reconfig?
			{
				switch (sensor_mode) {
					case SENSOR_SENSOR_MODE_LOW_NOISE:
						set_update_time_ms(6);
						LOG_INF("Switching sensors to low noise");
						break;
					case SENSOR_SENSOR_MODE_LOW_POWER:
						set_update_time_ms(33);
						LOG_INF("Switching sensors to low power");
						break;
					case SENSOR_SENSOR_MODE_LOW_POWER_2:
						set_update_time_ms(100);
						LOG_INF("Switching sensors to low power 2");
						break;
				};
			}

			// Suspend devices
			sys_interface_suspend();

			// Fuse all data
			float a_sum[3] = {0};
			int a_count = 0;
			max_gyro_speed_square = 0;
			int processed_packets = 0;
			for (uint16_t i = 0; i < packets;
				 i++)  // TODO: fifo_process_ext is available, need to implement it
			{
				float raw_a[3] = {0};
				float raw_g[3] = {0};
				if (sensor_imu->fifo_process(i, rawData, raw_a, raw_g)) {
					continue;  // skip on error
				}

				// TODO: split into separate functions
				if (raw_g[0] != 0 || raw_g[1] != 0 || raw_g[2] != 0) {
#if DEBUG
					if (valid_acquisition) {
						total_gyro_samples++;
					}
#endif
					sensor_calibration_process_gyro(raw_g);

					// Apply temperature compensation
					apply_temperature_compensation(temp, raw_g);

					float gx = raw_g[0];
					float gy = raw_g[1];
					float gz = raw_g[2];

					// Align to sensor axes
					float g_raw[] = {SENSOR_GYROSCOPE_AXES_ALIGNMENT};

					// Seed del EMA en la primera muestra
					if (!ema_g_inited) {
						memcpy(ema_g_prev, g_raw, sizeof(ema_g_prev));
						ema_g_inited = true;
					}

					// Instantaneous magnitude (dps) to parameterize filters
					float g_abs = sqrtf(
						g_raw[0] * g_raw[0] + g_raw[1] * g_raw[1] + g_raw[2] * g_raw[2]
					);

					// Dynamic EMA: stationary = smoother; fast = more responsive
					float ema_alpha_gyro = 0.25f + EMA_GYRO_K * g_abs;  // ~0.25..0.85
					if (ema_alpha_gyro > EMA_GYRO_ALPHA_MAX) {
						ema_alpha_gyro = EMA_GYRO_ALPHA_MAX;
					}
					if (ema_alpha_gyro < EMA_GYRO_ALPHA_MIN) {
						ema_alpha_gyro = EMA_GYRO_ALPHA_MIN;
					}

					// Anti-spike
					const float g_step_max = EMA_GYRO_STEP_MAX;  // dps/frame

					float g[3];
					for (int i = 0; i < 3; i++) {
						float dg = g_raw[i] - ema_g_prev[i];
						if (dg > g_step_max) {
							dg = g_step_max;
						}
						if (dg < -g_step_max) {
							dg = -g_step_max;
						}

						float g_limited = ema_g_prev[i] + dg;
						g[i] = ema_g_prev[i] * (1.0f - ema_alpha_gyro)
							 + g_limited * ema_alpha_gyro;
						ema_g_prev[i] = g[i];
					}

					// Process fusion
					sensor_fusion->update_gyro(g, gyro_actual_time);

					if (mag_available && mag_enabled) {
						float g_off[3] = {};
						sensor_fusion->get_gyro_bias(g_off);
						for (int i = 0; i < 3; i++) {
							g_off[i] = g[i] - g_off[i];
						}

						float gyro_speed_square = g_off[0] * g_off[0]
												+ g_off[1] * g_off[1]
												+ g_off[2] * g_off[2];
						if (gyro_speed_square > max_gyro_speed_square) {
							max_gyro_speed_square = gyro_speed_square;
						}
					}
				}

				if (raw_a[0] != 0 || raw_a[1] != 0 || raw_a[2] != 0) {
#if DEBUG
					if (valid_acquisition) {
						total_accel_samples++;
					}
#endif
					sensor_calibration_process_accel(raw_a);
					float ax = raw_a[0];
					float ay = raw_a[1];
					float az = raw_a[2];

					// Alineado a ejes del dispositivo
					float a_raw[] = {SENSOR_ACCELEROMETER_AXES_ALIGNMENT};

					// Seed del EMA en la primera muestra
					if (!ema_a_inited) {
						memcpy(ema_a_prev, a_raw, sizeof(ema_a_prev));
						ema_a_inited = true;
					}

					/* Deviation from 1g to detect jumps/impacts */
					float a_mag = sqrtf(
						a_raw[0] * a_raw[0] + a_raw[1] * a_raw[1] + a_raw[2] * a_raw[2]
					);
					float a_dev = fabsf(a_mag - 1.0f);

					/* Dynamic EMA: stationary = smoother; impact = less smoothing */
					float ema_alpha_acc
						= 0.30f
						+ 0.20f
							  * (a_dev > MAG_MAX_ADEV_G
									 ? 1.0f
									 : (a_dev / MAG_MAX_ADEV_G));  // 0.30..0.50
					if (ema_alpha_acc > EMA_ACCEL_ALPHA_MAX) {
						ema_alpha_acc = EMA_ACCEL_ALPHA_MAX;
					}
					if (ema_alpha_acc < EMA_ACCEL_ALPHA_MIN) {
						ema_alpha_acc = EMA_ACCEL_ALPHA_MIN;
					}

					/* Wide anti-spike for fast steps */
					const float a_step_max = EMA_ACCEL_STEP_MAX;  // g/frame

					float a[3];
					for (int i = 0; i < 3; i++) {
						float da = a_raw[i] - ema_a_prev[i];
						if (da > a_step_max) {
							da = a_step_max;
						}
						if (da < -a_step_max) {
							da = -a_step_max;
						}

						float a_limited = ema_a_prev[i] + da;
						a[i] = ema_a_prev[i] * (1.0f - ema_alpha_acc)
							 + a_limited * ema_alpha_acc;
						ema_a_prev[i] = a[i];
					}

					/* fusion */
					sensor_fusion->update_accel(a, accel_actual_time);

					/* acumula para promedio del frame */
					for (int i = 0; i < 3; i++) {
						a_sum[i] += a[i];
					}
					a_count++;
				}

				processed_packets++;
			}

			// Free the FIFO buffer
			k_free(rawData);

#if DEBUG
			if (valid_acquisition) {
				total_processed_packets += processed_packets;
			}
#endif

			if (mag_available && mag_enabled) {
				bool mag_calibrated = true;
				float uncalibrated_m[3] = {0};
				memcpy(
					uncalibrated_m,
					raw_m,
					sizeof(uncalibrated_m)
				);  // copy raw magnetometer data
				sensor_calibration_process_mag(raw_m);
				float zero_m[3] = {0};
				if (v_epsilon(
						raw_m,
						zero_m,
						1e-6
					))  // if the magnetometer is not calibrated, skip and send raw data
				{
					memcpy(raw_m, uncalibrated_m, sizeof(uncalibrated_m));
					mag_calibrated = false;
				}
				float mx = raw_m[0];
				float my = raw_m[1];
				float mz = raw_m[2];
				float m[] = {SENSOR_MAGNETOMETER_AXES_ALIGNMENT};

				// Process fusion
				if (mag_calibrated) {
					float mx = m[0], my = m[1], mz = m[2];
					float mag_norm = sqrtf(mx * mx + my * my + mz * mz);

					// Giro actual (dps)
					float gyro_speed = sqrtf(max_gyro_speed_square);

					// Frame average acceleration to detect impacts/jumps
					float a_avg[3] = {0};
					if (a_count > 0) {
						a_avg[0] = a_sum[0] / a_count;
						a_avg[1] = a_sum[1] / a_count;
						a_avg[2] = a_sum[2] / a_count;
					}
					float a_mag_avg = sqrtf(
						a_avg[0] * a_avg[0] + a_avg[1] * a_avg[1] + a_avg[2] * a_avg[2]
					);
					float a_dev_for_gate = fabsf(a_mag_avg - 1.0f);

					/* Reglas:
					 *  - Norma de B razonable
					 *  - Giro < 800 dps (spins: ignorar mag para no “snapear” yaw)
					 *  - |a|-1g < 0.4g (impactos)
					 */
					bool mag_ok = (mag_norm >= MAG_NORM_MIN && mag_norm <= MAG_NORM_MAX)
							   && (gyro_speed <= MAG_MAX_GYRO_DPS)
							   && (a_dev_for_gate < MAG_MAX_ADEV_G);

					if (mag_ok) {
						sensor_fusion->update_mag(m, sensor_update_time_ms / 1000.0f);
					}
				}

				v_rotate(m, q3, m);  // magnetic field in local device frame, no other
									 // transformation will be done
				connection_update_sensor_mag(m);
			}

			// Copy average acceleration for this frame
			static float a[3] = {0};  // keep persistent
			if (a_count > 0) {
				for (int i = 0; i < 3; i++) {
					a[i] = a_sum[i] / a_count;
				}
			}

			// Check packet processing
			if ((packets != 0 || k_uptime_get() > 100) && processed_packets == 0) {
				if (packets) {
					LOG_WRN("No packets processed");
				} else {
					LOG_WRN("No packets in buffer");
				}
				if (++packet_errors == 10) {
					LOG_ERR("Packet error threshold exceeded");
					set_status(SYS_STATUS_SENSOR_ERROR, true);
					if (packets) {
						sensor_retained_write();  // keep the fusion state
						sys_request_system_reboot();
					}
				}
			} else if (processed_packets == packets && packets > 0) {
				packet_errors = 0;
			}

			// Update fusion gyro sanity? // TODO: use to detect drift and correct or
			// suspend tracking
			//			sensor_fusion->update_gyro_sanity(g, m);

			// Get updated quaternion from fusion
			sensor_fusion->get_quat(q);
			q_normalize(q, q);  // safe to use self as output

			// alpha depends on current rotation energy
			float gyro_speed_out = sqrtf(max_gyro_speed_square);
			float q_alpha = 0.25f + 0.0005f * gyro_speed_out;  // ~0.25..0.85
			if (q_alpha > 0.85f) {
				q_alpha = 0.85f;
			}
			if (q_alpha < 0.20f) {
				q_alpha = 0.20f;
			}

			float q_smoothed[4];
			quat_smooth_lerp(q_smooth_prev, q, q_alpha, q_smoothed);
			memcpy(q_smooth_prev, q_smoothed, sizeof(q_smooth_prev));
			memcpy(q, q_smoothed, sizeof(q));  // use smoothed q for lin_a and sending

			// Get linear acceleration // TODO: move to util functions
			float lin_a[3] = {0};
			if (v_diff_mag(a, lin_a) != 0)  // lin_a as zero vector
			{
				float vec_gravity[3] = {0};
				vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
				vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
				vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);
				for (int i = 0; i < 3; i++) {
					lin_a[i] = (a[i] - vec_gravity[i])
							 * CONST_EARTH_GRAVITY;  // vector to m/s^2
				}
			}

			// Check the IMU gyroscope // TODO: gyro sanity not used
			bool calibrating = get_status(SYS_STATUS_CALIBRATION_RUNNING);
			bool resting
				= sensor_fusion->get_gyro_sanity() == 0
					? q_epsilon(q, last_q, 0.005)
					: q_epsilon(
						  q,
						  last_q,
						  0.05
					  );  // TODO: Probably okay to use the constantly updating last_q?
			if (!calibrating && resting) {
				int64_t last_data_delta = k_uptime_get() - last_data_time;
				if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER
					&& last_data_delta
						   > CONFIG_SENSOR_LP_TIMEOUT)  // No motion in lp timeout
				{
					LOG_INF("No motion from sensors in %dms", CONFIG_SENSOR_LP_TIMEOUT);
					sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER;
				}
#if CONFIG_SENSOR_USE_LOW_POWER_2 || CONFIG_USE_IMU_TIMEOUT
				int64_t imu_timeout = CLAMP(
					last_data_time - last_suspend_attempt_time,
					CONFIG_IMU_TIMEOUT_RAMP_MIN,
					CONFIG_IMU_TIMEOUT_RAMP_MAX
				);  // Ramp timeout from last_data_time
#endif
#if CONFIG_SENSOR_USE_LOW_POWER_2
				if (sensor_mode < SENSOR_SENSOR_MODE_LOW_POWER_2
					&& last_data_delta > imu_timeout) {  // No motion in ramp time
					sensor_mode = SENSOR_SENSOR_MODE_LOW_POWER_2;
				}
#endif
#if CONFIG_USE_ACTIVE_TIMEOUT
				if (sensor_timeout < SENSOR_SENSOR_TIMEOUT_ACTIVITY
					&& last_data_delta
						   > CONFIG_ACTIVE_TIMEOUT_THRESHOLD)  // higher priority than
															   // IMU timeout
				{
					LOG_INF("Switching to activity timeout");
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_ACTIVITY;
				}
				if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_ACTIVITY
					&& last_data_delta > CONFIG_ACTIVE_TIMEOUT_DELAY) {
					LOG_INF(
						"No motion from sensors in %dm",
						CONFIG_ACTIVE_TIMEOUT_DELAY / 60000
					);
#if CONFIG_SLEEP_ON_ACTIVE_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
					sys_request_WOM(
						true
					);  // TODO: should queue shutdown and suspend itself instead
//					main_imu_suspend(); // TODO: auto suspend, the device should
// configure WOM ASAP but it does not
#elif CONFIG_SHUTDOWN_ON_ACTIVE_TIMEOUT && CONFIG_USER_SHUTDOWN
					main_running
						= false;  // skip suspend step, at the moment the thread must be
								  // running to shutdown // TODO: should queue shutdown
								  // and suspend itself instead
					sys_request_system_off();
#endif
					sensor_timeout
						= SENSOR_SENSOR_TIMEOUT_ACTIVITY_ELAPSED;  // only try to
																   // suspend once
				}
#endif
#if CONFIG_USE_IMU_TIMEOUT && CONFIG_USE_IMU_WAKE_UP
				if (sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU
					&& last_data_delta > imu_timeout)  // No motion in ramp time
				{
					LOG_INF("No motion from sensors in %llds", imu_timeout / 1000);
					sys_request_WOM(
						false
					);  // TODO: should queue shutdown and suspend itself instead
					//					main_imu_suspend(); // TODO: auto suspend, the
					// device should configure WOM ASAP but it does not
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED;  // only try to
																		 // suspend once
				}
#endif
			} else {
				if (sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER_2
					|| sensor_timeout == SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED) {
					last_suspend_attempt_time = k_uptime_get();
				}
				last_data_time = k_uptime_get();
				if (sensor_timeout
					== SENSOR_SENSOR_TIMEOUT_IMU_ELAPSED) {  // Resetting IMU timeout
					sensor_timeout = SENSOR_SENSOR_TIMEOUT_IMU;
				}
				sensor_mode = SENSOR_SENSOR_MODE_LOW_NOISE;
			}

			// Update magnetometer mode
			if (mag_available && mag_enabled) {
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time
					= 1.0f / (4 * gyro_speed);  // target mag ODR for ~0.25 deg error
				if (mag_target_time < 0.005f
					&& mag_skip_oneshot) {  // only use continuous modes if oneshot is
											// not available
					mag_target_time = 0.005;
				}
				if (mag_target_time > 0.1f) {  // limit to 0.1 (minimum 10Hz)
					mag_target_time = 0.1;
				}
				sys_interface_resume();
				if (mag_target_time < 0.005f)  // cap at 0.005 (200Hz), above this the
											   // sensor will use oneshot mode instead
				{
					int err = sensor_mag->update_odr(INFINITY, &mag_actual_time);
					if (mag_actual_time == INFINITY) {
						if (!err) {
							LOG_DBG("Switching magnetometer to oneshot");
						}
						mag_use_oneshot = true;
					} else  // magnetometer did not have a oneshot mode, try 200Hz
					{
						if (!err) {
							mag_skip_oneshot = true;
						}
						mag_target_time = 0.005;
					}
				}
				if (mag_target_time >= 0.005f
					|| mag_actual_time != INFINITY)  // under 200Hz or magnetometer did
													 // not have a oneshot mode
				{
					int err = sensor_mag->update_odr(mag_target_time, &mag_actual_time);
					if (!err) {
						LOG_DBG(
							"Switching magnetometer ODR to %.2fHz",
							1.0 / (double)mag_actual_time
						);
					}
					mag_use_oneshot = false;
				}
				sys_interface_suspend();
			}

			// Check if last status is outdated
			if (!send_info && (k_uptime_get() - last_info_time > 100)) {
				send_info = true;
				last_info_time = k_uptime_get();
			}

			// Send packet with new orientation
			bool send_quat_data = !q_epsilon(q, last_q, 0.001);
			bool send_lin_accel_data = !v_epsilon(lin_a, last_lin_a, 0.05);
			if (send_quat_data || send_lin_accel_data) {
				bool send_precise_quat = q_epsilon(q, last_q, 0.005);
				memcpy(last_q, q, sizeof(q));
				memcpy(last_lin_a, lin_a, sizeof(lin_a));
				float q_offset[4];
				q_multiply(
					q,
					q3,
					q_offset
				);  // quaternion in device orientation, connection will change format
					// from wxyz to xyzw
				v_rotate(
					lin_a,
					q3,
					lin_a
				);  // linear acceleration in local device frame, no other
					// transformation will be done
				connection_update_sensor_data(q_offset, lin_a);
				if (send_info && !send_precise_quat)  // prioritize quat precision
				{
					connection_write_packet_2();
					send_info = false;
				} else if (mag_available && mag_enabled
						   && k_uptime_get() - last_mag_time
								  > 200)  // try to send mag data every 200ms
				{
					connection_write_packet_4();
					last_mag_time = k_uptime_get();
				} else {
					connection_write_packet_1();
				}
			} else if (send_info) {
				connection_write_packet_0();
				send_info = false;
			} else {
				connection_clocks_request_stop();
			}

			// Handle magnetometer calibration
			if (mag_available && mag_enabled
				&& last_sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER
				&& sensor_mode == SENSOR_SENSOR_MODE_LOW_POWER) {
				sensor_request_calibration_mag();
			}
		}

		main_running = false;
		int64_t time_delta = k_uptime_get() - time_begin;

		if (time_delta > sensor_update_time_ms && time_delta > max_loop_time) {
			max_loop_time = time_delta;
		}

		if (k_uptime_get() - last_status_time > STATUS_INTERVAL_MS) {
			last_status_time = k_uptime_get();
			if (max_loop_time > 0) {
				LOG_WRN("Last update steps took up to %lld ms", time_delta);
				max_loop_time = 0;
			}

			// Update temperature calibration periodically
			if (sensor_fusion_init) {
				float gyro_bias[3] = {0};
				sensor_fusion->get_gyro_bias(gyro_bias);
				update_temperature_calibration(temp_current, gyro_bias);
			}

#if DEBUG
			LOG_DBG(
				"packets read: %llu, processed: %llu, gyro samples: %llu, accel "
				"samples: %llu, total acquisition time: %lld us",
				total_read_packets,
				total_processed_packets,
				total_gyro_samples,
				total_accel_samples,
				k_ticks_to_us_near64(total_acquisition_time)
			);
			LOG_DBG(
				"reported gyro rate: %.2fHz, actual: %.2fHz, reported accel rate: "
				"%.2fHz, actual: %.2fHz",
				1.0 / (double)gyro_actual_time,
				(double)total_gyro_samples
					/ (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0,
				1.0 / (double)accel_actual_time,
				(double)total_accel_samples
					/ (double)k_ticks_to_us_near64(total_acquisition_time) * 1000000.0
			);

			// Debug temperature compensation
			if (temp_calibration_active) {
				LOG_DBG(
					"Temperature: %.1f°C, Active temp compensation",
					(double)temp_current
				);
			}
#endif
		}

		//		led_clock_offset += time_delta;
		if (time_delta > sensor_update_time_ms) {
			k_yield();
		} else {
			k_msleep(sensor_update_time_ms - time_delta);
		}

		if (main_suspended) {  // TODO:
			k_thread_suspend(&sensor_thread_id);
		}

		main_running = true;
	}
}

void wait_for_threads(void)  // TODO: add timeout
{
	while (main_running) {
		k_usleep(1);  // bane of my existence. don't use k_yield()!!!!!!
	}
}

void main_imu_suspend(void)  // TODO: add timeout
{
	main_suspended = true;
	if (!main_running) {  // don't suspend if already stopped (TODO: may be called from
						  // sensor thread)
		return;
	}
	while (sensor_sensor_scanning) {
		k_usleep(1);  // try not to interrupt scanning
	}
	while (main_running) {  // TODO: change to detect if i2c is busy
		k_usleep(1);  // try not to interrupt anything actually
	}
	k_thread_suspend(&sensor_thread_id);
	LOG_INF("Suspended sensor thread");
}

void main_imu_resume(void) {
	if (!main_suspended) {  // not suspended
		return;
	}
	k_thread_resume(&sensor_thread_id);
	LOG_INF("Resumed sensor thread");
}

void main_imu_wakeup(void) {
	if (!main_suspended) {  // don't wake up if pending suspension
		k_wakeup(&sensor_thread_id);
	}
}

void main_imu_restart(void) {
	if (main_ok) {  // only restart fusion if initialized
		// Use actual magnetometer time if available, otherwise fall back to current
		// update time
		float mag_time = (mag_available && mag_enabled)
						   ? mag_actual_time
						   : sensor_update_time_ms / 1000.0f;

		sensor_fusion->init(gyro_actual_time, accel_actual_time, mag_time);

		// Reset quaternion smoothing on fusion restart
		q_smooth_prev[0] = 1.0f;
		q_smooth_prev[1] = 0.0f;
		q_smooth_prev[2] = 0.0f;
		q_smooth_prev[3] = 0.0f;

		// Reset EMA state
		ema_g_inited = false;
		ema_a_inited = false;

		// Reset temperature compensation
		temp_ref = temp_current;  // Use current temperature as new reference
		for (int i = 0; i < 3; i++) {
			gyro_temp_coeff[i] = 0.0f;
		}
		temp_calibration_active = false;
	}
}
