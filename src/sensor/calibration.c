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
#include "calibration.h"

#include <math.h>

#include "globals.h"
#include "imu/BMI270.h"
#include "magneto/magneto1_4.h"
#include "sensors_enum.h"
#include "system/system.h"
#include "util.h"

static uint8_t imu_id;
static uint8_t sensor_data[128];  // any use sensor data

static float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0};  // offset biases

#define RUNTIME_GYRO_STORE_INTERVAL_MS (60000)
#define RUNTIME_GYRO_STORE_THRESHOLD (0.05f)
#define RUNTIME_GYRO_DELTA_EPS (0.0005f)

static float runtime_gyro_pending[3] = {0};
static int64_t runtime_gyro_last_store = 0;

static float accBAinv[4][3];
static float magBAinv[4][3];

static uint8_t magneto_progress;
static uint8_t last_magneto_progress;
static int64_t magneto_progress_time;

static double ata[100];  // init calibration
static double norm_sum;
static double sample_count;

// #define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_INF);
#endif

static void sensor_sample_accel(const float a[3]);
static int sensor_wait_accel(float a[3], k_timeout_t timeout);

static void sensor_sample_gyro(const float g[3]);
static int sensor_wait_gyro(float g[3], k_timeout_t timeout);

static void sensor_sample_mag(const float m[3]);
static int sensor_wait_mag(float m[3], k_timeout_t timeout);

static void sensor_calibrate_imu(void);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static void sensor_calibrate_6_side(void);
#endif
static int sensor_calibrate_mag(void);
static bool sensor_calibration_limit_bias(float* bias, float limit);

// helpers
static bool wait_for_motion(bool motion, int samples);
static int check_sides(const float*);
static void magneto_reset(void);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float*, float*, float, int*, int);
#endif

// calibration logic
static int sensor_offsetBias(float* dest1, float* dest2);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int sensor_6_sideBias(float a_inv[][3]);
#endif
static void sensor_sample_mag_magneto_sample(const float a[3], const float m[3]);

static int sensor_calibration_request(int id);

static void calibration_thread(void);
K_THREAD_DEFINE(
	calibration_thread_id,
	1024,
	calibration_thread,
	NULL,
	NULL,
	NULL,
	6,
	0,
	0
);

void sensor_calibration_process_accel(float a[3]) {
	sensor_sample_accel(a);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	apply_BAinv(a, accBAinv);
#else
	for (int i = 0; i < 3; i++) {
		a[i] -= accelBias[i];
	}
#endif
}

void sensor_calibration_process_gyro(float g[3]) {
	sensor_sample_gyro(g);
	for (int i = 0; i < 3; i++) {
		g[i] -= gyroBias[i];
	}
}

void sensor_calibration_process_mag(float m[3]) {
	//	for (int i = 0; i < 3; i++)
	//		m[i] -= magBias[i];
	sensor_sample_mag(m);
	apply_BAinv(m, magBAinv);
}

void sensor_calibration_update_sensor_ids(int imu) { imu_id = imu; }

uint8_t* sensor_calibration_get_sensor_data() { return sensor_data; }

void sensor_calibration_read(void) {
	memcpy(sensor_data, retained->sensor_data, sizeof(sensor_data));
	memcpy(accelBias, retained->accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained->gyroBias, sizeof(gyroBias));
	memcpy(magBias, retained->magBias, sizeof(magBias));
	memcpy(magBAinv, retained->magBAinv, sizeof(magBAinv));
	memcpy(accBAinv, retained->accBAinv, sizeof(accBAinv));
	memset(runtime_gyro_pending, 0, sizeof(runtime_gyro_pending));
	runtime_gyro_last_store = k_uptime_get();
}

void sensor_calibration_get_gyro_bias(float bias[3]) {
	if (bias == NULL) {
		return;
	}
	memcpy(bias, gyroBias, sizeof(gyroBias));
}

static bool sensor_calibration_limit_bias(float* bias, float limit) {
	bool changed = false;
	for (int i = 0; i < 3; i++) {
		if (!isfinite(bias[i])) {
			bias[i] = 0.0f;
			changed = true;
			continue;
		}
		if (bias[i] > limit) {
			bias[i] = limit;
			changed = true;
		} else if (bias[i] < -limit) {
			bias[i] = -limit;
			changed = true;
		}
	}
	return changed;
}

bool sensor_calibration_integrate_runtime_gyro_bias(
	const float delta[3],
	float applied[3]
) {
	if (applied != NULL) {
		for (int i = 0; i < 3; i++) {
			applied[i] = 0.0f;
		}
	}
	if (delta == NULL) {
		return false;
	}

	float original[3];
	memcpy(original, gyroBias, sizeof(original));

	bool updated = false;
	for (int i = 0; i < 3; i++) {
		float addition = delta[i];
		if (!isfinite(addition)) {
			continue;
		}
		if (fabsf(addition) < RUNTIME_GYRO_DELTA_EPS) {
			continue;
		}
		gyroBias[i] += addition;
		updated = true;
	}

	if (!updated) {
		return false;
	}

	bool limited = sensor_calibration_limit_bias(gyroBias, 50.0f);

	for (int i = 0; i < 3; i++) {
		float diff = gyroBias[i] - original[i];
		runtime_gyro_pending[i] += diff;
		if (applied != NULL) {
			applied[i] = diff;
		}
	}

	if (limited) {
		LOG_WRN("Runtime gyro bias limited to safe bounds");
	}

	float pending_max = fmaxf(
		fabsf(runtime_gyro_pending[0]),
		fmaxf(fabsf(runtime_gyro_pending[1]), fabsf(runtime_gyro_pending[2]))
	);
	int64_t now = k_uptime_get();
	if (pending_max >= RUNTIME_GYRO_STORE_THRESHOLD
		|| now - runtime_gyro_last_store >= RUNTIME_GYRO_STORE_INTERVAL_MS) {
		memcpy(retained->gyroBias, gyroBias, sizeof(gyroBias));
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));
		memset(runtime_gyro_pending, 0, sizeof(runtime_gyro_pending));
		runtime_gyro_last_store = now;
		LOG_INF("Persisted runtime gyro bias adjustment");
	}

	return true;
}

int sensor_calibration_validate(float* a_bias, float* g_bias, bool write) {
	if (a_bias == NULL) {
		a_bias = accelBias;
	}
	if (g_bias == NULL) {
		g_bias = gyroBias;
	}
	float zero[3] = {0};
	bool accel_valid = v_epsilon(a_bias, zero, 0.5);
	bool gyro_valid = v_epsilon(g_bias, zero, 50.0);

	if (!accel_valid || !gyro_valid) {
		if (write) {
			bool accel_updated = false;
			bool gyro_updated = false;
			if (!accel_valid) {
				accel_updated = sensor_calibration_limit_bias(a_bias, 0.5f);
				if (accel_updated) {
					LOG_WRN("Stored accelerometer bias out of range, clamping");
				}
			}
			if (!gyro_valid) {
				gyro_updated = sensor_calibration_limit_bias(g_bias, 50.0f);
				if (gyro_updated) {
					LOG_WRN("Stored gyroscope bias out of range, clamping");
				}
			}
			if (accel_updated || gyro_updated) {
				LOG_WRN("Keeping previous calibration after limiting to safe bounds");
				if (accel_updated) {
					sys_write(
						MAIN_ACCEL_BIAS_ID,
						&retained->accelBias,
						accelBias,
						sizeof(accelBias)
					);
				}
				if (gyro_updated) {
					sys_write(
						MAIN_GYRO_BIAS_ID,
						&retained->gyroBias,
						gyroBias,
						sizeof(gyroBias)
					);
				}
				return 0;
			}
		}

		sensor_calibration_clear(a_bias, g_bias, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_calibration_validate_6_side(float a_inv[][3], bool write) {
	if (a_inv == NULL) {
		a_inv = accBAinv;
	}
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = a_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(a_inv[0], zero, 0.5)
		|| !v_epsilon(
			diagonal,
			average,
			magnitude * 0.1f
		))  // check accel is <0.5G and diagonals are within 10%
	{
		sensor_calibration_clear_6_side(a_inv, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}
#endif

int sensor_calibration_validate_mag(float m_inv[][3], bool write) {
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(m_inv[0], zero, 1)
		|| !v_epsilon(
			diagonal,
			average,
			MAX(magnitude * 0.2f, 0.1f)
		))  // check offset is <1 unit and diagonals are within 20%
	{
		sensor_calibration_clear_mag(m_inv, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN(
			"The magnetometer may be damaged or calibration was not completed properly"
		);
		return -1;
	}
	return 0;
}

void sensor_calibration_clear(float* a_bias, float* g_bias, bool write) {
	if (a_bias == NULL) {
		a_bias = accelBias;
	}
	if (g_bias == NULL) {
		g_bias = gyroBias;
	}
	memset(a_bias, 0, sizeof(accelBias));
	memset(g_bias, 0, sizeof(gyroBias));
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, a_bias, sizeof(accelBias));
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, g_bias, sizeof(gyroBias));
	}

	sensor_fusion_invalidate();
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibration_clear_6_side(float a_inv[][3], bool write) {
	if (a_inv == NULL) {
		a_inv = accBAinv;
	}
	memset(a_inv, 0, sizeof(accBAinv));
	for (int i = 0; i < 3; i++) {  // set identity matrix
		a_inv[i + 1][i] = 1;
	}
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, a_inv, sizeof(accBAinv));
	}
}
#endif

void sensor_calibration_clear_mag(float m_inv[][3], bool write) {
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	memset(
		m_inv,
		0,
		sizeof(magBAinv)
	);  // zeroed matrix will disable magnetometer in fusion
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, m_inv, sizeof(magBAinv));
	}
}

void sensor_request_calibration(void) { sensor_calibration_request(1); }

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_request_calibration_6_side(void) { sensor_calibration_request(2); }
#endif

void sensor_request_calibration_mag(void) {
	magneto_progress |= 1 << 7;
	if (magneto_progress == 0b10111111) {
		magneto_progress |= 1 << 6;
	}
}

static float aBuf[3] = {0};
uint64_t accel_sample = 0;
uint64_t accel_wait_sample = 0;

static void sensor_sample_accel(const float a[3]) {
	memcpy(aBuf, a, sizeof(aBuf));
	accel_sample++;
	if (accel_wait_sample) {
		k_usleep(1);  // yield to waiting thread
	}
}

static int sensor_wait_accel(float a[3], k_timeout_t timeout) {
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	accel_wait_sample = accel_sample;
	while (accel_sample <= accel_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	accel_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Accelerometer wait timed out");
		return -1;
	}
	memcpy(a, aBuf, sizeof(aBuf));
	return 0;
}

static float gBuf[3] = {0};
uint64_t gyro_sample = 0;
uint64_t gyro_wait_sample = 0;

static void sensor_sample_gyro(const float g[3]) {
	memcpy(gBuf, g, sizeof(gBuf));
	gyro_sample++;
	if (gyro_wait_sample) {
		k_usleep(1);  // yield to waiting thread
	}
}

static int sensor_wait_gyro(float g[3], k_timeout_t timeout) {
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	gyro_wait_sample = gyro_sample;
	while (gyro_sample <= gyro_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	gyro_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Gyroscope wait timed out");
		return -1;
	}
	memcpy(g, gBuf, sizeof(gBuf));
	return 0;
}

static float mBuf[3] = {0};
uint64_t mag_sample = 0;
uint64_t mag_wait_sample = 0;

static void sensor_sample_mag(const float m[3]) {
	memcpy(mBuf, m, sizeof(mBuf));
	mag_sample++;
	if (mag_wait_sample) {
		k_usleep(1);  // yield to waiting thread
	}
}

static int sensor_wait_mag(float m[3], k_timeout_t timeout) {
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	mag_wait_sample = mag_sample;
	while (mag_sample <= mag_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	mag_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Magnetometer wait timed out");
		return -1;
	}
	memcpy(m, mBuf, sizeof(mBuf));
	return 0;
}

static void sensor_calibrate_imu() {
	float a_bias[3], g_bias[3];
	LOG_INF("Calibrating main accelerometer and gyroscope zero rate offset");
	LOG_INF("Rest the device on a stable surface");

	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	if (!wait_for_motion(false, 6))  // Wait for accelerometer to settle, timeout 3s
	{
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;  // Timeout, calibration failed
	}

	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
	k_msleep(500);  // Delay before beginning acquisition

	if (imu_id == IMU_BMI270)  // bmi270 specific
	{
		LOG_INF("Suspending sensor thread");
		main_imu_suspend();
		LOG_INF("Running BMI270 component retrimming");
		int err = bmi_crt(
			sensor_data
		);  // will automatically reinitialize // TODO: this blocks sensor!
		LOG_INF("Resuming sensor thread");
		main_imu_resume();
		if (err) {
			LOG_WRN("IMU specific calibration was not completed properly");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;  // Calibration failed
		}
		LOG_INF("Finished IMU specific calibration");
		sys_write(
			MAIN_SENSOR_DATA_ID,
			&retained->sensor_data,
			sensor_data,
			sizeof(sensor_data)
		);
		sensor_fusion_invalidate();  // only invalidate fusion if calibration was
									 // successful
		k_msleep(500);  // Delay before beginning acquisition
	}

	LOG_INF("Reading data");
	sensor_calibration_clear(a_bias, g_bias, false);
	int err = sensor_offsetBias(a_bias, g_bias);
	if (err)  // This takes about 3s
	{
		if (err == -1) {
			LOG_INF("Motion detected");
		}
		a_bias[0] = NAN;  // invalidate calibration
	} else {
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		LOG_INF(
			"Accelerometer bias: %.5f %.5f %.5f",
			(double)a_bias[0],
			(double)a_bias[1],
			(double)a_bias[2]
		);
#endif
		LOG_INF(
			"Gyroscope bias: %.5f %.5f %.5f",
			(double)g_bias[0],
			(double)g_bias[1],
			(double)g_bias[2]
		);
	}
	if (sensor_calibration_validate(a_bias, g_bias, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		LOG_INF(
			"Accelerometer bias: %.5f %.5f %.5f",
			(double)accelBias[0],
			(double)accelBias[1],
			(double)accelBias[2]
		);
#endif
		LOG_INF(
			"Gyroscope bias: %.5f %.5f %.5f",
			(double)gyroBias[0],
			(double)gyroBias[1],
			(double)gyroBias[2]
		);
		sensor_calibration_validate(
			NULL,
			NULL,
			true
		);  // additionally verify old calibration
		return;
	} else {
		LOG_INF("Applying calibration");
		memcpy(accelBias, a_bias, sizeof(accelBias));
		memcpy(gyroBias, g_bias, sizeof(gyroBias));
		sensor_fusion_invalidate();  // only invalidate fusion if calibration was
									 // successful
	}
	sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, gyroBias, sizeof(gyroBias));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static void sensor_calibrate_6_side(void) {
	float a_inv[4][3];
	LOG_INF("Calibrating main accelerometer 6-side offset");
	LOG_INF("Rest the device on a stable surface");

	sensor_calibration_clear_6_side(a_inv, false);
	int err = sensor_6_sideBias(a_inv);
	if (err) {
		magneto_reset();
		if (err == -1) {
			LOG_INF("Motion detected");
		}
		a_inv[0][0] = NAN;  // invalidate calibration
	} else {
		LOG_INF("Accelerometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)a_inv[0][i],
				(double)a_inv[1][i],
				(double)a_inv[2][i],
				(double)a_inv[3][i]
			);
		}
	}
	if (sensor_calibration_validate_6_side(a_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Accelerometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)accBAinv[0][i],
				(double)accBAinv[1][i],
				(double)accBAinv[2][i],
				(double)accBAinv[3][i]
			);
		}
		sensor_calibration_validate_6_side(
			NULL,
			true
		);  // additionally verify old calibration
		return;
	} else {
		LOG_INF("Applying calibration");
		memcpy(accBAinv, a_inv, sizeof(accBAinv));
		sensor_fusion_invalidate();  // only invalidate fusion if calibration was
									 // successful
	}
	sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, accBAinv, sizeof(accBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}
#endif

static int sensor_calibrate_mag(void) {
	float zero[3] = {0};
	if (v_diff_mag(magBAinv[0], zero) != 0) {
		return -1;  // magnetometer calibration already exists
	}

	float m[3];
	if (sensor_wait_mag(m, K_MSEC(1000))) {
		return -1;  // Timeout
	}
	sensor_sample_mag_magneto_sample(aBuf, m);  // 400us
	if (magneto_progress != 0b11111111) {
		return 0;
	}

	float m_inv[4][3];
	LOG_INF("Calibrating magnetometer hard/soft iron offset");

	// max allocated 1072 bytes
#if DEBUG
	printk("ata:\n");
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			printk("%7.2f, ", (double)ata[i * 10 + j]);
		}
		printk("\n");
		k_msleep(3);
	}
	printk("norm_sum: %.2f, sample_count: %.0f\n", norm_sum, sample_count);
#endif
	wait_for_threads();
	magneto_current_calibration(m_inv, ata, norm_sum, sample_count);  // 25ms
	magneto_reset();

	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF(
			"%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i],
			(double)m_inv[1][i],
			(double)m_inv[2][i],
			(double)m_inv[3][i]
		);
	}
	if (sensor_calibration_validate_mag(m_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)magBAinv[0][i],
				(double)magBAinv[1][i],
				(double)magBAinv[2][i],
				(double)magBAinv[3][i]
			);
		}
		sensor_calibration_validate_mag(
			NULL,
			true
		);  // additionally verify old calibration
		return -1;
	} else {
		LOG_INF("Applying calibration");
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		// fusion invalidation not necessary
	}
	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
	return 0;
}

// TODO: isAccRest
static bool wait_for_motion(bool motion, int samples) {
	uint8_t counts = 0;
	float a[3], last_a[3];
	if (sensor_wait_accel(last_a, K_MSEC(1000))) {
		return false;
	}
	LOG_INF(
		"Accelerometer: %.5f %.5f %.5f",
		(double)last_a[0],
		(double)last_a[1],
		(double)last_a[2]
	);
	for (int i = 0; i < samples + counts; i++) {
		k_msleep(500);
		if (sensor_wait_accel(a, K_MSEC(1000))) {
			return false;
		}
		LOG_INF(
			"Accelerometer: %.5f %.5f %.5f",
			(double)a[0],
			(double)a[1],
			(double)a[2]
		);
		if (v_epsilon(a, last_a, 0.1) != motion) {
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2) {
				return true;
			}
		} else {
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Motion detected");
	return false;
}

static int check_sides(const float* a) {
	return (-1.2f < a[0] && a[0] < -0.8f ? 1 << 0 : 0)
		 | (1.2f > a[0] && a[0] > 0.8f ? 1 << 1 : 0)
		 |  // dumb check if all accel axes were reached for calibration, assume the
			// user is intentionally doing this
		   (-1.2f < a[1] && a[1] < -0.8f ? 1 << 2 : 0)
		 | (1.2f > a[1] && a[1] > 0.8f ? 1 << 3 : 0)
		 | (-1.2f < a[2] && a[2] < -0.8f ? 1 << 4 : 0)
		 | (1.2f > a[2] && a[2] > 0.8f ? 1 << 5 : 0);
}

static void magneto_reset(void) {
	magneto_progress = 0;  // reusing ata, so guarantee cleared mag progress
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	memset(ata, 0, sizeof(ata));
	norm_sum = 0;
	sample_count = 0;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int
isAccRest(float* acc, float* pre_acc, float threshold, int* t, int restdelta) {
	float delta_x = acc[0] - pre_acc[0];
	float delta_y = acc[1] - pre_acc[1];
	float delta_z = acc[2] - pre_acc[2];

	float norm_diff = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

	if (norm_diff <= threshold) {
		*t += restdelta;
	} else {
		*t = 0;
	}

	if (*t > 2000) {
		return 1;
	}
	return 0;
}
#endif

// TODO: setup 6 sided calibration (bias and scale, and maybe gyro ZRO?), setup temp
// calibration (particulary for gyro ZRO)
int sensor_offsetBias(float* dest1, float* dest2) {
	float rawData[3], last_a[3];
	if (sensor_wait_accel(last_a, K_MSEC(1000))) {
		return -2;  // Timeout
	}
	int64_t sampling_start_time = k_uptime_get();
	int i = 0;
	while (k_uptime_get() < sampling_start_time + 10000) {
		if (sensor_wait_accel(rawData, K_MSEC(1000))) {
			return -2;  // Timeout
		}
		if (!v_epsilon(rawData, last_a, 0.1)) {
			return -1;  // Motion detected
		}
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		dest1[0] += rawData[0];
		dest1[1] += rawData[1];
		dest1[2] += rawData[2];
#endif
		if (sensor_wait_gyro(rawData, K_MSEC(1000))) {
			return -2;  // Timeout
		}
		dest2[0] += rawData[0];
		dest2[1] += rawData[1];
		dest2[2] += rawData[2];
		i++;
	}
	LOG_INF("Samples: %d", i);
#if !CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	dest1[0] /= i;
	dest1[1] /= i;
	dest1[2] /= i;
	if (dest1[0] > 0.9f) {
		dest1[0]
			-= 1.0f;  // Remove gravity from the x-axis accelerometer bias calculation
	} else if (dest1[0] < -0.9f) {
		dest1[0]
			+= 1.0f;  // Remove gravity from the x-axis accelerometer bias calculation
	} else if (dest1[1] > 0.9f) {
		dest1[1]
			-= 1.0f;  // Remove gravity from the y-axis accelerometer bias calculation
	} else if (dest1[1] < -0.9f) {
		dest1[1]
			+= 1.0f;  // Remove gravity from the y-axis accelerometer bias calculation
	} else if (dest1[2] > 0.9f) {
		dest1[2]
			-= 1.0f;  // Remove gravity from the z-axis accelerometer bias calculation
	} else if (dest1[2] < -0.9f) {
		dest1[2]
			+= 1.0f;  // Remove gravity from the z-axis accelerometer bias calculation
	} else {
		return -1;
	}
#endif
	dest2[0] /= i;
	dest2[1] /= i;
	dest2[2] /= i;
	return 0;
}

// TODO: can be used to get a better gyro bias
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_6_sideBias(float a_inv[][3]) {
	// Acc 6 side calibrate
	float rawData[3];
	float pre_acc[3] = {0};

	const float THRESHOLD_ACC = 0.05;
	int resttime = 0;

	magneto_reset();
	int c = 0;
	printk("Starting accelerometer calibration.\n");
	while (1) {
		set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
		printk("Waiting for a resting state...\n");
		while (1) {
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2;  // Timeout, magneto state not handled here
			}
			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			pre_acc[0] = rawData[0];
			pre_acc[1] = rawData[1];
			pre_acc[2] = rawData[2];

			// force not resting until a new side is detected and stable
			uint8_t new_magneto_progress = magneto_progress;
			new_magneto_progress |= check_sides(rawData);
			if (new_magneto_progress > magneto_progress
				&& new_magneto_progress == last_magneto_progress) {
				if (k_uptime_get() < magneto_progress_time) {
					rest = 0;
				}
			} else {
				magneto_progress_time = k_uptime_get() + 1000;
				last_magneto_progress = new_magneto_progress;
				rest = 0;
			}

			if (rest == 1) {
				magneto_progress = new_magneto_progress;
				printk(
					"Rest detected, starting recording. Please do not move. %d\n",
					c
				);
				set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
				k_msleep(100);

				int64_t sampling_start_time = k_uptime_get();
				uint8_t i = 0;
				while (k_uptime_get() < sampling_start_time + 1000) {
					if (sensor_wait_accel(rawData, K_MSEC(1000))) {
						return -2;  // Timeout, magneto state not handled here
					}
					if (!v_epsilon(rawData, pre_acc, 0.1)) {
						return -1;  // Motion detected
					}
					magneto_sample(
						rawData[0],
						rawData[1],
						rawData[2],
						ata,
						&norm_sum,
						&sample_count
					);
					if (k_uptime_get() >= sampling_start_time + i * 100) {
						printk("#");
						i++;
					}
				}
				set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
				printk("Recorded values!\n");
				printk("%d side done\n", c);
				c++;
				break;
			}
			k_msleep(100);
		}
		if (c >= 6) {
			break;
		}
		printk("Waiting for the next side... %d \n", c);
		while (1) {
			k_msleep(100);
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2;  // Timeout, magneto state not handled here
			}
			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			pre_acc[0] = rawData[0];
			pre_acc[1] = rawData[1];
			pre_acc[2] = rawData[2];

			if (rest == 0) {
				resttime = 0;
				break;
			}
		}
	}

	printk("Calculating the data....\n");
#if DEBUG
	printk("ata:\n");
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			printk("%7.2f, ", (double)ata[i * 10 + j]);
		}
		printk("\n");
		k_msleep(3);
	}
	printk("norm_sum: %.2f, sample_count: %.0f\n", norm_sum, sample_count);
#endif
	wait_for_threads();  // TODO: let the data cook or something idk why this has to be
						 // here to work
	magneto_current_calibration(a_inv, ata, norm_sum, sample_count);
	magneto_reset();

	printk("Calibration is complete.\n");
	return 0;
}
#endif

// TODO: terrible name
static void sensor_sample_mag_magneto_sample(const float a[3], const float m[3]) {
	magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count);  // 400us
	uint8_t new_magneto_progress = magneto_progress;
	new_magneto_progress |= check_sides(a);
	if (new_magneto_progress > magneto_progress
		&& new_magneto_progress == last_magneto_progress) {
		if (k_uptime_get() > magneto_progress_time) {
			magneto_progress = new_magneto_progress;
			LOG_INF(
				"Magnetometer calibration progress: %s %s %s %s %s %s",
				(new_magneto_progress & 0x01) ? "-X" : "--",
				(new_magneto_progress & 0x02) ? "+X" : "--",
				(new_magneto_progress & 0x04) ? "-Y" : "--",
				(new_magneto_progress & 0x08) ? "+Y" : "--",
				(new_magneto_progress & 0x10) ? "-Z" : "--",
				(new_magneto_progress & 0x20) ? "+Z" : "--"
			);
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		}
	} else {
		magneto_progress_time = k_uptime_get() + 1000;
		last_magneto_progress = new_magneto_progress;
	}
	if (magneto_progress == 0b10111111) {
		set_led(
			SYS_LED_PATTERN_FLASH,
			SYS_LED_PRIORITY_SENSOR
		);  // Magnetometer calibration is ready to apply
	}
}

static int sensor_calibration_request(int id) {
	static int requested = 0;
	switch (id) {
		case -1:
			requested = 0;
			return 0;
		case 0:
			return requested;
		default:
			if (requested != 0) {
				LOG_ERR("Sensor calibration is already running");
				return -1;
			}
			requested = id;
			return 0;
	}
}

static void calibration_thread(void) {
	sensor_calibration_read();
	// TODO: be able to block the sensor while doing certain operations
	// TODO: reset fusion on calibration finished
	// TODO: start and run thread from request?
	// TODO: replace wait_for_motion with isAccRest

	// Verify calibrations
	sensor_calibration_validate(NULL, NULL, true);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	sensor_calibration_validate_6_side(NULL, true);
#endif
	sensor_calibration_validate_mag(NULL, true);

	// requested calibrations run here
	while (1) {
		int requested = sensor_calibration_request(0);
		switch (requested) {
			case 1:
				set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
				sensor_calibrate_imu();
				sensor_calibration_request(-1);  // clear request
				set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
				break;
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
			case 2:
				set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
				sensor_calibrate_6_side();
				sensor_calibration_request(-1);  // clear request
				set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
				break;
#endif
			default:
				if (magneto_progress & 0b10000000) {
					requested = sensor_calibrate_mag();
				}
				break;
		}
		if (requested < 0) {
			k_msleep(5);
		} else {
			k_msleep(100);
		}
	}
}
