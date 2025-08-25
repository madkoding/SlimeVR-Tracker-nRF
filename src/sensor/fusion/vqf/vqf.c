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
#include "util.h"

#include "sensor/sensors_enum.h"
#include "../src/vqf.h"  // conflicting with vqf.h in local path

#include "../vqf/vqf.h"  // conflicting with vqf.h in vqf-c

#include <stdlib.h>  // for malloc/free
#include <math.h>  // for fabsf

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

static uint8_t imu_id;

// Use pointers to move large structures to heap instead of stack
static vqf_params_t *params = NULL;
static vqf_state_t *state = NULL;
static vqf_coeffs_t *coeffs = NULL;

static float last_a[3] = {0};

// Optimization: Use shared work buffer for unit conversions to avoid stack allocation
typedef union
{
	float g_rad[3];  // for gyroscope rad/s conversion
	float a_m_s2[3];  // for accelerometer m/s² conversion
} vqf_work_buffer_t;

static vqf_work_buffer_t work_buffer;

// Sensor status tracking for optimizations
static bool gyro_available = false;
static bool mag_available = false;
static uint32_t gyro_zero_count = 0;
static uint32_t mag_zero_count = 0;

// Magnetometer control - set to false to disable magnetometer usage
// Disabling magnetometer can help in environments with magnetic interference
// or when testing pure gyro+accel orientation tracking
static bool use_magnetometer = false;  // Disabled by default

// Motion-to-rest transition detection
static uint32_t rest_candidate_count = 0;
static float last_accel_magnitude = 0.0f;

// Motion state classification for adaptive bias estimation
typedef enum
{
	MOTION_STATE_REST,
	MOTION_STATE_SLOW_MOTION,
	MOTION_STATE_FAST_MOTION
} MotionState;

// Global state variables
static MotionState current_motion_state = MOTION_STATE_REST;
static uint32_t motion_state_counter = 0;

// Increased threshold to avoid false positives during normal operation
// At ~100Hz, 50 samples = ~500ms of truly zero data before marking as unavailable
#define SENSOR_DETECTION_SAMPLES 50  // Number of samples to detect if sensor is working
#define REST_DETECTION_SAMPLES 30  // ~300ms to confirm rest after motion
#define ACCEL_REST_THRESHOLD 0.05f  // Threshold for "at rest" accelerometer (in g)
#define MOTION_BIAS_CORRECTION_INTERVAL \
	50  // Apply bias correction every 50 motion samples (~500ms)

void vqf_update_sensor_ids(int imu) { imu_id = (uint8_t)imu; }

static void set_params()
{
	if (!params)
	{
		params = malloc(sizeof(vqf_params_t));
		if (!params)
		{
			return;  // allocation failed
		}
	}
	init_params(params);

	// Optimized parameters for better gyroscope bias estimation
	params->biasClip = 5.0f;
	params->tauMag = 10.0f;

	// Much more aggressive bias estimation parameters
	params->biasForgettingTime = 80.0f;  // Even faster bias adaptation
	params->biasSigmaInit = 1.5f;  // Lower initial uncertainty
	params->biasSigmaMotion = 0.1f;  // Very aggressive during motion (was 0.35)
	params->biasSigmaRest = 0.02f;  // Very aggressive during rest
	params->biasVerticalForgettingFactor
		= 0.02f;  // Much faster vertical bias correction

	params->motionBiasEstEnabled = true;
	params->restBiasEstEnabled = true;

	// More sensitive rest detection for better bias estimation
	params->restFilterTau = 1.0f;  // Faster rest filter (was 1.5)
	params->restMinT = 1.5f;  // Shorter time to detect rest (was 2.59)
	params->restThAcc = 0.8f;  // More sensitive accel threshold (was 1.42)
	params->restThGyr = 0.8f;  // More sensitive gyro threshold (was 1.40)
	params->tauAcc = 4.0f;
}

void vqf_init(float g_time, float a_time, float m_time)
{
	// Allocate memory for structures if not already allocated
	if (!state)
	{
		state = malloc(sizeof(vqf_state_t));
		if (!state)
		{
			return;  // allocation failed
		}
	}
	if (!coeffs)
	{
		coeffs = malloc(sizeof(vqf_coeffs_t));
		if (!coeffs)
		{
			return;  // allocation failed
		}
	}

	set_params();
	if (!params)
	{
		return;  // allocation failed
	}

	// Reset sensor status tracking
	gyro_available = true;
	mag_available = true;
	gyro_zero_count = 0;
	mag_zero_count = 0;

	// Reset motion transition detection
	rest_candidate_count = 0;
	last_accel_magnitude = CONST_EARTH_GRAVITY;  // Assume starting at rest

	// Reset adaptive bias estimation
	current_motion_state = MOTION_STATE_REST;
	motion_state_counter = 0;

	initVqf(params, state, coeffs, g_time, a_time, m_time);
}

void vqf_load(const void *data)
{
	// Allocate memory if needed
	if (!state)
	{
		state = malloc(sizeof(vqf_state_t));
		if (!state)
		{
			return;
		}
	}
	if (!coeffs)
	{
		coeffs = malloc(sizeof(vqf_coeffs_t));
		if (!coeffs)
		{
			return;
		}
	}

	set_params();
	if (!params)
	{
		return;
	}

	memcpy(state, data, sizeof(vqf_state_t));
	memcpy(coeffs, (const uint8_t *)data + sizeof(vqf_state_t), sizeof(vqf_coeffs_t));
}

void vqf_save(void *data)
{
	if (!state || !coeffs)
	{
		return;  // not initialized
	}

	memcpy(data, state, sizeof(vqf_state_t));
	memcpy((uint8_t *)data + sizeof(vqf_state_t), coeffs, sizeof(vqf_coeffs_t));
}

void vqf_update_gyro(const float *g, float time)
{
	(void)time;  // suppress unused parameter warning
	if (!params || !state || !coeffs)
	{
		return;  // not initialized
	}

	// Improved optimization: Check if gyroscope is providing meaningful data
	// Use a small threshold instead of exact zero to account for sensor noise/bias
	float gyro_magnitude = fabsf(g[0]) + fabsf(g[1]) + fabsf(g[2]);

	// Adaptive threshold based on motion state
	// During motion, be more lenient with gyro "rest" detection
	float gyro_threshold = (current_motion_state != MOTION_STATE_REST)
							 ? 0.05f
							 : 0.01f;  // Higher threshold during motion
	bool is_gyro_very_low = (gyro_magnitude < gyro_threshold);

	if (is_gyro_very_low)
	{
		gyro_zero_count++;
		// Only mark as unavailable after a very long period of truly dead data
		// This preserves bias estimation during rest periods
		if (gyro_zero_count > SENSOR_DETECTION_SAMPLES * 2)
		{
			gyro_available = false;
			return;  // Skip processing only if truly dead for a long time
		}
	}
	else
	{
		gyro_zero_count = 0;
		gyro_available = true;
	}

	// Always process gyro data for bias estimation, even during rest
	// This is critical for VQF's automatic bias correction
	work_buffer.g_rad[0] = g[0] * DEG_TO_RAD;
	work_buffer.g_rad[1] = g[1] * DEG_TO_RAD;
	work_buffer.g_rad[2] = g[2] * DEG_TO_RAD;

	// Adaptive bias estimation based on motion state
	motion_state_counter++;

	// Determine motion state based on gyroscope magnitude
	MotionState new_motion_state;
	if (gyro_magnitude < 0.5f)
	{
		new_motion_state = MOTION_STATE_REST;
	}
	else if (gyro_magnitude < 2.0f)
	{
		new_motion_state = MOTION_STATE_SLOW_MOTION;
	}
	else
	{
		new_motion_state = MOTION_STATE_FAST_MOTION;
	}

	// Update VQF parameters if motion state changed or periodically
	if (new_motion_state != current_motion_state || motion_state_counter % 100 == 0)
	{
		current_motion_state = new_motion_state;

		// Adjust VQF parameters based on motion state
		switch (current_motion_state)
		{
			case MOTION_STATE_REST:
				// More aggressive bias estimation at rest
				params->biasSigmaRest = 0.01f;  // Faster convergence at rest
				params->biasForgettingTime = 50.0f;  // Faster forgetting
				break;
			case MOTION_STATE_SLOW_MOTION:
				// Moderate bias estimation during slow motion
				params->biasSigmaRest = 0.03f;  // Default rest value
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
				params->biasSigmaMotion = 0.05f;  // More trust during slow motion
#endif
				params->biasForgettingTime = 100.0f;  // Default forgetting
				break;
			case MOTION_STATE_FAST_MOTION:
				// Conservative bias estimation during fast motion
				params->biasSigmaRest
					= 0.1f;  // Less aggressive at rest after fast motion
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
				params->biasSigmaMotion = 0.2f;  // Less trust during fast motion
#endif
				params->biasForgettingTime = 200.0f;  // Slower forgetting
				break;
		}
	}

	updateGyr(params, state, coeffs, work_buffer.g_rad);
}

void vqf_update_accel(const float *a, float time)
{
	(void)time;  // suppress unused parameter warning
	if (!params || !state || !coeffs)
	{
		return;  // not initialized
	}

	// Optimization: Use shared work buffer and direct assignment
	// Convert g to m/s^2 directly in work buffer
	work_buffer.a_m_s2[0] = a[0] * CONST_EARTH_GRAVITY;
	work_buffer.a_m_s2[1] = a[1] * CONST_EARTH_GRAVITY;
	work_buffer.a_m_s2[2] = a[2] * CONST_EARTH_GRAVITY;

	// Motion-to-rest transition detection
	// Calculate accelerometer magnitude (should be ~9.8 m/s² at rest)
	float accel_magnitude = sqrtf(
		work_buffer.a_m_s2[0] * work_buffer.a_m_s2[0]
		+ work_buffer.a_m_s2[1] * work_buffer.a_m_s2[1]
		+ work_buffer.a_m_s2[2] * work_buffer.a_m_s2[2]
	);

	// Check if accelerometer indicates rest (close to gravity magnitude)
	float accel_deviation = fabsf(accel_magnitude - CONST_EARTH_GRAVITY);
	bool accel_at_rest = (accel_deviation < ACCEL_REST_THRESHOLD * CONST_EARTH_GRAVITY);

	if (accel_at_rest)
	{
		rest_candidate_count++;
		// No need to set motion state here - it's handled in gyro update
	}
	else
	{
		rest_candidate_count = 0;
		// No need to set motion state here - it's handled in gyro update
	}

	last_accel_magnitude = accel_magnitude;

	// Store for linear acceleration calculation (check for non-zero data)
	if (work_buffer.a_m_s2[0] != 0.0f || work_buffer.a_m_s2[1] != 0.0f
		|| work_buffer.a_m_s2[2] != 0.0f)
	{
		last_a[0] = work_buffer.a_m_s2[0];
		last_a[1] = work_buffer.a_m_s2[1];
		last_a[2] = work_buffer.a_m_s2[2];
	}

	updateAcc(params, state, coeffs, work_buffer.a_m_s2);
}

void vqf_update_mag(const float *m, float time)
{
	(void)time;  // suppress unused parameter warning
	if (!params || !state || !coeffs)
	{
		return;  // not initialized
	}

	// Check if magnetometer usage is disabled
	if (!use_magnetometer)
	{
		mag_available = false;
		return;  // Skip magnetometer processing completely
	}

	// Improved optimization: Check if magnetometer is providing meaningful data
	// Use a small threshold for magnetometer - it should have significant readings
	float mag_magnitude = fabsf(m[0]) + fabsf(m[1]) + fabsf(m[2]);
	bool is_mag_very_low = (mag_magnitude < 0.1f);  // Very low magnetic field

	if (is_mag_very_low)
	{
		mag_zero_count++;
		// Magnetometer can be temporarily disturbed, so be more lenient
		if (mag_zero_count > SENSOR_DETECTION_SAMPLES * 3)
		{
			mag_available = false;
			return;  // Skip magnetometer processing if truly dead for a long time
		}
	}
	else
	{
		mag_zero_count = 0;
		mag_available = true;
	}

	// Process magnetometer data for heading correction
	updateMag(params, state, coeffs, m);
}

void vqf_update(const float *g, const float *a, const float *m, float time)
{
	(void)time;  // suppress unused parameter warning
	if (!params || !state || !coeffs)
	{
		return;  // not initialized
	}

	// Improved processing: Always feed data to VQF for proper bias estimation
	// VQF needs consistent data flow for bias correction and rest detection

	// Always process accelerometer - most reliable reference
	vqf_update_accel(a, time);

	// Always process gyroscope for bias estimation - critical for drift correction
	// VQF uses small movements and rest periods for automatic bias correction
	vqf_update_gyro(g, time);

	// Process magnetometer only if enabled and available
	if (use_magnetometer && mag_available)
	{
		vqf_update_mag(m, time);
	}
}

void vqf_get_gyro_bias(float *g_off)
{
	if (!state || !coeffs)
	{
		return;  // not initialized
	}
	getBiasEstimate(state, coeffs, g_off);
}

void vqf_set_gyro_bias(const float *g_off)
{
	if (!state)
	{
		return;  // not initialized
	}
	float temp_bias[3] = {g_off[0], g_off[1], g_off[2]};
	setBiasEstimate(state, temp_bias, -1);
}

void vqf_update_gyro_sanity(const float *g, const float *m)
{
	// Enhanced: Update sensor availability and assist with bias estimation
	// Check if gyroscope seems to be providing valid data
	float g_magnitude = fabsf(g[0]) + fabsf(g[1]) + fabsf(g[2]);
	float m_magnitude = fabsf(m[0]) + fabsf(m[1]) + fabsf(m[2]);

	bool g_has_data = (g_magnitude > 0.001f);  // Very low threshold for actual data
	bool m_has_data
		= (m_magnitude > 0.01f);  // Magnetometer should have reasonable field

	// Update counters based on data availability
	if (!g_has_data)
	{
		gyro_zero_count++;
	}
	else
	{
		gyro_zero_count = 0;
		gyro_available = true;
	}

	if (!m_has_data)
	{
		mag_zero_count++;
	}
	else
	{
		mag_zero_count = 0;
		mag_available = true;
	}

	// Mark sensors as unavailable if they've been zero for too long
	if (gyro_zero_count > SENSOR_DETECTION_SAMPLES)
	{
		gyro_available = false;
	}
	if (mag_zero_count > SENSOR_DETECTION_SAMPLES)
	{
		mag_available = false;
	}

	// Enhanced: If we've detected stable rest state, provide additional bias correction
	// This helps VQF's bias estimation by giving it clear rest periods
	if (current_motion_state == MOTION_STATE_REST
		&& rest_candidate_count > REST_DETECTION_SAMPLES)
	{
		// During confirmed rest periods, assist VQF with bias information
		// Get current bias estimate
		float current_bias[3];
		if (state && coeffs)
		{
			getBiasEstimate(state, coeffs, current_bias);

			// If gyroscope readings are very low during rest, update bias more
			// aggressively
			if (g_magnitude < 0.5f)
			{  // Less than 0.5°/s during rest
				// Apply small correction towards measured gyro readings
				// This helps VQF converge faster during rest periods
				float bias_correction[3];
				bias_correction[0] = current_bias[0] + g[0] * DEG_TO_RAD * 0.1f;
				bias_correction[1] = current_bias[1] + g[1] * DEG_TO_RAD * 0.1f;
				bias_correction[2] = current_bias[2] + g[2] * DEG_TO_RAD * 0.1f;

				setBiasEstimate(state, bias_correction, -1);
			}
		}
	}
}

int vqf_get_gyro_sanity(void)
{
	// Return sensor status: 0 = good, 1 = gyro issues, 2 = mag issues, 3 = both have
	// issues
	int status = 0;

	if (!gyro_available)
	{
		status |= 1;  // Gyro not available
	}
	if (!mag_available)
	{
		status |= 2;  // Magnetometer not available
	}

	return status;
}

void vqf_get_lin_a(float *lin_a)
{
	if (!state)
	{
		memset(lin_a, 0, sizeof(float) * 3);
		return;
	}

	float q[4] = {0};
	vqf_get_quat(q);

	float vec_gravity[3] = {0};
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

	const float *a = last_a;
	for (int i = 0; i < 3; i++)
	{
		lin_a[i] = a[i] - vec_gravity[i] * CONST_EARTH_GRAVITY;
	}
}

void vqf_get_quat(float *q)
{
	if (!state)
	{
		// Return identity quaternion if not initialized
		q[0] = 1.0f;
		q[1] = 0.0f;
		q[2] = 0.0f;
		q[3] = 0.0f;
		return;
	}
	getQuat9D(state, q);
}

bool vqf_get_rest_detected(void)
{
	if (!state)
	{
		return false;
	}
	return getRestDetected(state);
}

void vqf_get_relative_rest_deviations(float *out)
{
	if (!params || !state)
	{
		memset(out, 0, sizeof(float) * 2);
		return;
	}
	getRelativeRestDeviations(params, state, out);
}

// Cleanup function to free allocated memory
void vqf_cleanup(void)
{
	if (params)
	{
		free(params);
		params = NULL;
	}
	if (state)
	{
		free(state);
		state = NULL;
	}
	if (coeffs)
	{
		free(coeffs);
		coeffs = NULL;
	}
}

// Function to enable/disable magnetometer usage
void vqf_set_magnetometer_enabled(bool enabled)
{
	use_magnetometer = enabled;
	if (!enabled)
	{
		mag_available = false;  // Mark as unavailable when disabled
	}
}

// Function to get current magnetometer usage state
bool vqf_is_magnetometer_enabled(void) { return use_magnetometer; }

const sensor_fusion_t sensor_fusion_vqf
	= {vqf_init,
	   vqf_load,
	   vqf_save,

	   vqf_update_gyro,
	   vqf_update_accel,
	   vqf_update_mag,
	   vqf_update,

	   vqf_get_gyro_bias,
	   vqf_set_gyro_bias,

	   vqf_update_gyro_sanity,
	   vqf_get_gyro_sanity,

	   vqf_get_lin_a,
	   vqf_get_quat};
