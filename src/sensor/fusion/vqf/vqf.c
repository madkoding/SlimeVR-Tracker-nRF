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
#include "../src/vqf.h"
#include "../vqf/vqf.h"

// ============================================================================
// Constants
// ============================================================================

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#define VECTOR_SIZE 3

// ============================================================================
// Sensor state variables
// ============================================================================

static uint8_t imu_id;
static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
static float last_acceleration[VECTOR_SIZE] = {0};

// ============================================================================
// Configuration functions
// ============================================================================

/**
 * Configures optimal VQF filter parameters
 * These values were optimized through empirical testing
 */
static void configure_vqf_parameters() {
	init_params(&params);

	// Magnetometer configuration
	params.tauMag = 10.0f;  // Best result from VQF paper
	params.biasClip = 5.0f;

	// Bias estimation configuration (optimized values)
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.biasForgettingTime = 136.579346f;
	params.biasSigmaInit = 3.219453f;
	params.biasSigmaMotion = 0.348501f;
	params.biasSigmaRest = 0.063616f;
	params.biasVerticalForgettingFactor = 0.007056f;

	// Rest detection configuration
	params.restFilterTau = 1.114532f;
	params.restMinT = 2.586910f;
	params.restThAcc = 1.418598f;
	params.restThGyr = 1.399189f;

	// Accelerometer configuration
	params.tauAcc = 4.337983f;
}

void vqf_update_sensor_ids(int imu) {
	imu_id = imu;
}

void vqf_init(float gyro_time, float accel_time, float mag_time) {
	configure_vqf_parameters();
	initVqf(&params, &state, &coeffs, gyro_time, accel_time, mag_time);
}

// ============================================================================
// Persistence functions (save/load state)
// ============================================================================

void vqf_load(const void *data) {
	configure_vqf_parameters();

	// Load state and coefficients from memory
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t *)data + sizeof(state), sizeof(coeffs));
}

void vqf_save(void *data) {
	// Save state and coefficients to memory
	memcpy(data, &state, sizeof(state));
	memcpy((uint8_t *)data + sizeof(state), &coeffs, sizeof(coeffs));
}

// ============================================================================
// Conversion helper functions
// ============================================================================

/**
 * Converts values from degrees/second to radians/second
 */
static void convert_degrees_to_radians(const float *degrees, float *radians) {
	for (int i = 0; i < VECTOR_SIZE; i++) {
		radians[i] = degrees[i] * DEG_TO_RAD;
	}
}

/**
 * Converts values from G (gravity) to m/s²
 */
static void convert_g_to_meters_per_second_squared(const float *g_values, float *ms2_values) {
	for (int i = 0; i < VECTOR_SIZE; i++) {
		ms2_values[i] = g_values[i] * CONST_EARTH_GRAVITY;
	}
}

/**
 * Checks if a vector is zero (all components are 0)
 */
static bool is_vector_zero(const float *vector) {
	return (vector[0] == 0.0f && vector[1] == 0.0f && vector[2] == 0.0f);
}

// ============================================================================
// Sensor update functions
// ============================================================================

void vqf_update_gyro(float *gyro_deg_per_sec, float time) {
	float gyro_rad_per_sec[VECTOR_SIZE];
	convert_degrees_to_radians(gyro_deg_per_sec, gyro_rad_per_sec);
	updateGyr(&params, &state, &coeffs, gyro_rad_per_sec);
}

void vqf_update_accel(float *accel_g, float time) {
	float accel_ms2[VECTOR_SIZE];
	convert_g_to_meters_per_second_squared(accel_g, accel_ms2);

	// Save the last valid accelerometer reading
	if (!is_vector_zero(accel_ms2)) {
		memcpy(last_acceleration, accel_ms2, sizeof(accel_ms2));
	}

	updateAcc(&params, &state, &coeffs, accel_ms2);
}

void vqf_update_mag(float *magnetometer, float time) {
	updateMag(&params, &state, &coeffs, magnetometer);
}

void vqf_update(float *gyro, float *accel, float *mag, float time) {
	// Ignore gyroscope readings if all values are zero
	if (!is_vector_zero(gyro)) {
		vqf_update_gyro(gyro, time);
	}

	vqf_update_accel(accel, time);
	vqf_update_mag(mag, time);
}

// ============================================================================
// Gyroscope bias management functions
// ============================================================================

void vqf_get_gyro_bias(float *gyro_offset) {
	getBiasEstimate(&state, &coeffs, gyro_offset);
}

void vqf_set_gyro_bias(float *gyro_offset) {
	setBiasEstimate(&state, gyro_offset, -1);
}

void vqf_update_gyro_sanity(float *gyro, float *mag) {
	// TODO: Implement gyroscope sanity check
	// VQF does not yet provide a "recovery state"
	return;
}

int vqf_get_gyro_sanity() {
	// TODO: Implement sanity state retrieval
	// VQF does not yet provide a "recovery state"
	return 0;
}

// ============================================================================
// Orientation and acceleration retrieval functions
// ============================================================================

/**
 * Calculates the gravity vector in the sensor frame using the quaternion
 */
static void calculate_gravity_vector(const float *quaternion, float *gravity_vector) {
	const float q0 = quaternion[0];
	const float q1 = quaternion[1];
	const float q2 = quaternion[2];
	const float q3 = quaternion[3];

	gravity_vector[0] = 2.0f * (q1 * q3 - q0 * q2);
	gravity_vector[1] = 2.0f * (q2 * q3 + q0 * q1);
	gravity_vector[2] = 2.0f * (q0 * q0 - 0.5f + q3 * q3);
}

void vqf_get_lin_a(float *linear_acceleration) {
	float quaternion[4];
	vqf_get_quat(quaternion);

	// Calculate the gravity vector in the sensor frame
	float gravity_vector[VECTOR_SIZE];
	calculate_gravity_vector(quaternion, gravity_vector);

	// Subtract gravity from total acceleration to get linear acceleration
	for (int i = 0; i < VECTOR_SIZE; i++) {
		linear_acceleration[i] = last_acceleration[i] - (gravity_vector[i] * CONST_EARTH_GRAVITY);
	}
}

void vqf_get_quat(float *quaternion) {
	getQuat9D(&state, quaternion);
}

bool vqf_get_rest_detected() {
	return getRestDetected(&state);
}

void vqf_get_relative_rest_deviations(float *deviations) {
	getRelativeRestDeviations(&params, &state, deviations);
}

// ============================================================================
// Sensor fusion interface structure
// ============================================================================

const sensor_fusion_t sensor_fusion_vqf = {
	*vqf_init,
	*vqf_load,
	*vqf_save,

	*vqf_update_gyro,
	*vqf_update_accel,
	*vqf_update_mag,
	*vqf_update,

	*vqf_get_gyro_bias,
	*vqf_set_gyro_bias,

	*vqf_update_gyro_sanity,
	*vqf_get_gyro_sanity,

	*vqf_get_lin_a,
	*vqf_get_quat
};
