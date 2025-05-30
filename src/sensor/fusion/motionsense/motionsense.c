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
#include "motionsense.h"

#include <math.h>
#include <string.h>

#include "globals.h"
#include "util.h"

LOG_MODULE_REGISTER(sensorfusion, LOG_LEVEL_INF);

// ZUPT and Kalman filter constants (based on NXP analysis)
#define ZUPT_REST_TH_GYR_DEG 1.5f  // Rest detection threshold for gyro (°/s)
#define ZUPT_REST_TH_ACC_G 0.15f  // Rest detection threshold for accel (g)
#define ZUPT_REST_MIN_TIME_S 2.0f  // Minimum rest time (s)
#define ZUPT_BIAS_SIGMA_INIT_DEG 1.0f  // Initial bias uncertainty (°/s)
#define ZUPT_BIAS_SIGMA_REST_DEG 0.1f  // Bias uncertainty during rest (°/s)
#define ZUPT_BIAS_CLIP_DEG 3.0f  // Maximum bias magnitude (°/s)
#define ZUPT_BIAS_FORGETTING_TIME_S 200.0f  // Bias forgetting time constant (s)

// Kalman filter noise parameters
#define KALMAN_Q_GYRO_VAR (2E2f)  // Gyro process noise variance (deg/s)²
#define KALMAN_Q_BIAS_VAR (2E-2f)  // Bias process noise variance (deg/s)²
#define KALMAN_R_ACCEL_VAR (1.2E-3f)  // Accel measurement noise variance g²

// Constants
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define GRAVITY_NORM 1.0f
#define MAX_SAMPLE_RATE 200.0f

// ZUPT and Kalman filter state structure
typedef struct {
	// Orientation quaternion (w, x, y, z)
	float quat[4];

	// Gyroscope bias estimate (rad/s)
	float gyro_bias[3];

	// Linear acceleration in global frame (g)
	float lin_accel[3];

	// State covariance matrix (9x9: 3 orientation + 3 bias + 3 accel)
	float P[9][9];

	// ZUPT detection state
	float rest_timer;
	bool is_at_rest;
	float last_gyro_norm;
	float last_accel_norm;

	// Filter parameters
	float dt;
	float gyro_time;
	float accel_time;
	float mag_time;

	// Bias estimation parameters
	float bias_clip;
	float bias_sigma_init;
	float bias_sigma_rest;
	float bias_forgetting_factor;

	// Sanity check
	int gyro_sanity;

	// Initialization flag
	bool is_initialized;
} MotionSenseState;

static MotionSenseState state = {0};

// Utility functions
static float vector_norm(const float v[3]) {
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void vector_copy(float dst[3], const float src[3]) {
	memcpy(dst, src, 3 * sizeof(float));
}

static void vector_cross(float result[3], const float a[3], const float b[3]) {
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];
}

static void quat_normalize(float q[4]) {
	float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (norm > 1e-6f) {
		q[0] /= norm;
		q[1] /= norm;
		q[2] /= norm;
		q[3] /= norm;
	}
}

static void quat_multiply(float result[4], const float q1[4], const float q2[4]) {
	result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

static void quat_rotate_vector(float result[3], const float q[4], const float v[3]) {
	// Quaternion rotation: v' = q * v * q*
	float qv[4] = {0, v[0], v[1], v[2]};
	float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
	float temp[4];

	quat_multiply(temp, q, qv);
	quat_multiply(qv, temp, q_conj);

	result[0] = qv[1];
	result[1] = qv[2];
	result[2] = qv[3];
}

// ZUPT detection algorithm
static bool detect_zero_velocity(const float gyro[3], const float accel[3], float dt) {
	float gyro_norm = vector_norm(gyro);
	float accel_norm = vector_norm(accel);
	float accel_deviation = fabsf(accel_norm - GRAVITY_NORM);

	// Check if sensor readings are within rest thresholds
	bool gyro_still = gyro_norm < ZUPT_REST_TH_GYR_DEG;
	bool accel_still = accel_deviation < ZUPT_REST_TH_ACC_G;

	if (gyro_still && accel_still) {
		state.rest_timer += dt;
		return state.rest_timer >= ZUPT_REST_MIN_TIME_S;
	} else {
		state.rest_timer = 0.0f;
		return false;
	}
}

// Kalman filter prediction step
static void kalman_predict(const float gyro[3], float dt) {
	// Corrected gyro measurements (subtract bias)
	float corrected_gyro[3]
		= {(gyro[0] - state.gyro_bias[0]) * DEG_TO_RAD,
		   (gyro[1] - state.gyro_bias[1]) * DEG_TO_RAD,
		   (gyro[2] - state.gyro_bias[2]) * DEG_TO_RAD};

	// Integrate quaternion with corrected gyro
	float omega_norm = vector_norm(corrected_gyro);
	if (omega_norm > 1e-6f) {
		float half_angle = 0.5f * omega_norm * dt;
		float sin_half = sinf(half_angle);
		float cos_half = cosf(half_angle);

		float dq[4]
			= {cos_half,
			   sin_half * corrected_gyro[0] / omega_norm,
			   sin_half * corrected_gyro[1] / omega_norm,
			   sin_half * corrected_gyro[2] / omega_norm};

		float new_quat[4];
		quat_multiply(new_quat, state.quat, dq);
		vector_copy(state.quat, new_quat);
		quat_normalize(state.quat);
	}

	// Update process noise covariance
	float dt2 = dt * dt;
	float gyro_var = KALMAN_Q_GYRO_VAR * DEG_TO_RAD * DEG_TO_RAD * dt2;
	float bias_var = KALMAN_Q_BIAS_VAR * DEG_TO_RAD * DEG_TO_RAD * dt2;

	// Add process noise to diagonal elements
	for (int i = 0; i < 3; i++) {
		state.P[i][i] += gyro_var;  // Orientation uncertainty
		state.P[i + 3][i + 3] += bias_var;  // Bias uncertainty
	}
}

// Kalman filter update step with accelerometer
static void kalman_update_accel(const float accel[3]) {
	// Expected gravity vector in body frame
	float gravity_body[3] = {0, 0, GRAVITY_NORM};
	float gravity_expected[3];

	// Rotate expected gravity to body frame using current quaternion
	float q_conj[4] = {state.quat[0], -state.quat[1], -state.quat[2], -state.quat[3]};
	quat_rotate_vector(gravity_expected, q_conj, gravity_body);

	// Innovation (measurement residual)
	float innovation[3]
		= {accel[0] - gravity_expected[0],
		   accel[1] - gravity_expected[1],
		   accel[2] - gravity_expected[2]};

	// Measurement noise covariance
	float R = KALMAN_R_ACCEL_VAR;

	// Simplified Kalman gain calculation (assuming small angles)
	float K_orientation = 0.1f;  // Gain for orientation correction

	// Update orientation using innovation
	float correction[3]
		= {K_orientation * innovation[0],
		   K_orientation * innovation[1],
		   K_orientation * innovation[2]};

	// Convert correction to quaternion update
	float correction_norm = vector_norm(correction);
	if (correction_norm > 1e-6f) {
		float half_angle = 0.5f * correction_norm;
		float sin_half = sinf(half_angle);
		float cos_half = cosf(half_angle);

		float dq[4]
			= {cos_half,
			   sin_half * correction[0] / correction_norm,
			   sin_half * correction[1] / correction_norm,
			   sin_half * correction[2] / correction_norm};

		float new_quat[4];
		quat_multiply(new_quat, state.quat, dq);
		vector_copy(state.quat, new_quat);
		quat_normalize(state.quat);
	}

	// Calculate linear acceleration (remove gravity)
	float gravity_current[3];
	quat_rotate_vector(gravity_current, q_conj, gravity_body);

	state.lin_accel[0] = accel[0] - gravity_current[0];
	state.lin_accel[1] = accel[1] - gravity_current[1];
	state.lin_accel[2] = accel[2] - gravity_current[2];
}

// ZUPT update for bias estimation
static void zupt_update_bias(const float gyro[3]) {
	if (state.is_at_rest) {
		// During rest, gyro readings should be close to bias
		float bias_update_gain = 0.01f;  // Conservative gain

		// Update bias estimates
		for (int i = 0; i < 3; i++) {
			state.gyro_bias[i] += bias_update_gain * (gyro[i] - state.gyro_bias[i]);

			// Clip bias to reasonable range
			if (state.gyro_bias[i] > state.bias_clip) {
				state.gyro_bias[i] = state.bias_clip;
			} else if (state.gyro_bias[i] < -state.bias_clip) {
				state.gyro_bias[i] = -state.bias_clip;
			}
		}

		// Reduce bias uncertainty during rest
		for (int i = 3; i < 6; i++) {
			state.P[i][i] *= 0.99f;  // Slowly reduce bias uncertainty
			if (state.P[i][i] < state.bias_sigma_rest * state.bias_sigma_rest) {
				state.P[i][i] = state.bias_sigma_rest * state.bias_sigma_rest;
			}
		}
	} else {
		// Apply bias forgetting during motion
		for (int i = 3; i < 6; i++) {
			state.P[i][i] *= (1.0f + state.bias_forgetting_factor * state.dt);
		}
	}
}

void sensorfusion_init(float g_time, float a_time, float m_time) {
	// Initialize state
	memset(&state, 0, sizeof(MotionSenseState));

	// Set timing
	state.gyro_time = g_time;
	state.accel_time = a_time;
	state.mag_time = m_time;
	state.dt = 1.0f / MAX_SAMPLE_RATE;  // Default timestep

	// Initialize quaternion to identity (no rotation)
	state.quat[0] = 1.0f;  // w
	state.quat[1] = 0.0f;  // x
	state.quat[2] = 0.0f;  // y
	state.quat[3] = 0.0f;  // z

	// Initialize bias estimates to zero
	memset(state.gyro_bias, 0, sizeof(state.gyro_bias));

	// Initialize covariance matrix
	memset(state.P, 0, sizeof(state.P));

	// Set initial uncertainties
	state.bias_clip = ZUPT_BIAS_CLIP_DEG;
	state.bias_sigma_init = ZUPT_BIAS_SIGMA_INIT_DEG;
	state.bias_sigma_rest = ZUPT_BIAS_SIGMA_REST_DEG;
	state.bias_forgetting_factor = 1.0f / ZUPT_BIAS_FORGETTING_TIME_S;

	// Initialize diagonal covariance
	for (int i = 0; i < 3; i++) {
		state.P[i][i] = 0.1f;  // Initial orientation uncertainty
		state.P[i + 3][i + 3] = state.bias_sigma_init
							  * state.bias_sigma_init;  // Initial bias uncertainty
		state.P[i + 6][i + 6] = 0.01f;  // Initial acceleration uncertainty
	}

	// Initialize ZUPT state
	state.rest_timer = 0.0f;
	state.is_at_rest = false;
	state.gyro_sanity = 1;
	state.is_initialized = true;

	LOG_INF(
		"MotionSense ZUPT initialized - g_time: %.3f ms, a_time: %.3f ms",
		g_time * 1000.0f,
		a_time * 1000.0f
	);
}

void sensorfusion_load(const void* data) {
	if (data && state.is_initialized) {
		// Load saved state (bias estimates, etc.)
		const float* saved_data = (const float*)data;

		// Load gyro bias (3 floats)
		memcpy(state.gyro_bias, saved_data, 3 * sizeof(float));

		// Load quaternion (4 floats)
		memcpy(state.quat, saved_data + 3, 4 * sizeof(float));
		quat_normalize(state.quat);

		LOG_INF(
			"MotionSense state loaded - bias: [%.3f, %.3f, %.3f]",
			state.gyro_bias[0],
			state.gyro_bias[1],
			state.gyro_bias[2]
		);
	}
}

void sensorfusion_save(void* data) {
	if (data && state.is_initialized) {
		// Save current state (bias estimates, etc.)
		float* save_data = (float*)data;

		// Save gyro bias (3 floats)
		memcpy(save_data, state.gyro_bias, 3 * sizeof(float));

		// Save quaternion (4 floats)
		memcpy(save_data + 3, state.quat, 4 * sizeof(float));

		LOG_DBG("MotionSense state saved");
	}
}

void sensorfusion_update_gyro(float* g, float time) {
	if (!state.is_initialized) {
		return;
	}

	state.dt = time;

	// Update gyro sanity check
	float gyro_norm = vector_norm(g);
	state.last_gyro_norm = gyro_norm;

	// Check for reasonable gyro values
	if (gyro_norm > 2000.0f) {  // > 2000 deg/s is likely an error
		state.gyro_sanity = -1;
		return;
	} else {
		state.gyro_sanity = 1;
	}

	// Perform Kalman prediction step
	kalman_predict(g, time);

	LOG_DBG(
		"Gyro update: [%.2f, %.2f, %.2f] deg/s, dt: %.3f ms",
		g[0],
		g[1],
		g[2],
		time * 1000.0f
	);
}

void sensorfusion_update_accel(float* a, float time) {
	if (!state.is_initialized) {
		return;
	}

	state.dt = time;
	state.last_accel_norm = vector_norm(a);

	// Perform Kalman update step
	kalman_update_accel(a);

	LOG_DBG(
		"Accel update: [%.3f, %.3f, %.3f] g, norm: %.3f",
		a[0],
		a[1],
		a[2],
		state.last_accel_norm
	);
}

void sensorfusion_update_mag(float* m, float time) {
	if (!state.is_initialized) {
		return;
	}

	// Magnetometer update not implemented in this ZUPT version
	// Could be added for heading correction
	LOG_DBG("Mag update: [%.2f, %.2f, %.2f] (not used in ZUPT)", m[0], m[1], m[2]);
}

void sensorfusion_update(float* g, float* a, float* m, float time) {
	if (!state.is_initialized) {
		return;
	}

	state.dt = time;

	// Detect zero velocity state
	state.is_at_rest = detect_zero_velocity(g, a, time);

	// Update gyroscope bias using ZUPT
	zupt_update_bias(g);

	// Perform sensor fusion
	kalman_predict(g, time);
	kalman_update_accel(a);

	LOG_DBG(
		"Full update - at_rest: %s, bias: [%.3f, %.3f, %.3f]",
		state.is_at_rest ? "true" : "false",
		state.gyro_bias[0],
		state.gyro_bias[1],
		state.gyro_bias[2]
	);
}

void sensorfusion_get_gyro_bias(float* g_off) {
	if (g_off && state.is_initialized) {
		vector_copy(g_off, state.gyro_bias);
	}
}

void sensorfusion_set_gyro_bias(float* g_off) {
	if (g_off && state.is_initialized) {
		vector_copy(state.gyro_bias, g_off);

		// Update bias uncertainty
		for (int i = 3; i < 6; i++) {
			state.P[i][i] = state.bias_sigma_init * state.bias_sigma_init;
		}

		LOG_INF("Gyro bias set: [%.3f, %.3f, %.3f]", g_off[0], g_off[1], g_off[2]);
	}
}

void sensorfusion_update_gyro_sanity(float* g, float* m) {
	if (!state.is_initialized) {
		return;
	}

	float gyro_norm = vector_norm(g);

	// Update sanity based on gyro magnitude and consistency
	if (gyro_norm > 2000.0f || isnan(gyro_norm)) {
		state.gyro_sanity = -1;  // Failed
	} else if (gyro_norm < 0.01f) {
		state.gyro_sanity = 0;  // Questionable (too low)
	} else {
		state.gyro_sanity = 1;  // Good
	}
}

int sensorfusion_get_gyro_sanity(void) { return state.gyro_sanity; }

void sensorfusion_get_lin_a(float* lin_a) {
	if (lin_a && state.is_initialized) {
		vector_copy(lin_a, state.lin_accel);
	}
}

void sensorfusion_get_quat(float* q) {
	if (q && state.is_initialized) {
		// Return quaternion in w, x, y, z format
		q[0] = state.quat[0];  // w
		q[1] = state.quat[1];  // x
		q[2] = state.quat[2];  // y
		q[3] = state.quat[3];  // z
	}
}

const sensor_fusion_t sensor_fusion_motionsense
	= {*sensorfusion_init,
	   *sensorfusion_load,
	   *sensorfusion_save,

	   *sensorfusion_update_gyro,
	   *sensorfusion_update_accel,
	   *sensorfusion_update_mag,
	   *sensorfusion_update,

	   *sensorfusion_get_gyro_bias,
	   *sensorfusion_set_gyro_bias,

	   *sensorfusion_update_gyro_sanity,
	   *sensorfusion_get_gyro_sanity,

	   *sensorfusion_get_lin_a,
	   *sensorfusion_get_quat};
