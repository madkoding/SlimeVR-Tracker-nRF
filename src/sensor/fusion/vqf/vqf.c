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
#include <stdbool.h>  // Required by vqf.h for bool type

#include "../src/vqf.h"  // conflicting with vqf.h in local path

#include "../vqf/vqf.h"  // conflicting with vqf.h in vqf-c
#include "globals.h"
#include "sensor/sensors_enum.h"
#include "util.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

static uint8_t imu_id;

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;

static float last_a[3] = {0};

void vqf_update_sensor_ids(int imu) { imu_id = imu; }

static void set_params() {
	init_params(&params);

	// Optimized for ICM-45686 with IMPERFECT "zero" calibration
	// Zero calibration limitations: no temp compensation, no misalignment correction,
	// short sampling (3s) Strategy: More conservative bias estimation to handle
	// residual calibration errors

	// Bias estimation - balanced between correction speed and stability
	params.biasClip = 2.5f;
	params.biasForgettingTime = 120.0f;
	params.biasSigmaInit
		= 2.0f;  // Increased from 1.5 - lower initial confidence (imperfect cal)
	params.biasSigmaMotion = 0.25f;  // Increased from 0.2 - more cautious during motion
	params.biasSigmaRest = 0.05f;  // Increased from 0.04 - balanced rest estimation
	params.biasVerticalForgettingFactor
		= 0.008f;  // Increased from 0.005 - handle vertical drift better

	// Enable both bias estimation methods - critical with imperfect calibration
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;

	// Rest detection - more relaxed to handle residual noise from calibration errors
	params.restFilterTau = 1.0f;  // Slightly increased from 0.9 - smoother filtering
	params.restMinT = 2.0f;  // Increased from 1.8 - more stable rest detection
	params.restThGyr = 1.2f;  // Increased from 1.0 - account for residual gyro bias
	params.restThAcc = 1.1f;  // Increased from 1.0 - account for misalignment

	// Accelerometer fusion - compensates for potential accel/gyro misalignment
	// tauAcc controls trust in accelerometer vs gyroscope integration
	// Higher values = more trust in gyro, less sensitive to accel misalignment/noise
	// Recommended range: 3.0-5.0 (default VQF: 3.0)
	// Current: 3.5 - balanced compromise for ICM-45686 without 6-side calibration
	// To reduce misalignment sensitivity further, increase to 4.0-4.5
	params.tauAcc = 3.5f;

	// Magnetometer fusion (QMC6309 + no hard-iron calibration)
	params.tauMag = 10.0f;  // Keep conservative for QMC6309
}

void vqf_init(float g_time, float a_time, float m_time) {
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
}

void vqf_load(const void* data) {
	if (data == NULL) {
		return;
	}
	set_params();
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t*)data + sizeof(state), sizeof(coeffs));
}

void vqf_save(void* data) {
	if (data == NULL) {
		return;
	}
	memcpy(data, &state, sizeof(state));
	memcpy((uint8_t*)data + sizeof(state), &coeffs, sizeof(coeffs));
}

void vqf_update_gyro(float* g, float time) {
	if (g == NULL) {
		return;
	}
	// TODO: time unused?
	// g is in deg/s, convert to rad/s (optimized with direct assignment)
	float g_rad[3];
	g_rad[0] = g[0] * DEG_TO_RAD;
	g_rad[1] = g[1] * DEG_TO_RAD;
	g_rad[2] = g[2] * DEG_TO_RAD;
	updateGyr(&params, &state, &coeffs, g_rad);
}

void vqf_update_accel(float* a, float time) {
	if (a == NULL) {
		return;
	}
	// TODO: time unused?
	// TODO: how to handle change in sample rate
	// a is in g, convert to m/s^2 (optimized with direct assignment)
	float a_m_s2[3];
	a_m_s2[0] = a[0] * CONST_EARTH_GRAVITY;
	a_m_s2[1] = a[1] * CONST_EARTH_GRAVITY;
	a_m_s2[2] = a[2] * CONST_EARTH_GRAVITY;
	if (a_m_s2[0] != 0.0f || a_m_s2[1] != 0.0f || a_m_s2[2] != 0.0f) {
		last_a[0] = a_m_s2[0];
		last_a[1] = a_m_s2[1];
		last_a[2] = a_m_s2[2];
	}
	updateAcc(&params, &state, &coeffs, a_m_s2);
}

void vqf_update_mag(float* m, float time) {
	if (m == NULL) {
		return;
	}
	// TODO: time unused?
	updateMag(&params, &state, &coeffs, m);
}

void vqf_update(float* g, float* a, float* m, float time) {
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	if (g != NULL
		&& (g[0] != 0.0f || g[1] != 0.0f || g[2] != 0.0f)) {  // ignore zeroed gyro
		vqf_update_gyro(g, time);
	}
	if (a != NULL) {
		vqf_update_accel(a, time);
	}
	if (m != NULL) {
		vqf_update_mag(m, time);
	}
}

void vqf_get_gyro_bias(float* g_off) {
	if (g_off == NULL) {
		return;
	}
	getBiasEstimate(&state, &coeffs, g_off);
}

void vqf_set_gyro_bias(float* g_off) {
	if (g_off == NULL) {
		return;
	}
	setBiasEstimate(&state, g_off, -1);
}

void vqf_update_gyro_sanity(float* g, float* m) {
	// TODO: does vqf tell us a "recovery state"
	return;
}

int vqf_get_gyro_sanity(void) {
	// TODO: does vqf tell us a "recovery state"
	return 0;
}

void vqf_get_lin_a(float* lin_a) {
	if (lin_a == NULL) {
		return;
	}
	float q[4];
	vqf_get_quat(q);

	// Calculate gravity vector from quaternion (optimized)
	float vec_gravity[3];
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

	// Remove gravity to get linear acceleration (using last_a instead of
	// state.lastAccLp)
	lin_a[0] = last_a[0] - vec_gravity[0] * CONST_EARTH_GRAVITY;
	lin_a[1] = last_a[1] - vec_gravity[1] * CONST_EARTH_GRAVITY;
	lin_a[2] = last_a[2] - vec_gravity[2] * CONST_EARTH_GRAVITY;
}

void vqf_get_quat(float* q) {
	if (q == NULL) {
		return;
	}
	getQuat9D(&state, q);
}

bool vqf_get_rest_detected(void) { return getRestDetected(&state); }

void vqf_get_relative_rest_deviations(float* out) {
	if (out == NULL) {
		return;
	}
	getRelativeRestDeviations(&params, &state, out);
}

const sensor_fusion_t sensor_fusion_vqf
	= {*vqf_init,
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
	   *vqf_get_quat};
