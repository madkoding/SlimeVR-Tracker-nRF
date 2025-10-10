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
#include "../src/vqf.h"  // conflicting with vqf.h in local path

#include <math.h>

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

static float sigma_from_noise_density(float density, float sample_rate) {
	if (sample_rate <= 0.0f) {
		return 0.0f;
	}

	return density * sqrtf(sample_rate * 0.5f);
}

static float gyro_random_walk_per_sqrt_second(float arw_dps_sqrt_hr) {
	return arw_dps_sqrt_hr / sqrtf(3600.0f);
}

static void apply_icm45686_zero_calibration(vqf_params_t* tuned) {
	/*
	 * Noise and bias characteristics taken from DS-000442 (ICM-45686 datasheet).
	 * The goal is to keep rest detection and bias tracking grounded in the
	 * sensor's specified behaviour when only a zero calibration is present.
	 */
	const float gyro_noise_density_dps = 0.0035f;  // °/s/√Hz, LN mode
	const float accel_noise_density_g = 70e-6f;  // g/√Hz, LN mode
	const float gyro_bias_instability_dps = 0.5f;  // 1σ, zero-rate output
	const float gyro_bias_bound_dps = 3.0f;  // worst-case residual bias
	const float gyro_arw_dps_sqrt_hr = 0.23f;  // angular random walk
	const float accel_bias_bound_g = 0.03f;  // 30 mg residual bias

	const float accel_rate = CONFIG_SENSOR_ACCEL_ODR;
	const float gyro_rate = CONFIG_SENSOR_GYRO_ODR;

	const float gyro_sigma_dps
		= sigma_from_noise_density(gyro_noise_density_dps, gyro_rate);
	const float accel_sigma_g
		= sigma_from_noise_density(accel_noise_density_g, accel_rate);
	const float gyro_bias_rw_dps_sqrt_s
		= gyro_random_walk_per_sqrt_second(gyro_arw_dps_sqrt_hr);

	const float bias_coupling_time
		= accel_bias_bound_g / (gyro_bias_instability_dps * DEG_TO_RAD);
	const float tau_acc = (bias_coupling_time > 0.0f && isfinite(bias_coupling_time))
							? sqrtf(2.0f) * bias_coupling_time
							: tuned->tauAcc;

	tuned->tauAcc = tau_acc;
	tuned->biasClip = gyro_bias_bound_dps;
	tuned->biasSigmaInit = gyro_bias_instability_dps;

	const float rest_min_t = fmaxf(2.5f, tau_acc);
	tuned->restMinT = rest_min_t;
	tuned->restFilterTau = fmaxf(1.5f, 0.5f * rest_min_t);

	const float rest_th_gyr = hypotf(3.0f * gyro_sigma_dps, gyro_bias_instability_dps);
	tuned->restThGyr = fminf(tuned->biasClip, rest_th_gyr);

	const float accel_bias_bound_ms2 = accel_bias_bound_g * CONST_EARTH_GRAVITY;
	const float accel_noise_bound_ms2 = 3.0f * accel_sigma_g * CONST_EARTH_GRAVITY;
	tuned->restThAcc = hypotf(accel_noise_bound_ms2, accel_bias_bound_ms2);

	if (gyro_bias_rw_dps_sqrt_s > 0.0f) {
		const float random_walk_power
			= gyro_bias_rw_dps_sqrt_s * gyro_bias_rw_dps_sqrt_s;
		const float forgetting_time = 0.01f / random_walk_power;
		tuned->biasForgettingTime = fmaxf(tuned->biasForgettingTime, forgetting_time);
	}

	const float rest_samples = fmaxf(1.0f, gyro_rate * rest_min_t);
	const float rest_sigma_from_noise = gyro_sigma_dps / sqrtf(rest_samples);
	const float rest_sigma_from_rw = gyro_bias_rw_dps_sqrt_s * sqrtf(rest_min_t);
	tuned->biasSigmaRest = fmaxf(rest_sigma_from_noise, rest_sigma_from_rw);

	const float motion_window = fmaxf(1.0f, tau_acc);
	const float motion_sigma_from_rw = gyro_bias_rw_dps_sqrt_s * sqrtf(motion_window);
	tuned->biasSigmaMotion = fmaxf(tuned->biasSigmaRest, motion_sigma_from_rw);

	if (tuned->biasForgettingTime > 0.0f) {
		tuned->biasVerticalForgettingFactor
			= fminf(0.1f, 1.0f / (tuned->biasForgettingTime * gyro_rate));
	}
}

static bool has_valid_6_side_calibration(void) {
#if defined(CONFIG_SENSOR_USE_6_SIDE_CALIBRATION) \
	&& CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	if (retained == NULL) {
		return false;
	}

	bool offset_non_zero = false;
	for (int i = 0; i < 3; i++) {
		if (fabsf(retained->accBAinv[0][i]) > 1e-3f) {
			offset_non_zero = true;
			break;
		}
	}

	bool scale_non_identity = false;
	for (int i = 0; i < 3; i++) {
		if (fabsf(retained->accBAinv[i + 1][i] - 1.0f) > 1e-3f) {
			scale_non_identity = true;
			break;
		}
	}

	return offset_non_zero || scale_non_identity;
#else
	return false;
#endif
}

static bool magnetometer_in_use(void) {
#if MAG_ENABLED
	if (retained == NULL) {
		return true;
	}

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 3; col++) {
			if (fabsf(retained->magBAinv[row][col]) > 1e-3f) {
				return true;
			}
		}
	}

	return false;
#else
	return false;
#endif
}

void vqf_update_sensor_ids(int imu) { imu_id = imu; }

static void set_params() {
	init_params(&params);
	params.biasClip = 5.0f;
	params.tauMag = 10.0f;  // best result for VQF from paper
	// best result from optimizer
	params.biasForgettingTime = 136.579346;
	params.biasSigmaInit = 3.219453;
	params.biasSigmaMotion = 0.348501;
	params.biasSigmaRest = 0.063616;
	params.biasVerticalForgettingFactor = 0.007056;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 1.114532;
	params.restMinT = 2.586910;
	params.restThAcc = 1.418598;
	params.restThGyr = 1.399189;
	params.tauAcc = 4.337983;

	bool has_six_side_calibration = has_valid_6_side_calibration();
	bool has_active_magnetometer = magnetometer_in_use();
	bool is_icm45686 = imu_id == IMU_ICM45686;

	if (!has_active_magnetometer) {
		params.tauMag = -1.0f;
		params.magDistRejectionEnabled = false;
	}

	if (!has_six_side_calibration && is_icm45686) {
		apply_icm45686_zero_calibration(&params);
	}
}

void vqf_init(float g_time, float a_time, float m_time) {
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
}

void vqf_load(const void* data) {
	set_params();
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t*)data + sizeof(state), sizeof(coeffs));
}

void vqf_save(void* data) {
	memcpy(data, &state, sizeof(state));
	memcpy((uint8_t*)data + sizeof(state), &coeffs, sizeof(coeffs));
}

void vqf_update_gyro(float* g, float time) {
	// TODO: time unused?
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++) {
		g_rad[i] = g[i] * DEG_TO_RAD;
	}
	updateGyr(&params, &state, &coeffs, g_rad);
}

void vqf_update_accel(float* a, float time) {
	// TODO: time unused?
	// TODO: how to handle change in sample rate
	float a_m_s2[3] = {0};
	// a is in g, convert to m/s^2
	for (int i = 0; i < 3; i++) {
		a_m_s2[i] = a[i] * CONST_EARTH_GRAVITY;
	}
	if (a_m_s2[0] != 0 || a_m_s2[1] != 0 || a_m_s2[2] != 0) {
		memcpy(last_a, a_m_s2, sizeof(a_m_s2));
	}
	updateAcc(&params, &state, &coeffs, a_m_s2);
}

void vqf_update_mag(float* m, float time) {
	// TODO: time unused?
	updateMag(&params, &state, &coeffs, m);
}

void vqf_update(float* g, float* a, float* m, float time) {
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	if (g[0] != 0 || g[1] != 0 || g[2] != 0) {  // ignore zeroed gyro
		vqf_update_gyro(g, time);
	}
	vqf_update_accel(a, time);
	vqf_update_mag(m, time);
}

void vqf_get_gyro_bias(float* g_off) { getBiasEstimate(&state, &coeffs, g_off); }

void vqf_set_gyro_bias(float* g_off) { setBiasEstimate(&state, g_off, -1); }

void vqf_update_gyro_sanity(float* g, float* m) {
	// TODO: does vqf tell us a "recovery state"
	return;
}

int vqf_get_gyro_sanity(void) {
	// TODO: does vqf tell us a "recovery state"
	return 0;
}

void vqf_get_lin_a(float* lin_a) {
	float q[4] = {0};
	vqf_get_quat(q);

	float vec_gravity[3] = {0};
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

	//	float *a = state.lastAccLp; // not usable, rotated by inertial frame
	float* a = last_a;
	for (int i = 0; i < 3; i++) {
		lin_a[i]
			= a[i]
			- vec_gravity[i]
				  * CONST_EARTH_GRAVITY;  // gravity vector to m/s^2 before subtracting
	}
}

void vqf_get_quat(float* q) { getQuat9D(&state, q); }

bool vqf_get_rest_detected(void) { return getRestDetected(&state); }

void vqf_get_relative_rest_deviations(float* out) {
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
