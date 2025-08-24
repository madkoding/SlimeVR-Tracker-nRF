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
#include "ekf.h"

#include <math.h>
#include <string.h>

#include "globals.h"
#include "util.h"

LOG_MODULE_REGISTER(ekf, LOG_LEVEL_INF);

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float bias[3];
static float lin_a[3];
static float P[6][6];
static float m_ref[3] = {1.0f, 0.0f, 0.0f};
static bool m_ref_set;
static int gyro_sanity;
static int imu_id;

static void quat_normalize(float* q) {
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n == 0.0f) {
		q[0] = 1.0f;
		q[1] = q[2] = q[3] = 0.0f;
		return;
	}
	float inv = 1.0f / n;
	q[0] *= inv;
	q[1] *= inv;
	q[2] *= inv;
	q[3] *= inv;
}

static void quat_mult(const float* a, const float* b, float* r) {
	r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void quat_conj(const float* q, float* out) {
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

static void quat_rotate(const float* q, const float* v, float* out) {
	float qv[4] = {0.0f, v[0], v[1], v[2]};
	float tmp[4];
	float qc[4];
	quat_mult(q, qv, tmp);
	quat_conj(q, qc);
	quat_mult(tmp, qc, qv);
	out[0] = qv[1];
	out[1] = qv[2];
	out[2] = qv[3];
}

static void quat_rotate_conj(const float* q, const float* v, float* out) {
	float qc[4];
	quat_conj(q, qc);
	quat_rotate(qc, v, out);
}

static int invert3x3(const float A[3][3], float inv[3][3]) {
	float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
			  - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
			  + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
	if (fabsf(det) < 1e-9f) {
		return -1;
	}
	float invdet = 1.0f / det;
	inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invdet;
	inv[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * invdet;
	inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invdet;
	inv[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * invdet;
	inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invdet;
	inv[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * invdet;
	inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invdet;
	inv[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * invdet;
	inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invdet;
	return 0;
}

static void predict(float* g, float dt) {
	float w[3] = {g[0] - bias[0], g[1] - bias[1], g[2] - bias[2]};
	float dq[4] = {1.0f, 0.5f * w[0] * dt, 0.5f * w[1] * dt, 0.5f * w[2] * dt};
	float qn[4];
	quat_mult(q, dq, qn);
	memcpy(q, qn, sizeof(q));
	quat_normalize(q);

	float F[6][6] = {0};
	float wx = w[0], wy = w[1], wz = w[2];
	F[0][0] = 1.0f;
	F[0][1] = wz * dt;
	F[0][2] = -wy * dt;
	F[0][3] = -dt;
	F[1][0] = -wz * dt;
	F[1][1] = 1.0f;
	F[1][2] = wx * dt;
	F[1][4] = -dt;
	F[2][0] = wy * dt;
	F[2][1] = -wx * dt;
	F[2][2] = 1.0f;
	F[2][5] = -dt;
	F[3][3] = 1.0f;
	F[4][4] = 1.0f;
	F[5][5] = 1.0f;

	float temp[6][6] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			for (int k = 0; k < 6; ++k) {
				temp[i][j] += F[i][k] * P[k][j];
			}
		}
	}

	float Pn[6][6] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			for (int k = 0; k < 6; ++k) {
				Pn[i][j] += temp[i][k] * F[j][k];
			}
		}
	}

	const float q_gyro = 0.001f;
	const float q_bias = 0.0001f;
	for (int i = 0; i < 3; ++i) {
		Pn[i][i] += q_gyro * dt * dt;
	}
	for (int i = 3; i < 6; ++i) {
		Pn[i][i] += q_bias * dt * dt;
	}
	memcpy(P, Pn, sizeof(P));
}

static void update_vector(const float* z, const float* ref, float var) {
	float h[3];
	quat_rotate_conj(q, ref, h);
	float y[3] = {z[0] - h[0], z[1] - h[1], z[2] - h[2]};

	float H[3][6] = {
		{0.0f, -h[2], h[1], 0.0f, 0.0f, 0.0f},
		{h[2], 0.0f, -h[0], 0.0f, 0.0f, 0.0f},
		{-h[1], h[0], 0.0f, 0.0f, 0.0f, 0.0f},
	};

	float HP[3][6] = {0};
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 6; ++j) {
			for (int k = 0; k < 6; ++k) {
				HP[i][j] += H[i][k] * P[k][j];
			}
		}
	}

	float S[3][3] = {0};
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < 6; ++k) {
				S[i][j] += HP[i][k] * H[j][k];
			}
		}
	}
	for (int i = 0; i < 3; ++i) {
		S[i][i] += var;
	}

	float S_inv[3][3];
	if (invert3x3(S, S_inv)) {
		return;
	}

	float PHt[6][3] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < 6; ++k) {
				PHt[i][j] += P[i][k] * H[j][k];
			}
		}
	}

	float K[6][3] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < 3; ++k) {
				K[i][j] += PHt[i][k] * S_inv[k][j];
			}
		}
	}

	float x[6] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int k = 0; k < 3; ++k) {
			x[i] += K[i][k] * y[k];
		}
	}

	float KH[6][6] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			for (int k = 0; k < 3; ++k) {
				KH[i][j] += K[i][k] * H[k][j];
			}
		}
	}

	float I_KH[6][6];
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
		}
	}

	float Pn[6][6] = {0};
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			for (int k = 0; k < 6; ++k) {
				Pn[i][j] += I_KH[i][k] * P[k][j];
			}
		}
	}
	memcpy(P, Pn, sizeof(P));

	float dq[4] = {
		1.0f,
		0.5f * x[0],
		0.5f * x[1],
		0.5f * x[2],
	};
	float qn[4];
	quat_mult(q, dq, qn);
	memcpy(q, qn, sizeof(q));
	quat_normalize(q);
	bias[0] += x[3];
	bias[1] += x[4];
	bias[2] += x[5];
}

void ekf_update_sensor_ids(int imu) { imu_id = imu; }

void ekf_init(float g_time, float a_time, float m_time) {
	(void)g_time;
	(void)a_time;
	(void)m_time;
	q[0] = 1.0f;
	q[1] = q[2] = q[3] = 0.0f;
	memset(bias, 0, sizeof(bias));
	memset(lin_a, 0, sizeof(lin_a));
	memset(P, 0, sizeof(P));
	for (int i = 0; i < 6; ++i) {
		P[i][i] = 0.01f;
	}
	m_ref_set = false;
	gyro_sanity = 0;
}

void ekf_load(const void* data) {
	memcpy(q, data, sizeof(q));
	memcpy(bias, (const uint8_t*)data + sizeof(q), sizeof(bias));
}

void ekf_save(void* data) {
	memcpy(data, q, sizeof(q));
	memcpy((uint8_t*)data + sizeof(q), bias, sizeof(bias));
}

void ekf_update_gyro(float* g, float time) { predict(g, time); }

void ekf_update_accel(float* a, float time) {
	(void)time;
	float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	if (norm == 0.0f) {
		return;
	}
	float z[3] = {a[0] / norm, a[1] / norm, a[2] / norm};
	const float ref[3] = {0.0f, 0.0f, 1.0f};
	update_vector(z, ref, 0.01f);
	float aw[3];
	quat_rotate(q, a, aw);
	lin_a[0] = aw[0];
	lin_a[1] = aw[1];
	lin_a[2] = aw[2] - 1.0f;
}

void ekf_update_mag(float* m, float time) {
	(void)time;
	float norm = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
	if (norm == 0.0f) {
		return;
	}
	float z[3] = {m[0] / norm, m[1] / norm, m[2] / norm};
	if (!m_ref_set) {
		memcpy(m_ref, z, sizeof(z));
		m_ref_set = true;
	}
	update_vector(z, m_ref, 0.01f);
}

void ekf_update(float* g, float* a, float* m, float time) {
	ekf_update_gyro(g, time);
	if (a) {
		ekf_update_accel(a, time);
	}
	if (m) {
		ekf_update_mag(m, time);
	}
}

void ekf_get_gyro_bias(float* g_off) { memcpy(g_off, bias, sizeof(bias)); }

void ekf_set_gyro_bias(float* g_off) { memcpy(bias, g_off, sizeof(bias)); }

void ekf_update_gyro_sanity(float* g, float* m) {
	(void)g;
	(void)m;
	gyro_sanity = 0;
}

int ekf_get_gyro_sanity(void) { return gyro_sanity; }

void ekf_get_lin_a(float* out) { memcpy(out, lin_a, sizeof(lin_a)); }

void ekf_get_quat(float* out) { memcpy(out, q, sizeof(q)); }

const sensor_fusion_t sensor_fusion_ekf = {
	.init = ekf_init,
	.load = ekf_load,
	.save = ekf_save,
	.update_gyro = ekf_update_gyro,
	.update_accel = ekf_update_accel,
	.update_mag = ekf_update_mag,
	.update = ekf_update,
	.get_gyro_bias = ekf_get_gyro_bias,
	.set_gyro_bias = ekf_set_gyro_bias,
	.update_gyro_sanity = ekf_update_gyro_sanity,
	.get_gyro_sanity = ekf_get_gyro_sanity,
	.get_lin_a = ekf_get_lin_a,
	.get_quat = ekf_get_quat,
};
