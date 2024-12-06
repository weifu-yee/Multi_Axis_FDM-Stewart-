#include "stewart_platform.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "constants.h"
#include "control.h"
#include "timing.h"

Vector3D p[7], b[7];
SPPose current = create_default_stewart_platform();
SPPose next = create_default_stewart_platform();
SPPose target = create_default_stewart_platform();
SPVelocity Velo = create_default_stewart_velocity();
double current_lengths[7], next_lengths[7];
Vector3D T;

double deg_to_rad(double deg) {
    return deg * PI / 180.0;
}
void update_from_sensor(void) {
	for (int i = 1; i <= 6; ++i) {
		double delta = (double)pusher[i].enc
				/ (4 * pulse_per_mm);
		delta *= -1.0;
		current_lengths[i] += delta;
		pusher[i].insVel = delta * FREQUENCY;
	}
}
void fake_update_from_sensor(void) {
	for (int i = 1; i <= 6; ++i) {
		double delta = pusher[i].pulse / 100.0;
		if (pusher[i].dir >= 0)
			current_lengths[i] += delta;
		else
			current_lengths[i] -= delta;
		pusher[i].insVel = delta * (double)FREQUENCY;
	}
}
void presume_next(void) {
	double dt = (double)1 / FREQUENCY;
	next.x = (double)(current.x + (double)(dt * Velo.x));
	next.y = (double)current.y + dt * Velo.y;
	next.z = (double)current.z + dt * Velo.z;
	next.phi = (double)current.phi + dt * Velo.phi;
	next.theta = (double)current.theta + dt * Velo.theta;
	next.psi = (double)current.psi + dt * Velo.psi;
}
void calculate_diff_lengths(double diff_lengths[7]) {
	for (int i = 1; i <= 6; ++i) {
		diff_lengths[i] = next_lengths[i] - current_lengths[i];
	}
}
void assignSPPose(SPPose *dest, const SPPose *src) {
    dest->x = src->x;
    dest->y = src->y;
    dest->z = src->z;
    dest->phi = src->phi;
    dest->theta = src->theta;
    dest->psi = src->psi;
}
double calculateNorm(const double *vec) {
    double sum = 0.0;
    for (int i = 1; i <= 6; ++i) {
        sum += vec[i] * vec[i];
    }
    return sqrt(sum);
}
double calculate_disp_error(const SPPose* pose1, const SPPose* pose2) {
    double error = 0.0;

    // Accumulate the absolute differences for all 6 components
    error += fabs(pose1->x - pose2->x);
    error += fabs(pose1->y - pose2->y);
    error += fabs(pose1->z - pose2->z);
//    error += fabs(pose1->phi - pose2->phi);
//    error += fabs(pose1->theta - pose2->theta);
//    error += fabs(pose1->psi - pose2->psi);

    double phi_diff = pose1->phi - pose2->phi;
	phi_diff = fmod(phi_diff + 3*M_PI, 2*M_PI) - M_PI;
	error += fabs(phi_diff);

	double theta_diff = pose1->theta - pose2->theta;
	theta_diff = fmod(theta_diff + 3*M_PI, 2*M_PI) - M_PI;
	error += fabs(theta_diff);

	double psi_diff = pose1->psi - pose2->psi;
	psi_diff = fmod(psi_diff + 3*M_PI, 2*M_PI) - M_PI;
	error += fabs(psi_diff);

    return error;
}


// 用來記錄上次輸出的狀態
int last_pulse[7] = {0};

// 緩啟動與緩煞車限制函數
void limit_pusher_change(void) {
    for (int i = 1; i <= 6; ++i) {
    	int pulse_w_sign = pusher[i].goal_pulse * pusher[i].dir;
        int delta_pulse = pulse_w_sign - last_pulse[i];
		if (delta_pulse > MAX_DELTA) {
			pulse_w_sign = last_pulse[i] + MAX_DELTA;
			pusher[i].pulse = (double)abs(pulse_w_sign);
		} else if (delta_pulse < -MAX_DELTA) {
			pulse_w_sign = last_pulse[i] - MAX_DELTA;
			pusher[i].pulse = (double)abs(pulse_w_sign);
		}
		pusher[i].dir = (pulse_w_sign > 0) - (pulse_w_sign < 0);
        last_pulse[i] = pulse_w_sign;
    }
}
// 調用 limit_pusher_change 函數來限制變化量
void actuate_pushers_with_smooth(void) {
    limit_pusher_change();  // 限制變化量
    actuate_pushers();      // 執行推進器控制
}



double SPerror;
double prev_SPerror = 0;
int SPerror_increasing_count = 0;

bool same_SPPose(const SPPose *pose1, const SPPose *pose2) {
	SPerror = calculate_disp_error(pose1, pose2);
	double dt = (double)1 / FREQUENCY;
	extern double F;

	if (fabs(SPerror) >= fabs(prev_SPerror)) {
		SPerror_increasing_count++;
	} else if (SPerror_increasing_count > 0) {
		SPerror_increasing_count --;
	}

	prev_SPerror = SPerror;

//	if (fabs(SPerror) < fabs(0.8 * F * dt))
//		return true;
	if (fabs(SPerror) < fabs(0.8 * F * dt) || SPerror_increasing_count >= SPerror_TREND_THRESHOLD) {
		return true;
	}
	return false;
}
SPPose create_default_stewart_platform() {
    SPPose platform;
    platform.x = 0;
    platform.y = 0;
    platform.z = Ho;
    platform.phi = 0;
    platform.theta = 0;
    platform.psi = 0;
    return platform;
}
SPVelocity create_default_stewart_velocity() {
    SPVelocity Velo;
    Velo.x = 0;
    Velo.y = 0;
    Velo.z = 0;
    Velo.phi = 0;
    Velo.theta = 0;
    Velo.psi = 0;
    return Velo;
}
void initialize_platform(void) {
	const double p_angles[] = P_ANGLES;
	const double b_angles[] = B_ANGLES;

    for (int i = 1; i <= 6; ++i) {
        double p_rad = deg_to_rad(p_angles[i]);
        double b_rad = deg_to_rad(b_angles[i]);

        p[i].x = (double)(P_L * cos(p_rad));
        p[i].y = (double)(P_L * sin(p_rad));
        p[i].z = 0;

        b[i].x = (double)(B_L * cos(b_rad));
        b[i].y = (double)(B_L * sin(b_rad));
        b[i].z = 0;
    }
}
void rotation_matrix(const SPPose* platform, double pRb[3][3]) {
//    double phi = deg_to_rad(platform->phi);
//    double theta = deg_to_rad(platform->theta);
//    double psi = deg_to_rad(platform->psi);
    double phi = platform->phi;
    double theta = platform->theta;
    double psi = platform->psi;
    double cphi = cos(phi), sphi = sin(phi);
    double ctheta = cos(theta), stheta = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);
    pRb[0][0] = ctheta * cpsi;
    pRb[0][1] = -ctheta * spsi;
    pRb[0][2] = stheta;
    pRb[1][0] = sphi * stheta * cpsi + cphi * spsi;
    pRb[1][1] = -sphi * stheta * spsi + cphi * cpsi;
    pRb[1][2] = -sphi * ctheta;
    pRb[2][0] = -cphi * stheta * cpsi + sphi * spsi;
    pRb[2][1] = cphi * stheta * spsi + sphi * cpsi;
    pRb[2][2] = cphi * ctheta;
}
double calculate_length(const Vector3D* T, const double pRb[3][3],
                         const Vector3D* p, const Vector3D* b) {
    double dx = T->x + pRb[0][0] * p->x + pRb[0][1] * p->y + pRb[0][2] * p->z - b->x;
    double dy = T->y + pRb[1][0] * p->x + pRb[1][1] * p->y + pRb[1][2] * p->z - b->y;
    double dz = T->z + pRb[2][0] * p->x + pRb[2][1] * p->y + pRb[2][2] * p->z - b->z;
    return (double)sqrt(dx*dx + dy*dy + dz*dz);
}

double pRb[3][3];

void calculate_leg(const SPPose* platform,
                   double lengths[7]) {
//    double pRb[3][3];
    rotation_matrix(platform, pRb);
    T = {platform->x, platform->y, platform->z};
    for (int i = 1; i <= 6; ++i) {
        lengths[i] = calculate_length(&T, pRb, &p[i], &b[i]);
    }
}
void angularNormalizer(double *ang) {
	*ang = (double) fmod(*ang + M_PI, 2*M_PI) - M_PI;
}
void update_parameters(const SPPose* target_pose, double F) {
    // 更新目標位置
    target.x = target_pose->x;
    target.y = target_pose->y;
    target.z = target_pose->z;
    target.phi = target_pose->phi;
    target.theta = target_pose->theta;
    target.psi = target_pose->psi;

    // 計算位置差異
    double dx = target_pose->x - current.x;
    double dy = target_pose->y - current.y;
    double dz = target_pose->z - current.z;
    double total_distance = sqrt(dx * dx + dy * dy + dz * dz);
    double time = total_distance / F;

    // 計算線速度
    Velo.x = dx / time;
    Velo.y = dy / time;
    Velo.z = dz / time;

    // 計算角度差異
    double dphi = target_pose->phi - current.phi;
    double dtheta = target_pose->theta - current.theta;
    double dpsi = target_pose->psi - current.psi;

    // 正規化角度差異至 [-π, π]
    dphi = fmod(dphi + M_PI, 2 * M_PI) - M_PI;
    dtheta = fmod(dtheta + M_PI, 2 * M_PI) - M_PI;
    dpsi = fmod(dpsi + M_PI, 2 * M_PI) - M_PI;

    // 計算角速度
    Velo.phi = dphi / time;
    Velo.theta = dtheta / time;
    Velo.psi = dpsi / time;
}
