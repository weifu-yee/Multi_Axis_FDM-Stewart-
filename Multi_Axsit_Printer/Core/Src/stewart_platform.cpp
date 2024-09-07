#include "stewart_platform.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "constants.h"
#include "control.h"

Vector3D p[6], b[6];
SPPose current = create_default_stewart_platform();
SPPose next = create_default_stewart_platform();
SPPose target = create_default_stewart_platform();
double current_lengths[6], next_lengths[6];

void init_lengths_array(double *vec) {
	for (int i = 0; i < 6; ++i) {
		vec[i] = 0.0;
	}
}

double deg_to_rad(double deg) {
    return deg * PI / 18000.0;  // deg is in 0.01 degree
}

void update_from_sensor(void) {
	for (int i = 0; i < 6; ++i) {
		double delta = (double)pusher[i].enc * PI * Lead
				/ (4 * resolution * reduction_ratio);
		current_lengths[i] += delta;
		pusher[i].insVel = delta * FREQUENCY;
	}
}

void fake_update_from_sensor(void) {
	for (int i = 0; i < 6; ++i) {
		double delta = pusher[i].pulse;
		if (pusher[i].u < 0)
			delta *= -1;
		current_lengths[i] += delta;
		pusher[i].insVel = delta * FREQUENCY;
	}
}

void presume_next(void) {
	double dt = (double) 1.0 / FREQUENCY;
	next.disp.x = (double) current.disp.x + dt * target.velo.x;
	next.disp.y = (double) current.disp.y + dt * target.velo.y;
	next.disp.z = (double) current.disp.z + dt * target.velo.z;
	next.disp.phi = (double) current.disp.phi + dt * target.velo.phi;
	next.disp.theta = (double) current.disp.theta + dt * target.velo.theta;
	next.disp.psi = (double) current.disp.psi + dt * target.velo.psi;
}

void calculate_diff_lengths(double diff_lengths[6]) {
	for (int i = 0; i < 6; ++i) {
		diff_lengths[i] = next_lengths[i] - current_lengths[i];
	}
}

void assignSPPose(SPPose *dest, const SPPose *src) {
    dest->disp.x = src->disp.x;
    dest->disp.y = src->disp.y;
    dest->disp.z = src->disp.z;
    dest->disp.phi = src->disp.phi;
    dest->disp.theta = src->disp.theta;
    dest->disp.psi = src->disp.psi;

    dest->velo.x = src->velo.x;
    dest->velo.y = src->velo.y;
    dest->velo.z = src->velo.z;
    dest->velo.phi = src->velo.phi;
    dest->velo.theta = src->velo.theta;
    dest->velo.psi = src->velo.psi;
}

double calculateNorm(const double *vec) {
    double sum = 0.0;
    for (int i = 0; i < 6; ++i) {
        sum += vec[i] * vec[i];
    }
    return sqrt(sum);
}

double calculate_disp_error(const SPPose* pose1, const SPPose* pose2) {
    double error = 0.0;

    // Accumulate the absolute differences for all 6 components
    error += fabs(pose1->disp.x - pose2->disp.x);
    error += fabs(pose1->disp.y - pose2->disp.y);
    error += fabs(pose1->disp.z - pose2->disp.z);
    error += fabs(pose1->disp.phi - pose2->disp.phi);
    error += fabs(pose1->disp.theta - pose2->disp.theta);
    error += fabs(pose1->disp.psi - pose2->disp.psi);

    return error;
}

bool same_SPPose(const SPPose *pose1, const SPPose *pose2) {
	double error = calculate_disp_error(pose1, pose2);
	double dt = 1 / FREQUENCY;
	extern double F;
	if (fabs(error) < fabs(0.8 * F * dt))
		return true;
	return false;
}

SPPose create_default_stewart_platform() {
    SPPose platform;
    // 初始化位移部分
    platform.disp.x = 0;
    platform.disp.y = 0;
    platform.disp.z = 0;
    platform.disp.phi = 0;
    platform.disp.theta = 0;
    platform.disp.psi = 0;
    // 初始化速度部分
    platform.velo.x = 0;
    platform.velo.y = 0;
    platform.velo.z = 0;
    platform.velo.phi = 0;
    platform.velo.theta = 0;
    platform.velo.psi = 0;
    return platform;
}

void initialize_platform(void) {
	const double p_angles[] = P_ANGLES;
	const double b_angles[] = B_ANGLES;

    for (int i = 0; i < 6; ++i) {
        double p_rad = deg_to_rad(p_angles[i] * 100);
        double b_rad = deg_to_rad(b_angles[i] * 100);

        p[i].x = (double)(P_L * cos(p_rad));
        p[i].y = (double)(P_L * sin(p_rad));
        p[i].z = 0;

        b[i].x = (double)(B_L * cos(b_rad));
        b[i].y = (double)(B_L * sin(b_rad));
        b[i].z = 0;
    }
}

void rotation_matrix(const SPPose* platform, double pRb[3][3]) {
    double phi = deg_to_rad(platform->disp.phi);
    double theta = deg_to_rad(platform->disp.theta);
    double psi = deg_to_rad(platform->disp.psi);
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

void calculate_leg(const SPPose* platform,
                   double lengths[6]) {
    double pRb[3][3];
    rotation_matrix(platform, pRb);
    Vector3D T = {platform->disp.x, platform->disp.y, platform->disp.z};
    for (int i = 0; i < 6; ++i) {
        lengths[i] = calculate_length(&T, pRb, &p[i], &b[i]);
    }
}
