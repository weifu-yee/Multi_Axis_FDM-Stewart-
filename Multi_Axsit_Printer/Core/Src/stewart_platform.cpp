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
SPVelocity Velo = create_default_stewart_velocity();
double current_lengths[6], next_lengths[6];
Vector3D T;

double deg_to_rad(double deg) {
    return deg * PI / 18000.0;  // deg is in 0.01 degree
}

void update_from_sensor(void) {
	for (int i = 0; i < 6; ++i) {
//		double delta = (double)pusher[i].enc * PI * Lead
//				/ (4 * resolution * reduction_ratio);
		double delta = (double)pusher[i].enc
				/ (4 * pulse_per_mm);
		current_lengths[i] += delta;
		pusher[i].insVel = delta * FREQUENCY;
	}
}

void fake_update_from_sensor(void) {
	for (int i = 0; i < 6; ++i) {
		double delta = pusher[i].pulse / 100;
		if (pusher[i].u >= 0)
			current_lengths[i] += delta;
		else
			current_lengths[i] -= delta;
		pusher[i].insVel = delta * FREQUENCY;
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

void calculate_diff_lengths(double diff_lengths[6]) {
	for (int i = 0; i < 6; ++i) {
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
    for (int i = 0; i < 6; ++i) {
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

double SPerror;
double prev_SPerror = 0;
int SPerror_increasing_count = 0;

bool same_SPPose(const SPPose *pose1, const SPPose *pose2) {
	SPerror = calculate_disp_error(pose1, pose2);
	double dt = (double)1 / FREQUENCY;
	extern double F;

	if (fabs(SPerror) > fabs(prev_SPerror)) {
		SPerror_increasing_count++;
	} else {
		SPerror_increasing_count = 0;
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
    platform.z = 0;
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
    double phi = deg_to_rad(platform->phi);
    double theta = deg_to_rad(platform->theta);
    double psi = deg_to_rad(platform->psi);
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
    T = {platform->x, platform->y, platform->z};
    for (int i = 0; i < 6; ++i) {
        lengths[i] = calculate_length(&T, pRb, &p[i], &b[i]);
    }
}
