#include "stewart_platform.h"
#include "mainpp.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


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

void presume_next(void) {
	double dt = 1 / FREQUENCY;
	next.disp.x = current.disp.x + dt * target.velo.x;
	next.disp.y = current.disp.y + dt * target.velo.y;
	next.disp.z = current.disp.z + dt * target.velo.z;
	next.disp.phi = current.disp.phi + dt * target.velo.phi;
	next.disp.theta = current.disp.theta + dt * target.velo.theta;
	next.disp.psi = current.disp.psi + dt * target.velo.psi;
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

SPPose calculate_difference(const SPPose *current, const SPPose *target) {
    SPPose difference;
    difference.disp.x = target->disp.x - current->disp.x;
    difference.disp.y = target->disp.y - current->disp.y;
    difference.disp.z = target->disp.z - current->disp.z;
    difference.disp.phi = target->disp.phi - current->disp.phi;
    difference.disp.theta = target->disp.theta - current->disp.theta;
    difference.disp.psi = target->disp.psi - current->disp.psi;
    difference.velo.x = target->velo.x - current->velo.x;
    difference.velo.y = target->velo.y - current->velo.y;
    difference.velo.z = target->velo.z - current->velo.z;
    difference.velo.phi = target->velo.phi - current->velo.phi;
    difference.velo.theta = target->velo.theta - current->velo.theta;
    difference.velo.psi = target->velo.psi - current->velo.psi;
    return difference;
}

double totalDiff(const SPPose *current, const SPPose *target) {
    // Displacement differences (x, y, z)
    double dx = target->disp.x - current->disp.x;
    double dy = target->disp.y - current->disp.y;
    double dz = target->disp.z - current->disp.z;
    // Calculate the Euclidean norm (distance) for displacement
    double displacement_norm = sqrt(dx * dx + dy * dy + dz * dz);

    // Angular differences (phi, theta, psi)
    double dphi = target->disp.phi - current->disp.phi;
    double dtheta = target->disp.theta - current->disp.theta;
    double dpsi = target->disp.psi - current->disp.psi;
    // Normalize angular differences if necessary
    double angular_norm = sqrt(dphi * dphi + dtheta * dtheta + dpsi * dpsi);

    // Combine displacement and angular differences
    // Weight the angular part if needed, or leave it as is
    double total_diff = displacement_norm + angular_norm * ANG_NORM_WEIGHT;
    return total_diff;
}

void updatePoseIntegral(SPPose *current) {
    const double dt = 1.0 / FREQUENCY;
    current->disp.x += current->velo.x * dt;
    current->disp.y += current->velo.y * dt;
    current->disp.z += current->velo.z * dt;
    current->disp.phi += current->velo.phi * dt;
    current->disp.theta += current->velo.theta * dt;
    current->disp.psi += current->velo.psi * dt;
    // Normalize angles to [-π, π]
    current->disp.phi = fmod(current->disp.phi + M_PI, 2*M_PI) - M_PI;
    current->disp.theta = fmod(current->disp.theta + M_PI, 2*M_PI) - M_PI;
    current->disp.psi = fmod(current->disp.psi + M_PI, 2*M_PI) - M_PI;
}

void update_SPPose(SPPose* platform, double x, double y, double z,
                       double phi, double theta, double psi,
                       double vx, double vy, double vz,
                       double vphi, double vtheta, double vpsi) {
    platform->disp.x = x;
    platform->disp.y = y;
    platform->disp.z = z;
    platform->disp.phi = phi;
    platform->disp.theta = theta;
    platform->disp.psi = psi;
    platform->velo.x = vx;
    platform->velo.y = vy;
    platform->velo.z = vz;
    platform->velo.phi = vphi;
    platform->velo.theta = vtheta;
    platform->velo.psi = vpsi;
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
