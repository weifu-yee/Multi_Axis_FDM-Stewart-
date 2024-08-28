#include "stewart_platform.h"
#include "mainpp.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 輔助函數：將弧度轉換為0.01度的整數
double rad_to_centidegree(double rad) {
    return (double)(rad * 180.0 / PI * 100.0);
}

// 輔助函數：將度轉換為弧度
double deg_to_rad(double deg) {
    return deg * PI / 18000.0;  // deg is in 0.01 degree
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

void initialize_platform(Vector3D p[6],
                         Vector3D b[6]) {
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
                   const Vector3D p[6],
                   const Vector3D b[6],
                   double lengths[6]) {
    double pRb[3][3];
    rotation_matrix(platform, pRb);
    Vector3D T = {platform->disp.x, platform->disp.y, platform->disp.z};
    for (int i = 0; i < 6; ++i) {
        lengths[i] = calculate_length(&T, pRb, &p[i], &b[i]);
    }
}
