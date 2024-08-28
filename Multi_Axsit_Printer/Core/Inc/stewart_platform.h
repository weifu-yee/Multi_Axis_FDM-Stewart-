#ifndef STEWART_PLATFORM_H
#define STEWART_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double x;
    double y;
    double z;
} Vector3D;

extern Vector3D p[6];
extern Vector3D b[6];

struct Angle {
    double platform_angle; // 角度 * 100
    double base_angle; // 角度 * 100
};

typedef struct {
    struct {
        double x, y, z;           // 位移 (Displacement)
        double phi, theta, psi;   // 旋轉 (Rotation)
    } disp; // 位移結構
    struct {
        double x, y, z;           // 速度 (Velocity)
        double phi, theta, psi;   // 角速度 (Angular Velocity)
    } velo; // 速度結構
} SPPose;

SPPose create_default_stewart_platform();

void update_SPPose(SPPose* platform, double x, double y, double z,
                       double phi, double theta, double psi,
                       double vx, double vy, double vz,
                       double vphi, double vtheta, double vpsi);

SPPose calculate_difference(const SPPose *current, const SPPose *target);

void update_leg_speeds(SPPose* current, const SPPose* target);

void plan_velocity(const double current_length[6], const double target_length[6], double time_step);

void initialize_platform(Vector3D p[6],
                         Vector3D b[6]);

void calculate_leg(const SPPose* platform,
                   const Vector3D p[6],
                   const Vector3D b[6],
                   double lengths[6]);

#ifdef __cplusplus
}
#endif

#endif // STEWART_PLATFORM_H
