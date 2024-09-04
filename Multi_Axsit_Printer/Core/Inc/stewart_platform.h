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


void update_from_sensor(void);
void presume_next(void);
void calculate_diff_lengths(double diff_lengths[6]);
void assignSPPose(SPPose *dest, const SPPose *src);
double calculateNorm(const double *vec);

SPPose create_default_stewart_platform();
void update_SPPose(SPPose* platform, double x, double y, double z,
                       double phi, double theta, double psi,
                       double vx, double vy, double vz,
                       double vphi, double vtheta, double vpsi);
void updatePoseIntegral(SPPose *current);
SPPose calculate_difference(const SPPose *current, const SPPose *target);
double totalDiff(const SPPose *current, const SPPose *target);
void initialize_platform(void);
void calculate_leg(const SPPose* platform,
                   double lengths[6]);

#ifdef __cplusplus
}
#endif

#endif // STEWART_PLATFORM_H
