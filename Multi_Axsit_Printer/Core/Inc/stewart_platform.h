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

extern Vector3D p[6], b[6];
extern SPPose current;
extern SPPose next;
extern SPPose target;
extern double current_lengths[6], next_lengths[6];

SPPose create_default_stewart_platform();
void update_from_sensor(void);
void fake_update_from_sensor(void);
void presume_next(void);
void calculate_diff_lengths(double diff_lengths[6]);
void assignSPPose(SPPose *dest, const SPPose *src);
double calculateNorm(const double *vec);
bool same_SPPose(const SPPose *pose1, const SPPose *pose2);
void initialize_platform(void);
void calculate_leg(const SPPose* platform,
                   double lengths[6]);

#ifdef __cplusplus
}
#endif

#endif // STEWART_PLATFORM_H
