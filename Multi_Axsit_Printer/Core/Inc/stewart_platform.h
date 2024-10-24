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
	double x, y, z;           // 位移 (Displacement)
	double phi, theta, psi;   // 旋轉 (Rotation)
} SPPose;

typedef struct {
	double x, y, z;           // 速度 (Velocity)
	double phi, theta, psi;   // 角速度 (Angular Velocity)
} SPVelocity;

extern Vector3D p[7], b[7];
extern SPPose current;
extern SPPose next;
extern SPPose target;
extern SPVelocity Velo;
extern double current_lengths[7], next_lengths[7];

extern double SPerror;

extern double X, Y, Z, E, F, PHI, THETA, PSI;

void init_lengths_array(double *vec);
SPPose create_default_stewart_platform();
SPVelocity create_default_stewart_velocity();
void update_from_sensor(void);
void fake_update_from_sensor(void);
void presume_next(void);
void calculate_diff_lengths(double diff_lengths[7]);
void assignSPPose(SPPose *dest, const SPPose *src);
double calculateNorm(const double *vec);
bool same_SPPose(const SPPose *pose1, const SPPose *pose2);
void initialize_platform(void);
void calculate_leg(const SPPose* platform,
                   double lengths[7]);
void angularNormalizer(double *ang);
void update_parameters(void);

#ifdef __cplusplus
}
#endif

#endif // STEWART_PLATFORM_H
