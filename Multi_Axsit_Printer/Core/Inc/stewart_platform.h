#ifndef STEWART_PLATFORM_H
#define STEWART_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} Vector3D;

extern Vector3D p[6];
extern Vector3D b[6];
extern int steps;
extern bool targetReached;

struct Angle {
    int32_t platform_angle; // 角度 * 100
    int32_t base_angle; // 角度 * 100
};

typedef struct {
    int32_t x, y, z, phi, theta, psi, h0, p_l, b_l;
    int32_t current_lengths[6];
    struct Angle current_angles[6];
} StewartPlatform;

StewartPlatform create_stewart_platform(int32_t x, int32_t y, int32_t z,
                                        int32_t phi, int32_t theta, int32_t psi,
                                        int32_t h0, int32_t p_l, int32_t b_l);

StewartPlatform create_default_stewart_platform();

void update_parameters(StewartPlatform* platform, int32_t x, int32_t y, int32_t z,
                       int32_t phi, int32_t theta, int32_t psi,
                       int32_t h0, int32_t p_l, int32_t b_l);
StewartPlatform calculate_difference(const StewartPlatform *current, const StewartPlatform *target);

void move_platform_to_target_pose(StewartPlatform* current, const StewartPlatform* target);
void move_to_target_state(StewartPlatform* current, const StewartPlatform* target, int steps);

void read_parameters_from_gcode(const char* filename, int* x, int* y, int* z,
                                int* phi, int* theta, int* psi,
                                int* h0, int* p_l, int* b_l);

void update_platform_from_gcode(StewartPlatform* platform, const char* filename);

void initialize_platform(const StewartPlatform* platform,
                         Vector3D p[6],
                         Vector3D b[6]);

void calculate_leg(const StewartPlatform* platform,
                   const Vector3D p[6],
                   const Vector3D b[6],
                   int32_t lengths[6],
                   struct Angle angles[6]);

#ifdef __cplusplus
}
#endif

#endif // STEWART_PLATFORM_H
