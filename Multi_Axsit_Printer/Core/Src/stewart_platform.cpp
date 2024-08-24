#include "stewart_platform.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

Vector3D p[6], b[6];
int steps = 3;
bool targetReached = false;
const bool DEBUG_PRINT = true;
int identicalStatePrintCount = 0;
const int MAX_IDENTICAL_STATE_PRINTS = 1;

// ANSI escape codes for color
const char* RESET_COLOR = "\033[0m";
const char* GREEN_COLOR = "\033[32m";
const char* CYAN_COLOR = "\033[36m";
const char* RED_COLOR = "\033[31m";
const char* PURPLE_COLOR = "\033[35m";
const char* ORANGE_COLOR = "\033[33m";

// 輔助函數：將弧度轉換為0.01度的整數
int32_t rad_to_centidegree(double rad) {
    return (int32_t)(rad * 180.0 / PI * 100.0);
}

// 輔助函數：將度轉換為弧度
double deg_to_rad(int32_t deg) {
    return deg * PI / 18000.0;  // deg is in 0.01 degree
}

// 輔助函數：印出平台狀態
void printPlatformState(const StewartPlatform* platform) {
    if (!DEBUG_PRINT) return;
    printf("x: %ld, y: %ld, z: %ld, phi: %ld, theta: %ld, psi: %ld\n",
           platform->x, platform->y, platform->z,
           platform->phi, platform->theta, platform->psi);
}

// 輔助函數：印出六根桿的長度
void printLegLengths(const int32_t* current_lengths) {
    if (!DEBUG_PRINT) return;
    printf("Leg lengths: ");
    for (int i = 0; i < 6; i++) {
        printf("%ld ", current_lengths[i]);
    }
    printf("\n");
}

// 解析 G-code 檔案，並返回 StewartPlatform 所需的參數
void readParametersFromGCode(const char* filename, int* x, int* y, int* z, int* phi, int* theta, int* psi, int* h0, int* p_l, int* b_l) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error in open file: %s\n", filename);
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        char param[32];
        int value;
        if (sscanf(line, "%s = %d", param, &value) == 2) {
            if (strcmp(param, "x") == 0) *x = value;
            else if (strcmp(param, "y") == 0) *y = value;
            else if (strcmp(param, "z") == 0) *z = value;
            else if (strcmp(param, "phi") == 0) *phi = value;
            else if (strcmp(param, "theta") == 0) *theta = value;
            else if (strcmp(param, "psi") == 0) *psi = value;
            else if (strcmp(param, "h0") == 0) *h0 = value;
            else if (strcmp(param, "p_l") == 0) *p_l = value;
            else if (strcmp(param, "b_l") == 0) *b_l = value;
        }
    }
    fclose(file);
}

// 解析 G-code 檔案並更新 StewartPlatform 的參數
void updatePlatformFromGCode(StewartPlatform* platform, const char* filename) {
    int x, y, z, phi, theta, psi, h0, p_l, b_l;

    readParametersFromGCode(filename, &x, &y, &z, &phi, &theta, &psi, &h0, &p_l, &b_l);
    
    // 根據用戶約束設定 z = h0
    z = h0;

    // 更新平台參數
    update_parameters(platform, x, y, z, phi, theta, psi, h0, p_l, b_l);
}

StewartPlatform create_stewart_platform(int32_t x, int32_t y, int32_t z,
                                        int32_t phi, int32_t theta, int32_t psi,
                                        int32_t h0, int32_t p_l, int32_t b_l) {
    StewartPlatform platform = {x, y, z, phi, theta, psi, h0, p_l, b_l};
    return platform;
}

StewartPlatform create_default_stewart_platform() {
    return create_stewart_platform(0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void update_parameters(StewartPlatform* platform, int32_t x, int32_t y, int32_t z,
                       int32_t phi, int32_t theta, int32_t psi,
                       int32_t h0, int32_t p_l, int32_t b_l) {
    platform->x = x;
    platform->y = y;
    platform->z = z;
    platform->phi = phi;
    platform->theta = theta;
    platform->psi = psi;
    platform->h0 = h0;
    platform->p_l = p_l;
    platform->b_l = b_l;
}

void move_to_target_state(StewartPlatform* current, const StewartPlatform* target, int steps) {
    // 檢查當前平台狀態是否與目標狀態相同
    if (current->x == target->x &&
        current->y == target->y &&
        current->z == target->z &&
        current->phi == target->phi &&
        current->theta == target->theta &&
        current->psi == target->psi) {
        
        // 只在計數器小於限制時打印訊息
        if (DEBUG_PRINT && identicalStatePrintCount < MAX_IDENTICAL_STATE_PRINTS) {
            printf("%sCurrent state is identical to target state.%s\n", ORANGE_COLOR, RESET_COLOR);
            identicalStatePrintCount++;
        }

        // 已經到達目標狀態，不需要移動，直接設定 targetReached 為 true
        targetReached = true;
        return;
    } else {
        identicalStatePrintCount = 0;
    }

    // 計算每個參數的增量
    double dx = (double)(target->x - current->x) / steps;
    double dy = (double)(target->y - current->y) / steps;
    double dz = (double)(target->z - current->z) / steps;
    double dphi = (double)(target->phi - current->phi) / steps;
    double dtheta = (double)(target->theta - current->theta) / steps;
    double dpsi = (double)(target->psi - current->psi) / steps;

    // 打印初始狀態和目標狀態
    if (DEBUG_PRINT) {
        printf("%sInitial platform state:%s\n", GREEN_COLOR, RESET_COLOR);
        printPlatformState(current);
        printf("\n");
        printf("%sTarget platform state (for %d steps) :%s\n", CYAN_COLOR, steps, RESET_COLOR);
        printPlatformState(target);
        printf("\n");
    }
    
    // 循環逐步移動平台
    for (int i = 0; i < steps; ++i) {
        current->x += dx;
        current->y += dy;
        current->z += dz;
        current->phi += dphi;
        current->theta += dtheta;
        current->psi += dpsi;

        // 計算桿長和角度
        calculate_leg(current, p, b, current->current_lengths, current->current_angles);

        // 打印當前狀態
        if (DEBUG_PRINT) {
            printf("%sStep %d: %s\n", RED_COLOR, i + 1, RESET_COLOR);
            printPlatformState(current);
            printLegLengths(current->current_lengths);
        }

        // 可以在這裡呼叫函數將當前狀態發送到執行器
        // send_lengths_to_actuators(current_lengths);
        // display_angles(current_angles);
    }

    // 最後更新到目標狀態
    *current = *target;

    // 再次計算桿長和角度，確保最終狀態正確
    calculate_leg(current, p, b, current->current_lengths, current->current_angles);

    // 移動完成後，設置 targetReached 為 true
    targetReached = true;

    if (DEBUG_PRINT) {
        printf("%sTarget reached!!!~~%s\n\n", PURPLE_COLOR, RESET_COLOR);
    }
}

void initialize_platform(const StewartPlatform* platform,
                         Vector3D p[6],
                         Vector3D b[6]) {
    const int32_t p_angles[] = {350, 10, 110, 130, 230, 250};
    const int32_t b_angles[] = {310, 50, 70, 170, 190, 290};

    for (int i = 0; i < 6; ++i) {
        double p_rad = deg_to_rad(p_angles[i] * 100);
        double b_rad = deg_to_rad(b_angles[i] * 100);

        p[i].x = (int32_t)(platform->p_l * cos(p_rad));
        p[i].y = (int32_t)(platform->p_l * sin(p_rad));
        p[i].z = 0;

        b[i].x = (int32_t)(platform->b_l * cos(b_rad));
        b[i].y = (int32_t)(platform->b_l * sin(b_rad));
        b[i].z = 0;
    }
}

void print_matrix(const double matrix[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            printf("%e ", matrix[i][j]);
        }
        printf("\n");
    }
}

void rotation_matrix(const StewartPlatform* platform, double pRb[3][3]) {
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

    // print_matrix(pRb);
}

int32_t calculate_length(const Vector3D* T, const double pRb[3][3],
                         const Vector3D* p, const Vector3D* b) {
    double dx = T->x + pRb[0][0] * p->x + pRb[0][1] * p->y + pRb[0][2] * p->z - b->x;
    double dy = T->y + pRb[1][0] * p->x + pRb[1][1] * p->y + pRb[1][2] * p->z - b->y;
    double dz = T->z + pRb[2][0] * p->x + pRb[2][1] * p->y + pRb[2][2] * p->z - b->z;

    return (int32_t)sqrt(dx*dx + dy*dy + dz*dz);
}

struct Angle calculate_angles(const Vector3D* T, const double pRb[3][3],
                              const Vector3D* p_i, const Vector3D* b_i) {
    Vector3D transformed_p = {
        T->x + (int32_t)(pRb[0][0] * p_i->x + pRb[0][1] * p_i->y + pRb[0][2] * p_i->z),
        T->y + (int32_t)(pRb[1][0] * p_i->x + pRb[1][1] * p_i->y + pRb[1][2] * p_i->z),
        T->z + (int32_t)(pRb[2][0] * p_i->x + pRb[2][1] * p_i->y + pRb[2][2] * p_i->z)
    };

    Vector3D leg_vector = {
        transformed_p.x - b_i->x,
        transformed_p.y - b_i->y,
        transformed_p.z - b_i->z
    };

    double leg_length = sqrt(leg_vector.x*leg_vector.x + leg_vector.y*leg_vector.y + leg_vector.z*leg_vector.z);
    double platform_angle = acos(leg_vector.z / leg_length);
    double base_angle = atan2(sqrt(leg_vector.x*leg_vector.x + leg_vector.y*leg_vector.y), leg_vector.z);

    struct Angle result = {
        rad_to_centidegree(platform_angle),
        rad_to_centidegree(base_angle)
    };
    return result;
}

void calculate_leg(const StewartPlatform* platform,
                   const Vector3D p[6],
                   const Vector3D b[6],
                   int32_t lengths[6],
                   struct Angle angles[6]) {
    double pRb[3][3];
    rotation_matrix(platform, pRb);
    Vector3D T = {platform->x, platform->y, platform->z};

    for (int i = 0; i < 6; ++i) {
        lengths[i] = calculate_length(&T, pRb, &p[i], &b[i]);
        angles[i] = calculate_angles(&T, pRb, &p[i], &b[i]);
    }
}
