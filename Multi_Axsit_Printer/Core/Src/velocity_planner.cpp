/*
 * velocity_planner.cpp
 *
 *  Created on: Aug 29, 2024
 *      Author: chenw
 */

#include "velocity_planner.h"
#include "mainpp.h"

void plan_velocity(const double current_length[6], const double target_length[6]) {
	double time_step = 1 / frequency;
    for (int i = 0; i < 6; ++i) {
        double length_diff = target_length[i] - current_length[i];
        double desired_speed = length_diff / time_step; // 简单的速度规划，不考虑加速和减速
        // 平滑速度变化
        double current_speed = pusher[i].goalVel;
        double speed_change = desired_speed - current_speed;
        if (fabs(speed_change) > MAX_ACCELERATION * time_step) {
            pusher[i].goalVel = current_speed + (speed_change > 0) ? MAX_ACCELERATION * time_step : -MAX_ACCELERATION * time_step;
        } else {
            pusher[i].goalVel = desired_speed;
        }
        // 确保速度在范围内
        if (pusher[i].goalVel > MAX_SPEED) pusher[i].goalVel = MAX_SPEED;
        if (pusher[i].goalVel < -MAX_SPEED) pusher[i].goalVel = -MAX_SPEED;
    }
}

void sub_plan_velocity() {

}

