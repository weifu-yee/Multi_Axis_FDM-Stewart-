/*
 * velocity_planner.cpp
 *
 *  Created on: Aug 29, 2024
 *      Author: chenw
 */

#include "velocity_planner.h"
#include "mainpp.h"

SPPose plan_velocity(void) {
	SPPose diff = calculate_difference(current, target);

    SPPose next_goal = create_default_stewart_platform();
    next_goal.velo.x = sub_plan_velocity(diff.disp.x, diff.velo.x);
//    next_goal.velo.y = sub_plan_velocity(current->velo.y, target->velo.y);
//    next_goal.velo.z = sub_plan_velocity(current->velo.z, target->velo.z);
//    next_goal.velo.phi = sub_plan_velocity(current->velo.phi, target->velo.phi);
//    next_goal.velo.theta = sub_plan_velocity(current->velo.theta, target->velo.theta);
//    next_goal.velo.psi = sub_plan_velocity(current->velo.psi, target->velo.psi);
    //按最大比例縮小
    return next_goal;
}

double sub_plan_velocity(const double diff, const double vdiff) {
	double next;
	next = tar;
	next = smooth_start_clamper
}

double smooth_start_clamper(double planned_velocity, double max_acceleration, double current_velocity) {
    double time_step = 1 / FREQUENCY;
	double accel_limit = max_acceleration * time_step;
    double velocity_change = planned_velocity - current_velocity;

    if (velocity_change > accel_limit) {
        return current_velocity + accel_limit;
    } else if (velocity_change < -accel_limit) {
        return current_velocity - accel_limit;
    } else {
        return planned_velocity;
    }
}

double max_speed_clamper(double planned_velocity, double max_speed) {
    if (planned_velocity > max_speed) {
        return max_speed;
    } else if (planned_velocity < -max_speed) {
        return -max_speed;
    } else {
        return planned_velocity;
    }
}

double smooth_stop_clamper(double planned_velocity, double max_acceleration, double target_velocity, double current_velocity, double time_step) {
    double v0;
	double t = v0 / max_acceleration;
    double s = 0.5 * v0 * t;
    if ()
}
