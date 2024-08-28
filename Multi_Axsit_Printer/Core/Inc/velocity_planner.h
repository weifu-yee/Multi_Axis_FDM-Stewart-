/*
 * velocity_planner.h
 *
 *  Created on: Aug 29, 2024
 *      Author: chenw
 */

#ifndef INC_VELOCITY_PLANNER_H_
#define INC_VELOCITY_PLANNER_H_

#ifdef __cplusplus
extern "C" {
#endif

void plan_velocity(const double current_length[6], const double target_length[6]);

#ifdef __cplusplus
}
#endif

#endif /* INC_VELOCITY_PLANNER_H_ */
