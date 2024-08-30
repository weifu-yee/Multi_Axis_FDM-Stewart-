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

#include "stewart_platform.h"

SPPose plan_velocity(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_VELOCITY_PLANNER_H_ */
