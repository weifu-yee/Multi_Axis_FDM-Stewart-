/*
 * control.h
 *
 *  Created on: Aug 28, 2024
 *      Author: chenw
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct { // In mm units
    int16_t enc;
    double insVel;
    double u, up, pulse, goal_pulse;
    int dir;
} Actuator;

extern Actuator pusher[7];

void reset_pushers_to_home(void);
void update_pushers_PWM(const double diff_lengths[6]);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONTROL_H_ */
