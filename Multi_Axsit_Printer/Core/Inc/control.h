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
    double Kp, Ki, Kd;
    int16_t enc;
    double goalVel, insVel, error, last_error;
    double u, up, ui, ud, pulse;
} ActuatorPID;

void reset_pushers_to_home(void);
void update_pushers_PWM(const double diff_lengths[6]);
void calculatePID(int i);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONTROL_H_ */
