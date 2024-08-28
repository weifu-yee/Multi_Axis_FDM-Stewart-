/*
 * PIDcontrol.h
 *
 *  Created on: Aug 28, 2024
 *      Author: chenw
 */

#ifndef INC_PIDCONTROL_H_
#define INC_PIDCONTROL_H_

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

void calculatePID(int i);
void update_from_sensor(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PIDCONTROL_H_ */
