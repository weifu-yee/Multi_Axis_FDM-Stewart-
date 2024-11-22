/*
 * determine_KP_mode.cpp
 *
 *  Created on: Nov 22, 2024
 *      Author: user
 */
#include "determine_KP_mode.h"
#include "TFTransform.h"
#include "constants.h"
#include "timing.h"
#include "stewart_platform.h"

extern int line_of_Gcode;
extern bool reached;
extern double X;
extern double Y;
extern double Z;
extern double A;
extern double B;
extern double C;
extern double F;

double mod_Kp[7] = {0, KP_1, KP_2, KP_3, KP_4, KP_5, KP_6};
double mod_Kp_times_1000[7];
double mod_W2N_Z = 600.0;

void pose_init(void) {
	X = 0.0;
	Y = 0.0;
	Z = 40.0;
	A = 0.0;
	B = 0.0;
	C = 0.0;
}
void rectangular_Gcode(void) {
	pose_init();
	int line = (int)line_of_Gcode % 4;
	switch(line){
	case 0:
		X = 30.0;
		Y = 30.0;
		break;
	case 1:
		X = -30.0;
		Y = 30.0;
		break;
	case 2:
		X = -30.0;
		Y = -30.0;
		break;
	case 3:
		X = 30.0;
		Y = -30.0;
		break;
	default:
		break;
	}
}
void elevator_Gcode(void) {
	pose_init();
	int line = (int)line_of_Gcode % 2;
	switch(line){
	case 0:
		Z -= 30.0;
		break;
	case 1:
		Z += 30.0;
		break;
	default:
		break;
	}
}
void update_KP_s(void) {
	//update 'mod_Kp_times_1000' array directly in the "live representation" window
	if(mod_Kp_times_1000[0]) {
		for(int i = 1; i <= 6; ++i) {
			mod_Kp_times_1000[i] = mod_Kp_times_1000[0];
		}
		mod_Kp_times_1000[0] = 0.0;
	}
	for(int i = 1; i <= 6; ++i) {
		mod_Kp[i] = mod_Kp_times_1000[i] / 1000.0;
	}
}
void update_WORD2NOZZLE_TRANSLATION(void) {
	transformer.setWordToNozzleTransform(
		WORD2NOZZLE_TRANSLATION_X,
		WORD2NOZZLE_TRANSLATION_Y,
		mod_W2N_Z,
		WORD2NOZZLE_ROTATION_X_DEGREE,
		WORD2NOZZLE_ROTATION_Y_DEGREE,
		WORD2NOZZLE_ROTATION_Z_DEGREE);
}

void determine_KP_loop(int kind) {
	for(int i = 1; i <= 6; ++i) { //init the array
		mod_Kp_times_1000[i] = mod_Kp[i] * 1000.0;
	}
	while(1){
		line_of_Gcode++;

		if(kind == 1) rectangular_Gcode();
		else elevator_Gcode();
		transformer.setPartToNozzleTransform(X, Y, Z, A, B, C);
		SPPose pose = transformer.getJointPlanePoseInWorldFrame();
		update_parameters(&pose, F);

		prev_diffNorm = 0;
		increasing_count = 0;
		prev_SPerror = 0;
		SPerror_increasing_count = 0;

		reached = false;
		while(!reached){
			update_KP_s();
			update_WORD2NOZZLE_TRANSLATION();
		}; //waiting the process in timing.cpp

//		if() { //when M2 end of file been delivered.
//
//		}

		//this while is for debug, lock the process between each line of Gcode.
//		while(_c != 0){}
//		_c ++;
		HAL_Delay(1000);  // Delay for 700 ms
	}
}

