#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#include <robotcontrol.h>


int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_controller_cleanup();

#define DT 0.01
#define SOFT_START_SEC 0.7
#define THETA_REF_MAX 0.33
#define STEERING_INPUT_MAX 0.5


// DSM channel config
#define DSM_DRIVE_POL		-1
#define DSM_TURN_POL		-1
#define DSM_DRIVE_CH		3
#define DSM_TURN_CH			4
#define DSM_MANUAL_CTL_CH   5
#define DSM_DEAD_ZONE		0.04
#define DRIVE_RATE_NOVICE   8
#define TURN_RATE_NOVICE 	3

#endif

