#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#include <robotcontrol.h>


int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, rc_mpu_data_t* mpu_data);
int mb_controller_cleanup();

#define DT 0.01
#define SOFT_START_SEC 0.7
#define THETA_REF_MAX 0.5
#define STEERING_INPUT_MAX 0.5


// DSM channel config
#define DSM_DRIVE_POL		-1
#define DSM_TURN_POL		-1
#define DSM_DRIVE_CH		3
#define DSM_TURN_CH			4
#define DSM_MANUAL_CTL_CH   5
#define DSM_DEAD_ZONE		0.04
#define DRIVE_RATE_NOVICE   25
#define TURN_RATE_NOVICE 	4

//For the controllers
#define D1_NUM_LEN 3
#define D1_DEN_LEN 3

#define D2_NUM_LEN 3
#define D2_DEN_LEN 3




#endif

