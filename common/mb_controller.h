#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.csv"

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state);
int mb_controller_cleanup();

#define DT 0.01
#define SOFT_START_SEC 0.7

//global variables
rc_filter_t D1 = RC_FILTER_INITIALIZER;
double kp_1, ki_1, kd_1, Tf_1;

#endif

