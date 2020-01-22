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

#endif

