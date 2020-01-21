#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    rc_filter_enable_saturation(&D1, -1, 1);
    rc_filter_enable_soft_start(&D1, SOFT_START_SEC);
    
    if(rc_filter_pid(&D1, kp_1, ki_1, kd_1, 4 * DT, DT)){
        fprintf(stderr,"ERROR in rc_balance, failed to make inner-loop controller\n");
        return -1;
    }
    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/
    if (fscanf(file, "%lf,%lf,%lf", &kp_1, &ki_1, &kd_1) != 1) {
        fprintf(stderr, "Couldn't read value.\n");
        return NULL;
    }
    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
    //D1.gain = D1_GAIN * V_NOMINAL/mb_state->vBatt;
    mb_state->d1_u = rc_filter_march(&D1,(mb_setpoints->theta_ref-mb_state->theta));

    mb_state->left_cmd = mb_state->d1_u;   // include d3_u for steering
    mb_state->right_cmd = mb_state->d1_u;
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    return 0;
}