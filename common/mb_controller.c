#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"


//global variables
rc_filter_t D1 = RC_FILTER_INITIALIZER;
rc_filter_t D2 = RC_FILTER_INITIALIZER;
rc_filter_t D3 = RC_FILTER_INITIALIZER;

double kp_1, ki_1, kd_1, gain_1;
double kp_2, ki_2, kd_2, gain_2;
double kp_3, ki_3, kd_3, gain_3;

double num_11,num_12,num_13;
double den_11,den_12,den_13;

double num_21,num_22,num_23;
double den_21,den_22,den_23;


int mb_controller_init(){
    mb_controller_load_config();

    // Inner loop controller
    kp_1 = kp_1*gain_1;
    kd_1 = kd_1*gain_1;
    ki_1 = ki_1*gain_1;

    kp_2 = kp_2*gain_2;
    kd_2 = kd_2*gain_2;
    ki_2 = ki_2*gain_2;

    kp_3 = kp_3*gain_3;
    kd_3 = kd_3*gain_3;
    ki_3 = ki_3*gain_3;

    if(rc_filter_pid(&D1, kp_1, ki_1, kd_1, 0.0278, DT)){       //Third argument is Tf
        fprintf(stderr,"ERROR in rc_balance, failed to make inner-loop controller\n");
        return -1;
    }

    rc_filter_enable_saturation(&D1, -1.0, 1.0);
    rc_filter_enable_soft_start(&D1, SOFT_START_SEC);
    printf("inner loop controller D1: \n");
    rc_filter_print(D1);

    // Outer loop controller
    if(rc_filter_pid(&D2, kp_2, ki_2, kd_2, 0.1944, DT)){
        fprintf(stderr,"ERROR in rc_balance, failed to make outer-loop controller\n");
        return -1;
    }

    rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
    rc_filter_enable_soft_start(&D2, SOFT_START_SEC);
    printf("outer loop controller D2: \n");
    rc_filter_print(D2);

    // set up D3 gamma (steering) controller
    if(rc_filter_pid(&D3, kp_3, ki_3, kd_3, 4*DT, DT)){
            fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
            return -1;
    }
    rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);
    rc_filter_enable_soft_start(&D3, SOFT_START_SEC);
    return 0;
}

int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    if (fscanf(file, "%lf,%lf,%lf,%lf\n", &kp_1, &ki_1, &kd_1, &gain_1) != 4) {
        fprintf(stderr, "Couldn't read value for inner loop.\n");
        return -1;
    }
    if (fscanf(file, "%lf,%lf,%lf,%lf\n", &kp_2, &ki_2, &kd_2, &gain_2) != 4) {
        fprintf(stderr, "Couldn't read value for outer loop.\n");
        return -1;
    }
    if (fscanf(file, "%lf,%lf,%lf,%lf\n", &kp_3, &ki_3, &kd_3, &gain_3) != 4) {
        fprintf(stderr, "Couldn't read value for outer loop.\n");
        return -1;
    }
    fclose(file);

    return 0;
}


int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, rc_mpu_data_t* mpu_data){

    //Outer loop
    int ENABLE_POSITION_HOLD = 1;    //for balancing

    if(ENABLE_POSITION_HOLD){
        if(fabs(mb_setpoints->phi_dot) > 0.001) mb_setpoints->phi_ref += mb_setpoints->phi_dot*DT;
        mb_state->d2_u = rc_filter_march(&D2,mb_setpoints->phi_ref-mb_state->phi);
        mb_setpoints->theta_ref = mb_state->d2_u - 0.0;
    }
    else mb_setpoints->theta_ref = 0.0;

    //inner loop marching
    mb_state->d1_u = rc_filter_march(&D1,(mb_setpoints->theta_ref-mb_state->theta));

    mb_state->gamma = (mb_state->wheelAngleR-mb_state->wheelAngleL) * (WHEEL_DIAMETER/ (2 * WHEEL_BASE));

    if(fabs(mb_setpoints->gamma_dot)>0.0001) mb_setpoints->gamma_ref += mb_setpoints->gamma_dot * DT;
    mb_state->d3_u = rc_filter_march(&D3,mb_setpoints->gamma_ref - mb_state->gamma);

    mb_state->left_cmd = mb_state->d1_u - mb_state->d3_u;   // include d3_u for steering
    mb_state->right_cmd = mb_state->d1_u + mb_state->d3_u;

    return 0;
}

int mb_controller_cleanup(){
    return 0;
}
