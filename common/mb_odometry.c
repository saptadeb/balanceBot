/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

double e_d, e_b;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
/* TODO */
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->psi = theta;
	mb_odometry->left_last_angle = 0.0;
	mb_odometry->right_last_angle = 0.0;
	mb_odometry->last_yaw = 0.0;


    FILE* fp = fopen(ERR_PATH, "r");
    if (fp == NULL){
        printf("Error opening %s\n", ERR_PATH );
    }
    if (fscanf(fp, "%lf,%lf\n", &e_d, &e_b) != 2) {
        fprintf(stderr, "Couldn't read value for correction factors\n");
        return;
    }
    fclose(fp);
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
/* TODO */
	mb_state->wheelAngleR = (rc_encoder_eqep_read(RIGHT_MOTOR) * 2.0 * M_PI) / (ENC_2_POL * GEAR_RATIO * ENCODER_RES);
    mb_state->wheelAngleL = (rc_encoder_eqep_read(LEFT_MOTOR) * 2.0 * M_PI) / (ENC_1_POL * GEAR_RATIO * ENCODER_RES);

    double c_l = 2 / (e_d + 1);
    double c_r = 2 / (1 / e_d + 1);

    double left_travel = (mb_state->wheelAngleL - mb_odometry->left_last_angle) * (WHEEL_DIAMETER * c_l) / 2.0;
    double right_travel = (mb_state->wheelAngleR - mb_odometry->right_last_angle) * (WHEEL_DIAMETER * c_r) / 2.0;

    mb_state->wheelVelR = right_travel / DT;
    mb_state->wheelVelL = left_travel / DT;

    mb_odometry->left_last_angle = mb_state->wheelAngleL;
    mb_odometry->right_last_angle = mb_state->wheelAngleR;

    	double delta_d = (left_travel + right_travel) / 2.0;
	    double delta_psi = -(right_travel - left_travel) / (WHEEL_BASE * e_b);

	    mb_odometry->x -= delta_d * cos(mb_odometry->psi + delta_psi / 2.0);
	    mb_odometry->y -= delta_d * sin(mb_odometry->psi + delta_psi / 2.0);
	    mb_odometry->last_yaw = mb_odometry->psi;
	    mb_odometry->psi += delta_psi;

}


float mb_clamp_radians(float angle){
    return 0;
}
