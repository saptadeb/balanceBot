/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"


void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
/* TODO */
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->psi = theta;
	mb_odometry->left_last_angle = 0.0;
	mb_odometry->right_last_angle = 0.0;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
/* TODO */
	mb_state->wheelAngleR = (rc_encoder_eqep_read(RIGHT_MOTOR) * 2.0 * M_PI) / (ENC_2_POL * GEAR_RATIO * ENCODER_RES);
    mb_state->wheelAngleL = (rc_encoder_eqep_read(LEFT_MOTOR) * 2.0 * M_PI) / (ENC_1_POL * GEAR_RATIO * ENCODER_RES);

    double left_travel = (mb_state->wheelAngleL - mb_odometry->left_last_angle) * WHEEL_DIAMETER / 2.0;
    double right_travel = (mb_state->wheelAngleR - mb_odometry->right_last_angle) * WHEEL_DIAMETER / 2.0;

    mb_state->wheelVelR = right_travel / DT;
    mb_state->wheelVelL = left_travel / DT;

    mb_odometry->left_last_angle = mb_state->wheelAngleL;
    mb_odometry->right_last_angle = mb_state->wheelAngleR;

    double w = (mb_state->wheelVelR - mb_state->wheelVelL) / WHEEL_BASE;

    if (abs(w) < 0.001)
    {
    	double delta_d = (left_travel + right_travel) / 2.0;
	    double delta_psi = (right_travel - left_travel) / WHEEL_BASE;

	    mb_odometry->x += delta_d * cos(mb_odometry->psi + delta_psi / 2.0);
	    mb_odometry->y += delta_d * sin(mb_odometry->psi + delta_psi / 2.0);
	    mb_odometry->psi += delta_psi;
    } else {
	    double R = (WHEEL_BASE / 2.0) *(mb_state->wheelVelR + mb_state->wheelVelL) / (mb_state->wheelVelR - mb_state->wheelVelL);
   	    double C_R[2];
	    C_R[0] = mb_odometry->x - R * sin(mb_odometry->psi);
	    C_R[1] = mb_odometry->y + R * cos(mb_odometry->psi);
    	double delta_psi = w*DT;
	    mb_odometry->x = (mb_odometry->x - C_R[0])*cos(delta_psi) - (mb_odometry->y - C_R[1]) * sin(delta_psi) + C_R[0];
	    mb_odometry->y = (mb_odometry->x - C_R[0])*sin(delta_psi) + (mb_odometry->y - C_R[1]) * cos(delta_psi) + C_R[1];
	    mb_odometry->psi = mb_odometry->psi + delta_psi;
    }
    
}


float mb_clamp_radians(float angle){
    return 0;
}