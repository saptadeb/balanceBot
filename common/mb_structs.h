#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    float   gamma;             // heading angle
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    float wheelAngleR;
    float wheelAngleL;
    float wheelVelR;
    float wheelVelL;
    float d1_u;
    float d2_u;  //duty for outer loop
    float d3_u;  //duty for steering
    float vbatt;
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float theta_ref;
    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
    float phi_dot;
    float phi_ref;
    float gamma_ref;
    float gamma_dot;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
    float left_last_angle;
    float right_last_angle;
    float last_yaw;
};

#endif
