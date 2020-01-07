/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR                  0 // id of left motor
#define RIGHT_MOTOR                 0 // id of right motor
#define MDIR1_CHIP                  0 // chip of MDIR1 gpio pin
#define MDIR1_PIN                   0 //MDIRR1 gpio(CHIP.PIN) P9.12
#define MDIR2_CHIP                  0 // chip of MDIR2 gpio pin
#define MDIR2_PIN                   0 //  MDIRR2 gpio(CHIP.PIN) P9.15
#define MOT_BRAKE_EN            0,20  // gpio0.20  P9.41
#define MOT_1_POL                   0 // polarity of motor 1
#define MOT_2_POL                   0 // polarity of motor 2
#define ENC_1_POL                   0 // polarity of encoder 1
#define ENC_2_POL                   0 // polarity of encoder 2
#define MOT_1_CS                    0 // analog in of motor 1 current sense
#define MOT_2_CS                    0 // analog in of motor 2 current sense
#define GEAR_RATIO                  0 // gear ratio of motor
#define ENCODER_RES                 0 // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER              0 // diameter of wheel in meters
#define WHEEL_BASE                  0 // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY       0.1 // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY      0.1 // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ            100 // main filter and control loop speed
#define DT                       0.01 // 1/sample_rate
#define PRINTF_HZ                  10 // rate of print loop
#define RC_CTL_HZ                  25 // rate of RC data update

#endif
