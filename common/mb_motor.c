#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;

int mb_motor_init(){
    
    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

int mb_motor_init_freq(int pwm_freq_hz){
    rc_pwm_init(1,pwm_freq_hz);
    rc_pwm_set_duty(1,'A',0);
    rc_pwm_set_duty(1,'B',0);

    rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    init_flag = 1;
    return 0;
}

int mb_motor_cleanup(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }

    return 0;
}

int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

   return 0;
}

int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    
    return 0;
}

int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN,1);
    rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN,0);

    if(duty < 0) {
        rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN,0);
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN,1);
        duty = -duty;
    }
    if(duty>1.0){duty = 1.0;}
    if(motor == 1) {
        rc_pwm_set_duty(1,'A',duty);
    } else {
        rc_pwm_set_duty(1,'B',duty);
    }
    
    return 0;
}

int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    
    return 0;
}

double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    return 0.0;
}
