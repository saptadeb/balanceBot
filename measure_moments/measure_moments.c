#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>

static rc_mpu_data_t data;
FILE* f1;
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21


void writeData(void) {
    fprintf(f1, "%5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %6.1f, %6.1f, %6.1f\n", data.accel[0], data.accel[1], data.accel[2], 
                                                data.gyro[0], data.gyro[1], data.gyro[2],\
                                                data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                                data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                                data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
    printf("%5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %6.1f, %6.1f, %6.1f\n", data.accel[0], data.accel[1], data.accel[2], 
                                                data.gyro[0], data.gyro[1], data.gyro[2],\
                                                data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                                data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                                data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
}

int main(){
	f1 = fopen("data.csv","w+");
    if(f1 == NULL) {
        printf("Could not open file");
        return 1;
    }
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.orient = ORIENTATION_Z_DOWN;
    //conf.orient = ORIENTATION_X_FORWARD; 
    conf.i2c_bus = I2C_BUS;
    conf.dmp_sample_rate = 100;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.dmp_fetch_accel_gyro=1;

    rc_mpu_initialize_dmp(&data, conf);
    rc_mpu_set_dmp_callback(&writeData);

    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
    	rc_nanosleep(1E9);
    }

    fclose(f1);
	// exit cleanly
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}
