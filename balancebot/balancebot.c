/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>


#include "balancebot.h"

double kp_1, ki_1, kd_1;
double kp_2, ki_2, kd_2;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_UP;
	mpu_config.orient = ORIENTATION_X_FORWARD;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);


	printf("initializing starting angles...\n");
	mb_setpoints.phi_dot = 0.0;     //for balancing; need to change later
    mb_setpoints.phi_ref = 0.0;         //for balancing; need to change later
    
    mb_setpoints.gamma_dot = 0.0;     //for balancing; need to change later
    mb_setpoints.gamma_ref = 0.0;      //for balancing; need to change later


	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}

/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU

	mb_state.theta = mpu_data.dmp_TaitBryan[TB_ROLL_Y];				//Roll corresponds to pitch and vice versa

	// Read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);
    // Update odometry 
 
	mb_state.wheelAngleR = (rc_encoder_eqep_read(RIGHT_MOTOR) * 2.0 * M_PI) / (ENC_2_POL * GEAR_RATIO * ENCODER_RES);
    mb_state.wheelAngleL = (rc_encoder_eqep_read(LEFT_MOTOR) * 2.0 * M_PI) / (ENC_1_POL * GEAR_RATIO * ENCODER_RES);

    mb_state.phi = ((mb_state.wheelAngleL+mb_state.wheelAngleR)/2) + mb_state.theta;

    // Calculate controller outputs
    //mb_setpoints.theta_ref = 0.0;

    mb_controller_update(&mb_state, &mb_setpoints);

    float dutyL = mb_state.left_cmd;
    float dutyR = mb_state.right_cmd;

    mb_motor_set(LEFT_MOTOR, MOT_1_POL * dutyL);
    mb_motor_set(RIGHT_MOTOR, MOT_2_POL * dutyR);

    /*if(!mb_setpoints.manual_ctl){	
        mb_motor_set(LEFT_MOTOR, MOT_1_POL * dutyL);
        mb_motor_set(RIGHT_MOTOR, MOT_2_POL * dutyR);
   	}

    if(mb_setpoints.manual_ctl){
    	//send motor commands
   	}*/

	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	
	if (rc_get_state() == EXITING)
	{
		mb_motor_set(LEFT_MOTOR,0.0);
		mb_motor_set(RIGHT_MOTOR,0.0);
	}
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);
}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	double drive_stick, turn_stick;
	while(1){

		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.
			//pthread_mutex_lock(&setpoint_mutex);
			
			if(rc_dsm_ch_normalized(DSM_MANUAL_CTL_CH) == 1){
    			mb_setpoints.manual_ctl = 1;
    		} else{
    			mb_setpoints.manual_ctl = 0;
    		}
    		if (mb_setpoints.manual_ctl == 1)
    		{
    			turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
		        drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;
		        // saturate the inputs to avoid possible erratic behavior
		        rc_saturate_double(&drive_stick,-1,1);
		        rc_saturate_double(&turn_stick,-1,1);
		        // use a small deadzone to prevent slow drifts in position
		        if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
		        if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;
		        // translate normalized user input to real setpoint values
		        /*switch(mb_setpoints.drive_mode){
		        case NOVICE:
		                setpoint.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
		                setpoint.gamma_dot =  TURN_RATE_NOVICE * turn_stick;
		                break;
		        case ADVANCED:
		                setpoint.phi_dot   = DRIVE_RATE_ADVANCED * drive_stick;
		                setpoint.gamma_dot = TURN_RATE_ADVANCED  * turn_stick;
		                break;
		        default: break;
	        	}*/
	        	mb_setpoints.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
                mb_setpoints.gamma_dot =  TURN_RATE_NOVICE * turn_stick;
    		} else {
    			mb_setpoints.phi_dot = 0.0;
    			mb_setpoints.gamma_dot = 0.0;
    		}
	        
    		//pthread_mutex_unlock(&setpoint_mutex);
		}
		else {
			mb_setpoints.phi_dot = 0.0;
			mb_setpoints.gamma_dot = 0.0;
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("theta_ref|");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("left_duty|");
			printf("right_duty|");
			printf("  gamma  |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_setpoints.theta_ref);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			printf("%7.3f  |", mb_state.left_cmd);
			printf("%7.3f  |", mb_state.right_cmd);
			printf("%7.3f  |", mb_state.gamma);

			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 


/*static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
        double new_v;
        while(rc_get_state()!=EXITING){
                new_v = rc_adc_batt();
                // if the value doesn't make sense, use nominal voltage
                if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL; // need to change the limits in this if statement
                cstate.vBatt = new_v;
                rc_usleep(1000000 / BATTERY_CHECK_HZ);
        }
        return NULL;
}*/