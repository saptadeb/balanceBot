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

int WRITE_FLAG = 0;
int GOT_TO_FINISH = 0;
int MODE = 0; //0 balancing    1 for drag race    2 for square
FILE* f1;
double yaw_init = 0.0;
int START = 1;
int TO_TURN = 0;

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(int argc, char **argv){
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

	if (argc < 2)
	{
		fprintf(stderr, "usage: sudo %s <MODE>\n", argv[0]);
		return 1;
	}

	MODE = atoi(argv[1]);

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
	mpu_config.orient = ORIENTATION_Z_DOWN;
	//mpu_config.orient = ORIENTATION_X_FORWARD;


	/*printf("opening file for writing data...\n");
	f1 = fopen("data.csv","w+");
    if(f1 == NULL) {
        printf("Could not open file");
        return 1;
    }
    fprintf(f1, "x, y, psi, yaw\n");*/

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

	mb_setpoints.phi_dot = 0.0;
    mb_setpoints.phi_ref = 0.0;
    mb_setpoints.gamma_dot = 0.0;
    mb_setpoints.gamma_ref = 0.0;

    /*printf("Press enter to start recording data: \n");
    WRITE_FLAG = getchar();*/

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
	//pthread_mutex_lock(&setpoint_mutex);
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = -mpu_data.dmp_TaitBryan[TB_PITCH_X];				//Roll corresponds to pitch and vice versa

	// Read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);

	// Update odometry
	mb_odometry_update(&mb_odometry, &mb_state);
    mb_state.phi = ((mb_state.wheelAngleL+mb_state.wheelAngleR)/2) - mb_state.theta;
    //mb_state.phi = ((mb_state.wheelAngleL+mb_state.wheelAngleR)/2);

    // Calculate controller outputs
    mb_controller_update(&mb_state, &mb_setpoints, &mpu_data);

    float dutyL = mb_state.left_cmd;
    float dutyR = mb_state.right_cmd;

    /*if (abs(dutyL) < 0.002 && abs(dutyR) < 0.002)
    {
    	dutyL = 0.0;
    	dutyR = 0.0;
    }*/

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


	/*if (WRITE_FLAG == '\n')
	{
		fprintf(f1, "%5.4f, %5.4f, %5.4f, %5.4f\n", mb_odometry.x, mb_odometry.y, mb_odometry.psi, mpu_data.dmp_TaitBryan[TB_YAW_Z]);
	}*/
   	//unlock state mutex
	//pthread_mutex_unlock(&setpoint_mutex);

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
		if (MODE == 0)
		{
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
	    																// DRIVING VIA RC

	    			turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
			        drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;
			        // saturate the inputs to avoid possible erratic behavior
			        rc_saturate_double(&drive_stick,-1,1);
			        rc_saturate_double(&turn_stick,-1,1);
			        // use a small deadzone to prevent slow drifts in position
			        if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
			        if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;
			        // GET DRIVE RATES FROM CSV
		        	mb_setpoints.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
	                mb_setpoints.gamma_dot =  TURN_RATE_NOVICE * turn_stick;
	    		} else {
	    																// AUTONOMOUS BALANCING

	    			mb_setpoints.phi_dot = 0.0;
	    			mb_setpoints.gamma_dot = 0.0;
	    		}
			} else {
																		//BALANCING WHEN NO DATA FROM RC

				mb_setpoints.phi_dot = 0.0;
	    		mb_setpoints.gamma_dot = 0.0;
			}
		}

		else if(MODE == 1){
																		//DRAG RACING


			double length;
			double phi_dot_max;
			double initial_dist;   //THE DIST WHEN IT STARTS THE SECOND PARABOLA
			double init_phi_dot;


			FILE* file = fopen("../common/dragrace.csv", "r");
		    if (file == NULL){
		        printf("Error opening %s\n", "dragrace.csv" );
		    }
		    /* TODO parse your config file here*/
		    if (fscanf(file, "%lf,%lf,%lf,%lf\n", &length, &phi_dot_max, &initial_dist, &init_phi_dot) != 4) {
		        fprintf(stderr, "Couldn't read value for drag race.\n");
		        return -1;
		    }

		    fclose(file);
			if (!GOT_TO_FINISH)
			{
				if (mb_odometry.x > length)
				{
					GOT_TO_FINISH = 1;
					mb_setpoints.phi_dot = 0.0;
				} else if (mb_odometry.x < initial_dist)// || mb_odometry.x > length - initial_dist)
				{
					if (mb_odometry.x < initial_dist)
					{
						mb_setpoints.phi_dot = -(init_phi_dot/((initial_dist)*(initial_dist))) * fabs(mb_odometry.x)*(initial_dist * 2 - mb_odometry.x);
					} /*else {
						//mb_setpoints.phi_dot = -(init_phi_dot/(initial_dist)*(length - initial_dist)) * fabs(length - mb_odometry.x) * fabs(mb_odometry.x);
                        mb_setpoints.phi_dot = -1.25*init_phi_dot;
					}*/
				}
				else {
					//mb_setpoints.phi_dot = -(((phi_dot_max-init_phi_dot)/(((length+initial_dist)/2.0-initial_dist)*((length+initial_dist) / 2.0 - initial_dist))
                    mb_setpoints.phi_dot = -(phi_dot_max - init_phi_dot)/(((length+initial_dist)/2.0-initial_dist)*((length + initial_dist)/2.0-initial_dist))*(mb_odometry.x - initial_dist)
						*fabs(length - mb_odometry.x) - init_phi_dot;
				}
			} else {
				mb_setpoints.phi_dot = 0.0;
			}

			mb_setpoints.gamma_dot = 0.0;
		}
		else if(MODE == 2) {
																			//TO DO AUTONOMOUS SQUARE

			double length;
			double radius;
			double ref_gammadot;
			double ref_phidot;
			double temp_x;
			double temp_y;
			double distance;
			double phi_dot_turning;

			FILE* file = fopen("../common/square.csv", "r");
		    if (file == NULL){
		        printf("Error opening %s\n", "square.csv" );
		    }
		    /* TODO parse your config file here*/
		    if (fscanf(file, "%lf,%lf,%lf,%lf,%lf\n", &length, &radius, &ref_gammadot, &ref_phidot, &phi_dot_turning) != 5) {
		        fprintf(stderr, "Couldn't read value for sqaure.\n");
		        return -1;
		    }

		    fclose(file);

			double temp_yaw = fabs(mpu_data.dmp_TaitBryan[TB_YAW_Z]);

			/*if (temp_yaw < -0.001)
			{
				temp_yaw += 2*3.14;
			}*/


			mb_setpoints.phi_dot = -ref_phidot;
			if (START)
			{
				if (mb_odometry.x < (length))
				{
					mb_setpoints.phi_dot = -(4*ref_phidot/((length)*(length))) * fabs(mb_odometry.x)*(length - mb_odometry.x)-phi_dot_turning;
					mb_setpoints.gamma_dot = 0.0;
				} else {
					START = 0;
					TO_TURN += 1;
					//printf("TO_TURN %d\n", TO_TURN);
				}
			} /*else {
				switch(TO_TURN % 2){
					case 1: mb_setpoints.gamma_dot = -ref_gammadot;
							if ((int)100*fabs((temp_yaw) - (yaw_init)) > 150)
							//if (fabs((temp_yaw) - (yaw_init)) > (3.14/2))
							{
								printf("\nStarted Moving straight x: %2.4f, y: %2.4f, temp_yaw: %2.4f, yaw_init: %2.4f TO_TURN: %d, temp_x: %2.4f, temp_y: %2.4f, dist: %2.4f\n",
									mb_odometry.x, mb_odometry.y, temp_yaw, yaw_init, TO_TURN, temp_x, temp_y,distance);
								TO_TURN += 1;
								yaw_init = temp_yaw;
								temp_x = mb_odometry.x;
								temp_y = mb_odometry.y;
							}
							break;
					case 0: mb_setpoints.gamma_dot = 0.0;
							distance = sqrt(pow((mb_odometry.x-temp_x),2)+pow((mb_odometry.y-temp_y),2));


							if (distance > length - (2*radius))
							{
								printf("\nTurning started x: %2.4f, y: %2.4f, temp_yaw: %2.4f, yaw_init: %2.4f TO_TURN: %d, temp_x: %2.4f, temp_y: %2.4f, dist: %2.4f\n",
									mb_odometry.x, mb_odometry.y, temp_yaw, yaw_init, TO_TURN, temp_x, temp_y,distance);
								TO_TURN += 1;
							}
							break;
					}
				}*/
			else {
				switch(TO_TURN % 2){
					case 1: //mb_setpoints.gamma_dot = -ref_gammadot;
							mb_setpoints.gamma_dot = -(ref_gammadot * (temp_yaw)*fabs(yaw_init - temp_yaw))-0.3;
							mb_setpoints.phi_dot = -phi_dot_turning;
							if ((int)100*fabs((temp_yaw) - (yaw_init)) > 150)
							//if (fabs((temp_yaw) - (yaw_init)) > (3.14/2))
							{
								printf("\nStarted Moving straight x: %2.4f, y: %2.4f, temp_yaw: %2.4f, yaw_init: %2.4f TO_TURN: %d, temp_x: %2.4f, temp_y: %2.4f, dist: %2.4f\n",
									mb_odometry.x, mb_odometry.y, temp_yaw, yaw_init, TO_TURN, temp_x, temp_y,distance);
								TO_TURN += 1;
								yaw_init = temp_yaw;
								temp_x = mb_odometry.x;
								temp_y = mb_odometry.y;
								rc_nanosleep(1E9 / RC_CTL_HZ);
							}
							break;
					case 0: mb_setpoints.gamma_dot = 0.0;

							distance = sqrt(pow((mb_odometry.x-temp_x),2)+pow((mb_odometry.y-temp_y),2));
							mb_setpoints.phi_dot = -(4*ref_phidot/((length)*(length))) * fabs(distance)*(length - distance)-phi_dot_turning;

							if (distance > length - (2*radius))
							{
								printf("\nTurning started x: %2.4f, y: %2.4f, temp_yaw: %2.4f, yaw_init: %2.4f TO_TURN: %d, temp_x: %2.4f, temp_y: %2.4f, dist: %2.4f\n",
									mb_odometry.x, mb_odometry.y, temp_yaw, yaw_init, TO_TURN, temp_x, temp_y,distance);
								TO_TURN += 1;
								rc_nanosleep(1E9 / RC_CTL_HZ);
							}
							break;
					}
				}

		}
		//pthread_mutex_unlock(&setpoint_mutex);
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
			printf(" phi_ref |");
			printf("    θ    |");
			printf("    φ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf(" left_d  |");
			printf(" right_d |");
			printf("  gamma  |");
			printf("yaw_init |");
			printf("    x    |");
			printf("    y    |");
			printf(" TO_TURN |");
			printf("   psi   |");
			printf("  phidot |");
			printf("gammadot |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;

		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with \n
			printf("%7.3f  |", mb_setpoints.theta_ref);
			printf("%7.3f  |", mb_setpoints.phi_ref);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			printf("%7.3f  |", mb_state.left_cmd);
			printf("%7.3f  |", mb_state.right_cmd);
			printf("%7.3f  |", mpu_data.dmp_TaitBryan[TB_YAW_Z]);
			printf("%7.3f  |", yaw_init);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%d     |", TO_TURN);
			printf("%7.3f  |", mb_odometry.psi);
			printf("%7.3f  |", mb_setpoints.phi_dot);
			printf("%7.3f  |", mb_setpoints.gamma_dot);

			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
}
