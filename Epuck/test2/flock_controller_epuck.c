#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../library/ircom/ircom.h"
#include "../library/btcom/btcom.h"
#include "../library/ircom/e_ad_conv.h"
#include "../library/epfl/e_init_port.h"
#include "../library/epfl/e_epuck_ports.h"
#include "../library/epfl/e_uart_char.h"
#include "../library/epfl/e_led.h"

#include "../library/epfl/e_led.h"
#include "../library/epfl/e_motors.h"
#include "../library/epfl/e_agenda.h"

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed

#define FLOCK_SIZE	  3	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


#define RULE1_WEIGHT        (0.8/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.12   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.5/10)	   // Weight of dispersion rule. default 0.02/10

#define MIGRATION_WEIGHT    (0.04/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define BRAITENBERG_WEIGHT 4.0f

#define ABS(x) ((x>=0)?(x):-(x))
#define M_PI 3.141592

#define OBSTACLE_SCENARIO 1

float maxSpeed = MAX_SPEED* WHEEL_RADIUS;

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

int robot_id;

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float my_position[3];     		// X, Z, Theta of the current robot

float new_speed[2] = {0.0f, 0.0f};
float migr[2] = {0.0,-10.0};	        // Migration vector
int start, end;

/*
 * Reset the robot's devices and get its ID
 */
static void reset() 
{
	// Initialize system and sensors
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();
    
    e_calibrate_ir(); 
    
    robot_id = 0;
    
    migr[0] = 0.0f;
	migr[1] = -1.2f * maxSpeed;

	if(OBSTACLE_SCENARIO) {
    	start = 0;
    	end = FLOCK_SIZE;
	}
	else if(robot_id < FLOCK_SIZE / 2) {
		start = 0;
		end = FLOCK_SIZE / 2;
	}
	else {
		start  = FLOCK_SIZE / 2;
		end = FLOCK_SIZE;
	}

}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit) {
		*number = limit;
	}
	if (*number < -limit) {
		*number = -limit;
	}
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	float speed_norm = sqrtf(new_speed[0]*new_speed[0] + new_speed[1]*new_speed[1]);	  // Desired speed, maximum = 0.12 : maxSpeed
	float bearing = atan2(new_speed[0], -new_speed[1]);	  // Orientation of the wanted position
	//printf("id = %d, x = %f, y = %f, theta = %f, speed_x = %f, speed_y = %f\n", robot_id, x, z, my_position[2], new_speed[0], new_speed[1]);

	// Compute forward control
	if(speed_norm > maxSpeed)
		speed_norm = maxSpeed;

	// Compute priority oof rotation to forward control
	float w = 0.0f;
	float u = 0.0f;
	if(bearing > M_PI / 2.0f ) {
		// More than 90deg, turn to max speed
		w = maxSpeed;
		u = 0.0f;
	}
	else if( bearing < -M_PI / 2.0f ) {
		// More than 90deg, turn to max speed
		w = -maxSpeed;
		u = 0.0f;
	}
	else {
		//Linear function between 0 and max_speed while bearing going from 0 to pi/2
		w = bearing * 2.0f / M_PI * maxSpeed;
		u = speed_norm * ( M_PI / 2.0f - ABS(bearing) ) / (M_PI / 2.0f);
	}
	
	// Convert to wheel speeds!
	*msl = (u - w) * (1000.0f / maxSpeed);
	*msr = (u + w) * (1000.0f / maxSpeed);
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i;							// Loop counter
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	
	/* Compute averages over the whole flock */
	for(i = start; i < end; i++) {
		if(i == robot_id) {
			continue;
		}

		rel_avg_loc[0] += relative_pos[i][0];
		rel_avg_loc[1] += relative_pos[i][1];
		//printf("Nb : %d, rel_0 = %f, rel_1 = %f, avg_0 = %f, avg_1 = %f\n", i, relative_pos[i][0], relative_pos[i][1], rel_avg_loc[0], rel_avg_loc[1]);
	}

	rel_avg_loc[0] /= end - start - 1;
	rel_avg_loc[1] /= end - start - 1;
	
	//printf("Robot %s, x_avg: %f, y_avg: %f\n",robot_name,rel_avg_loc[0],rel_avg_loc[1]);
	
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	cohesion[0] = rel_avg_loc[0];
	cohesion[1] = rel_avg_loc[1];

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for(i = start; i < end; i++) {
		if(i != robot_id) {

			float distance = sqrtf( pow(relative_pos[i][0], 2)+pow(relative_pos[i][1], 2) );
			float repulsion1 = RULE2_THRESHOLD/distance - 1;

			if( distance < RULE2_THRESHOLD) {
				dispersion[0] -= relative_pos[i][0] / distance * repulsion1;
				dispersion[1] -= relative_pos[i][1] / distance * repulsion1;
				//printf("Robot : %d - %d Dispersion: dispersion[0] = %f, dispersion[1] = %f, rel_pos[0] = %f, rel_pos[1] = %f\n", robot_id, k, dispersion[0], dispersion[1], relative_pos[k][0], relative_pos[k][1]);
			}
		}
	}

	/* Aggregation of all behaviors with relative influence determined by weights */
	new_speed[0] = cohesion[0] * RULE1_WEIGHT;
	new_speed[1] = cohesion[1] * RULE1_WEIGHT;

	//printf("Robot %d, coh_x = %f, coh_y = %f, new_speed_0 = %f, new_speed_1 = %f\n", robot_id, cohesion[0], cohesion[1], new_speed[0], new_speed[1]);
	new_speed[0] +=  dispersion[0] * RULE2_WEIGHT;
	new_speed[1] +=  dispersion[1] * RULE2_WEIGHT;
	//printf("Robot %d, dis_x = %f, dis_y = %f, new_speed_0 = %f, new_speed_1 = %f\n", robot_id, dispersion[0], dispersion[1], new_speed[0], new_speed[1]);

	new_speed[1] *= -1; //y axis of webots is inverted

	/* Migration - Conversion from absolute direction to relative frame */
	new_speed[0] +=  migr[0] * cos(my_position[2]) + migr[1] * sin(my_position[2]);
	new_speed[1] += -migr[0] * sin(my_position[2]) + migr[1] * cos(my_position[2]); //y axis of webots is inverted
	//printf("Robot %d, migr0 = %f, migr1 = %f, speed0 = %f, speed1 = %f, theta = %f \n", robot_id, migr[0], migr[1], new_speed[0], new_speed[1], my_position[2] );
	}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{
	int i = 0;
	//while(ircomSendDone() == 0) {
	while(i < 200) {
		ircomSend(robot_id);
		if(ircomSendDone() == 0) {
			break;
		}
		i++;
	}
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	int other_robot_id;
	double range;
	double direction;
	double theta;
	IrcomMessage imsg;
	
	int i = 0;
	while(i < 200) {
		ircomPopMessage(&imsg);
		if(imsg.error == 0) {
			other_robot_id = (int)imsg.value;
			char tmp[128];
			sprintf(tmp, "Receive successful : %d  - distance=%f \t direction=%f \n", other_robot_id, (double)imsg.distance, (double)imsg.direction);
			btcomSendString(tmp);
			range = (double) imsg.distance;
			direction = (double) imsg.direction;
		}else if (imsg.error > 0) {
			btcomSendString("Receive failed \n");		
		}
		if(imsg.error != -1) {
			i++;
		}
	}
	double x = sinf(direction); // A verifier
	double y = cosf(direction);
	theta = -atan2(y, x);
	theta = theta + my_position[2];
	
	relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
	relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos
}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	
	reset();			// Resetting the robot
	
	// initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();
    

	msl = 0; msr = 0; 
	max_sens = 0; 
	
	// Forever
	for(;;){

		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;

		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) 
		{
			distances[i]= e_get_calibrated_prox(i); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		update_self_motion(msl,msr);

		process_received_ping_messages();

		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();

		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);

		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}

		// Add Braitenberg
		msl += bmsl*BRAITENBERG_WEIGHT;
		msr += bmsr*BRAITENBERG_WEIGHT;
		
		
		e_set_speed_left(msl);
		e_set_speed_right(msr);
	}
	ircomStop();
}
