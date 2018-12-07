/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions		     */
/*                                                                           */
/* Author: 	 06-Oct-15 by Ali Marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          100     // Minimum sensibility value
#define MAX_SENS          1024    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  9	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)

#define RULE1_WEIGHT        -0.8	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.12   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        -0.8	   // Weight of dispersion rule. default 0.02/10

#define BRAITENBERG_WEIGHT 4.0f

#define MIGRATION_WEIGHT    (0.04/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define OBSTACLE_SCENARIO 1

#define ABS(x) ((x>=0)?(x):-(x))

float maxSpeed = MAX_SPEED_WEB * WHEEL_RADIUS;

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver2;		// Handle for the receiver node
WbDeviceTag emitter2;		// Handle for the emitter node

char* robot_name;
int robot_id_u, robot_id;				// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int initialized[FLOCK_SIZE];			// != 0 if initial positions have been received
int start, end;							// In crossing scenario to detect which robots are in the same flock

float relative_pos[FLOCK_SIZE][3];		// relative X, Z, Theta of all robots
float my_position[3];     				// X, Z, Theta of the current robot

float new_speed[2] = { 0.0f, 0.0f };	//Speed goal for next iteration
float migr[2] = { 0.0f, 0.0f };	        // Migration vector

/*
 * Reset the robot's devices and get its ID
 */
static void reset() 
{
	wb_robot_init();

	receiver2 = wb_robot_get_device("receiver");
	emitter2 = wb_robot_get_device("emitter");
	
	/*Webots 2018b*/
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	/*Webots 2018b*/
	
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;							// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++) {
		wb_distance_sensor_enable(ds[i],64);
	}

	wb_receiver_enable(receiver2,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); 	// read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  		// normalize between 0 and FLOCK_SIZE-1

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  				// Set initialization to 0 (= not yet initialized)
	}

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

	printf("Reset: robot %d\n",robot_id_u);
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

	//printf("Robot %d, posX = %f, posy = %f, theta = %f\n", robot_id, my_position[0], my_position[1], my_position[2]);

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

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

	//printf("bearing = %f, u = %f, w = %f, msl = %d, msr = %d\n", bearing, u, w, *msl, *msr);
	//printf("Final speed computed msl = %d, msr = %d\n", *msl, *msr);
}

/*
 *  Update speed according to modified Reynold's rules
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
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter2,out,strlen(out)+1); 
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;

	while (wb_receiver_get_queue_length(receiver2) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver2);
		message_direction = wb_receiver_get_emitter_direction(receiver2);
		message_rssi = wb_receiver_get_signal_strength(receiver2);
		double y = message_direction[2];
		double x = message_direction[0];

		theta =	-atan2(y,x);
		range = sqrt((1/message_rssi));
		
		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos

		//printf("Nb : %d, rel_0 = %f, rel_1 = %f, theta = %f, range = %f, x = %f, y = %f\n", other_robot_id, relative_pos[other_robot_id][0], relative_pos[other_robot_id][1], theta, range, x, y);
		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
	
		wb_receiver_next_packet(receiver2);
	}
}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	
	reset();			// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 
	
	// Forever
	for(;;){

		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
		new_speed[0] = 0.0f;
		new_speed[1] = 0.0f;

		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) 
		{
			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
			//printf("Robot %d, capteur %d, distance %d\n",robot_id, i, distances[i] );

			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		// Send position to flockmate
		send_ping();

		update_self_motion(msl,msr);

		// Get last positions from flockmates
		process_received_ping_messages();

		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();

		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);

		max_sens = max_sens > MAX_SENS ? MAX_SENS : max_sens;

		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2.0*MAX_SENS);
			msr -= msr*max_sens/(2.0*MAX_SENS);
		}
		//printf("Final speed : msl = %d, msr = %d, bmsl = %d, bmsr = %d\n", msl, msr, bmsl, bmsr);

		// Add Braitenberg
		msl += BRAITENBERG_WEIGHT * bmsl;
		msr += BRAITENBERG_WEIGHT * bmsr;

		limit(&msl,1000);
		limit(&msr,1000);
		
		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1001;
		msr_w = msr*MAX_SPEED_WEB/1001;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}
