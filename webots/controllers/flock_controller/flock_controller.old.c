/*****************************************************************************/
/* File:         flocking_supper.c                                           */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  Reynolds flocking control 				     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
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

#define NB_SENSORS			8	  	// Number of distance sensors
#define MIN_SENS			350     // Minimum sensibility value
#define MAX_SENS			4096    // Maximum sensibility value
#define MAX_SPEED			800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB		6.28    // Maximum speed webots
/*Webots 2018b*/	
#define FLOCK_SIZE			10	  	// Size of flock
#define TIME_STEP			64	  	// [ms] Length of time step

#define AXLE_LENGTH			0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T				0.064	// Timestep (seconds)


#define RULE1_THRESHOLD		0.25   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT		(0.6/10)	// Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD		0.15   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT		(0.02/10)	// Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT		(1.0/10)   	// Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT	(0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE		1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

#define USE_SUPERVISOR		0

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//int e_puck_matrix[16] = {17,29,12,10,8,-38,-56,-76,-72,-58,-36,8,10,12,28,18}; // Crossing

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
float theta_robots[FLOCK_SIZE];
bool pos_valid[FLOCK_SIZE];

float loc[FLOCK_SIZE][3];	// X, Z, Theta of all robots
float prev_loc[FLOCK_SIZE][3];	// Previous X, Z, Theta values
float speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int   initialized[FLOCK_SIZE];	// != 0 if initial positions have been received
float migr[2] = {0.f,100.f};	                // Migration vector

char* robot_name; 

/*
 * Reset the robot's devices and get its ID
 *
 */
static void reset() {
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");

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
		s[2]++;				// increases the device number
	}

	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++) {
		wb_distance_sensor_enable(ds[i],64);
	}
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	for(i=0; i<FLOCK_SIZE; i++) 
	{
		initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	}

	printf("Reset: robot %d\n",robot_id_u);
}

/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {
	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 * Used for odometry
 */
void update_self_motion(int msl, int msr) {
	float theta = loc[robot_id][2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	loc[robot_id][0] += dx;
	loc[robot_id][1] += dz;
	loc[robot_id][2] += dtheta;
	
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
	
	if (loc[robot_id][2] > 2*M_PI) loc[robot_id][2] -= 2.0*M_PI;
	if (loc[robot_id][2] < 0) loc[robot_id][2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	//float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	//float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	float x = speed[robot_id][0];
	float z = speed[robot_id][1];
	if(USE_SUPERVISOR) {
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	}
	if(robot_id == 1) {
  	//printf("speed_0 = %f, speed_1 = %f, theta = %f\n", speed[robot_id][0], speed[robot_id][1], loc[robot_id][2]);
	//printf("x = %f, y = %f\n", x, z);
	}
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 1;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
//	printf("bearing = %f, u = %f, w = %f, msl = %f, msr = %f\n", bearing, u, w, msl, msr);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}

/*
 * Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i, j, k;			// Loop counters
	float avg_loc[2] = {0,0};	// Flock average positions
	float avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};

	int nb_neighbors = 0;
	bool neighbors_used = false;
	
	/* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++) {
		if (i == robot_id) 
		{	
			// don't consider yourself for the average 
			continue;
		}
		for (j=0;j<2;j++) 
		{
			if(USE_SUPERVISOR) {
				avg_speed[j] += speed[i][j];
				avg_loc[j] += loc[i][j];
				neighbors_used = true;

			}
			else if(pos_valid[i]) {
				avg_speed[j] += relative_speed[i][j];
				avg_loc[j] += relative_pos[i][j];
				//printf("Nb : %d, rel_pos_0 = %f - rel_pos_1 = %f\n",i , relative_pos[i][0], relative_pos[i][1]);
  			neighbors_used = true;
			}
		}
		if(neighbors_used){
				nb_neighbors++;
				neighbors_used = false;
		}
	}
	
	for (j=0;j<2;j++) 
	{
		avg_speed[j] /= nb_neighbors;
		avg_loc[j] /= nb_neighbors;
	}
	
	printf("Nb nb = %d, Avg_loc_0 = %f, Avg_loc_1 = %f\n",nb_neighbors, avg_loc[0], avg_loc[1]);
	
	
	
	/* Reynold's rules */
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
		// If center of mass is too far
		if(USE_SUPERVISOR) {
			if (sqrt(pow(loc[robot_id][0]-avg_loc[0],2)+pow(loc[robot_id][1]-avg_loc[1],2)) > RULE1_THRESHOLD) {
				cohesion[0] = avg_loc[0] - loc[robot_id][0];   // Relative distance to the center of the swarm
				cohesion[1] = avg_loc[1] - loc[robot_id][1];   // Relative distance to the center of the swarm
			}
		}
	else {
  	float distance = sqrt(pow(avg_loc[0],2)+pow(avg_loc[1],2));
          //printf("Avg_dist = %f, Avg_loc_0 = %f, Avg_loc_1 = %f\n", distance, avg_loc[0], avg_loc[1]);  	
		if (sqrt(pow(avg_loc[0],2)+pow(avg_loc[1],2)) > RULE1_THRESHOLD) {
				cohesion[0] = avg_loc[0];   // Relative distance to the center of the swarm
				cohesion[1] = avg_loc[1];   // Relative distance to the center of the swarm
			}
		}

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {        // Loop on flockmates only
			// If neighbor k is too close (Euclidean distance)
			if(USE_SUPERVISOR) {
				if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD) {
					for (j=0;j<2;j++) {
						dispersion[j] += 1/(loc[robot_id][j] -loc[k][j]);	// Relative distance to k
					}
				}
			}
			else if(pos_valid[k]){
				if (sqrt(pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2)) < RULE2_THRESHOLD) {
					for (j=0;j<2;j++) {
						dispersion[j] -= 1/relative_pos[k][j];	// Relative distance to k
					}
				}
			}
		}
	}

	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	consistency[0] = 0;
	consistency[1] = 0;
	/* add code for consistency[j]*/
	if(USE_SUPERVISOR) {
		for (j=0;j<2;j++) {
			consistency[j] = 0; 
		}
	}
	else {
		for (j=0;j<2;j++) {
			consistency[j] = avg_speed[j]; 
		}
	}

	// aggregation of all behaviors with relative influence determined by weights
	//printf("id = %d, cx = %f, cy = %f\n", robot_id, cohesion[0], cohesion[1]);
	//speed[robot_id][0] = 10.0f;
	//speed[robot_id][1] = 0.0f;
	for (j=0;j<2;j++) {
	//speed[robot_id][j] = 0;
		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		//speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		//speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
	}
	//speed[robot_id][1] *= -1; //y axis of webots is inverted

	//move the robot according to some migration rule
	if(USE_SUPERVISOR) {
		if(MIGRATORY_URGE == 0) {
			speed[robot_id][0] += 0.01*cos(loc[robot_id][2] + M_PI/2);
			speed[robot_id][1] += 0.01*sin(loc[robot_id][2] + M_PI/2);
		}
		else {
			/* Implement migratory urge */
			speed[robot_id][0] += 0;
			speed[robot_id][1] -= 0; //y axis of webots is inverted
		}
	}
	else {
		if(MIGRATORY_URGE == 0) {
			speed[robot_id][0] += 0.01*cos(loc[robot_id][2] + M_PI/2);
			speed[robot_id][1] += 0.01*sin(loc[robot_id][2] + M_PI/2);
		}
		else {
		    /* Implement migratory urge */
			speed[robot_id][0] += MIGRATION_WEIGHT * migr[0] * sinf(my_position[2]);
		    speed[robot_id][1] += MIGRATION_WEIGHT * migr[1] * cosf(my_position[2]); //y axis of webots is inverted
  		printf("CouCou\n");
		}
	}
}

/*
 * Initialize robot's position
 */
void initial_pos(void){
	char *inbuffer;	
	int rob_nb;
	float rob_x, rob_z, rob_theta; // Robot position and orientation

	while (initialized[robot_id] == 0) {
		
		// wait for message
		while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);
		
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta, &migr[0], &migr[1]);
		// Only info about self will be taken into account at first.

		//robot_nb %= FLOCK_SIZE;
		if (rob_nb == robot_id) 
		{
			// Initialize self position
			loc[rob_nb][0] = rob_x; 		// x-position
			loc[rob_nb][1] = rob_z; 		// z-position
			loc[rob_nb][2] = rob_theta; 		// theta
			prev_loc[rob_nb][0] = loc[rob_nb][0];
			prev_loc[rob_nb][1] = loc[rob_nb][1];
			initialized[rob_nb] = 1; 		// initialized = true
		}		
		wb_receiver_next_packet(receiver);
	}	
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
	wb_emitter_send(emitter,out,strlen(out)+1); 
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

	for (int k=0;k<FLOCK_SIZE;k++) {
		pos_valid[k] = false;
	}

	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		//X pointing forward, Y pointing right
		double x = -message_direction[2];
		double y = message_direction[0];
		
		//if(robot_id == 1)
		//printf("X:%f, Y:%f, Z:%f\n", message_direction[0],message_direction[2], message_direction[2]);

		theta = atan2(y,x);
		//theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));
		
		//if(robot_id == 1)
  		//printf("This id = %d, my_pos_theta = %f, theta = %f, range = %f\n", robot_id, my_position[2], theta, range);
		

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
		
		pos_valid[other_robot_id] = true;

		// Get position update
		//theta += dtheta_g[other_robot_id];
		//theta_robots[other_robot_id] = 0.8*theta_robots[other_robot_id] + 0.2*theta;
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][1] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][0] = range*sin(theta);   // relative y pos
		
		//if(robot_id == 1)
  		//printf("id = %d, rel_x = %f, rel_y = %f\n", other_robot_id, relative_pos[other_robot_id][0], relative_pos[other_robot_id][1]);

		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
		
		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);		
		 
		wb_receiver_next_packet(receiver);
	}
}

/*
 * Main function
*/
int main(){ 	

	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int rob_nb;			// Robot number
	float rob_x, rob_z, rob_theta;  // Robot position and orientation
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	char *inbuffer;			// Buffer for the receiver node
	int max_sens;			// Store highest sensor value

	reset();			// Resetting the robot
	if(USE_SUPERVISOR) {
		initial_pos();		// Initializing the robot's position
	}
	
	msl = 0; msr = 0;  
	max_sens = 0;
	
	loc[robot_id][0] = 0;
	loc[robot_id][1] = 0;
	loc[robot_id][2] = 0;
	
	prev_loc[robot_id][0] = 0;
	prev_loc[robot_id][1] = 0;
	prev_loc[robot_id][2] = 0;
	
	my_position[0] = 0;
	my_position[1] = 0;
	my_position[2] = 0;
	
	prev_my_position[0] = 0;
	prev_my_position[1] = 0;
	prev_my_position[2] = 0;
	
	// Forever
	for(;;){
		
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
		/* Braitenberg */
		/*
		for(i=0;i<NB_SENSORS;i++) {
			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenberg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;
		*/

		/* Get information */
		if(USE_SUPERVISOR) {
			int count = 0;
			while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE) {
				inbuffer = (char*) wb_receiver_get_data(receiver);
				sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta);

				if ((int) rob_nb/FLOCK_SIZE == (int) robot_id/FLOCK_SIZE) {
					rob_nb %= FLOCK_SIZE;
					if (initialized[rob_nb] == 0) {
						// Get initial positions
						loc[rob_nb][0] = rob_x; //x-position
						loc[rob_nb][1] = rob_z; //z-position
						loc[rob_nb][2] = rob_theta; //theta
						prev_loc[rob_nb][0] = loc[rob_nb][0];
						prev_loc[rob_nb][1] = loc[rob_nb][1];
						initialized[rob_nb] = 1;
					}
					else {
						// Get position update
						//printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
						prev_loc[rob_nb][0] = loc[rob_nb][0];
						prev_loc[rob_nb][1] = loc[rob_nb][1];
						loc[rob_nb][0] = rob_x; //x-position
						loc[rob_nb][1] = rob_z; //z-position
						loc[rob_nb][2] = rob_theta; //theta
					}
				
					speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
					speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
					count++;
				}

				wb_receiver_next_packet(receiver);
			}
			// Compute self position & speed
			prev_loc[robot_id][0] = loc[robot_id][0];
			prev_loc[robot_id][1] = loc[robot_id][1];

			update_self_motion(msl,msr);
		}
		else {
			send_ping();
			// Compute self position
			prev_my_position[0] = my_position[0];
			prev_my_position[1] = my_position[1];
			prev_my_position[2] = my_position[2];
			
			prev_loc[robot_id][0] = loc[robot_id][0];
			prev_loc[robot_id][1] = loc[robot_id][1];
			prev_loc[robot_id][2] = loc[robot_id][2];

			update_self_motion(msl,msr);

			process_received_ping_messages();
		}		

		speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);

		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		//printf("%f %f\n", speed[robot_id][0], speed[robot_id][1]);

		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&msl, &msr);

		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}

		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;

		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		/*Webots 2018b*/

		// Send current position to neighbors, uncomment for I14, don't forget to uncomment the declration of "outbuffer" at the begining of this function.
		
		/*Implement your code here*/

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}

