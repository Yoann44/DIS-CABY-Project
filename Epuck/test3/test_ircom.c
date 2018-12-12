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

#define BRAITENBERG_WEIGHT 4.0f

#define ABS(x) ((x>=0)?(x):-(x))
#define M_PI 3.141592

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

int robot_id;

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots

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
    
    //e_calibrate_ir();

	btcomSendString("Reset finish \r\n");
    
    robot_id = 0;
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
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{
	btcomSendString("Send start \r\n");
	ircomSend(robot_id);
	while(ircomSendDone() == 0) {
		asm("nop");
	}
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	btcomSendString("Process ping \r\n");
	int other_robot_id;
	double range;
	double real_distance;
	double direction;
	double theta;
	IrcomMessage imsg;
	
	int i = 0;
	while(i < 200) {
		ircomPopMessage(&imsg);
		if(imsg.error == 0) {
			other_robot_id = (int)imsg.value;
			char tmp[128];
			sprintf(tmp, "Robot %d;\tDistance %f;\tDirection %f\r\n", other_robot_id, (double)imsg.distance, (double)imsg.direction);
			btcomSendString(tmp);
			range = (double) imsg.distance;
			// From interpolation of real values
			real_distance = 1./0.5876*(range - 7.473);
			direction = (double) imsg.direction;
			double x = sinf(direction); // A verifier
			double y = cosf(direction);
			theta = -atan2(y, x);
			
			relative_pos[other_robot_id][0] = real_distance*cos(theta);  // relative x pos
			relative_pos[other_robot_id][1] = -1.0 * real_distance*sin(theta);   // relative y pos
			
			char tmp2[128];
			sprintf(tmp2, "Robot %d;\tDistance %f\r\n", other_robot_id, real_distance);
			btcomSendString(tmp2);
		
		}else if (imsg.error > 0) {
			btcomSendString("Receive failed \r\n");		
		}
		else if(imsg.error < 0) {	
			i++;
		}
	}
	
}


// the main function
int main(){
	/*
	int msl, msr;			// Wheel speeds
	
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	*/
	
	reset();			// Resetting the robot
	
	// initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();
    
	// max_sens = 0; 
	
	// Forever
	for(;;){
		/*
		msl = 0; msr = 0;
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;

		
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
		*/

		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

		//update_self_motion(msl,msr);

		process_received_ping_messages();

		btcomSendString("Wait start \r\n");
		long j;
		for(j=0;j<3000000;j++) {
			asm("nop");
		}
		btcomSendString("Wait end \r\n");

		// Reynold's rules with all previous info (updates the speed[][] table)
		//reynolds_rules();

		// Compute wheels speed from reynold's speed
		//compute_wheel_speeds(&msl, &msr);

		// Adapt speed instinct to distance sensor values
		/*
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}

		// Add Braitenberg
		msl += bmsl*BRAITENBERG_WEIGHT;
		msr += bmsr*BRAITENBERG_WEIGHT;
		*/
		
		
		//e_set_speed_left(msl);
		//e_set_speed_right(msr);
	}
	ircomStop();
}
