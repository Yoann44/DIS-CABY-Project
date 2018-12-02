/*****************************************************************************/
/* File:         performance_estimation.c                                    */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  estimating the performance of a formation 		     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5 		// Number of robots in flock (varying depending on the scenario)
#define TIME_STEP	64		// [ms] Length of time step

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define RULE1_THRESHOLD 0.2
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0

#define v_e_puck_max 0.1287		// [m/s] E-puck maximum speed
#define SCENARIO 0                      //0 OBSTACLES, 1 CROSSING 

int offset;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation
int t;

/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


/*
 * Compute performance metric.
 */
void compute_fitness(float* fit_c, float* fit_o) {
	*fit_c = 0; *fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float angle_diff;
	int i; int j;
	for (i=0;i<FLOCK_SIZE;i++) {
		for (j=i+1;j<FLOCK_SIZE;j++) 
		{	
			// Distance measure for each pair ob robots
			*fit_c += fabs(sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2))-RULE1_THRESHOLD*2);
		}

		// Angle measure for each robot
		angle_diff = fabsf(loc[i][2]-orient_migr);
		*fit_o += angle_diff > M_PI ? 2*M_PI-angle_diff : angle_diff;
	}
	*fit_c /= FLOCK_SIZE*(FLOCK_SIZE+1)/2;
	*fit_o /= FLOCK_SIZE;
}

//Compute metrics for the project
void compute_metric(float* met,float* cm_pos,int offset){
	float or_cos=0.; float or_sin=0.;
    	float dist=0.; float proj=0.;       //Distance between robot pos and cm, projection of velo of cm onto migratory urge
	float new_cm_pos[2]= {0,0};		//Position of center of mass of the swarm
	int i; int j;
	int f_size = FLOCK_SIZE;
	
	if (SCENARIO == 1) {
          	f_size = FLOCK_SIZE/2;
         }
	for (i=0;i<f_size;i++)
	{	
		or_cos+= cos(loc[i+offset][2]);
		or_sin+= sin(loc[i+offset][2]);
		for (j=0;j<2;j++){
			new_cm_pos[j] += loc[i+offset][j];
		}
	}
	// Compute center of mass of the swarm
	for (j=0;j<2;j++){
		new_cm_pos[j] /= f_size;
	}
	for (i=0;i<f_size;i++){
		dist+=sqrtf(powf(loc[i+offset][0]-new_cm_pos[0],2)+powf(loc[i+offset][1]-new_cm_pos[1],2));
	}
	
	// Orientation metric
	met[0]=sqrtf(powf(or_cos,2)+powf(or_sin,2))/f_size;	
	
    // Cohesion metric
	met[1]=1/(1+dist/f_size);
    
    // Velocity metric
    // Compute projection of the velocity of the center of mass onto the migratory urge vector
    proj = (new_cm_pos[0]-cm_pos[0]) * migrx + (new_cm_pos[1]-cm_pos[1]) * migrz;
    proj /= sqrtf(powf(migrx,2)+powf(migrz,2)) * TIME_STEP / 1000.;
    met[2]=fmax(proj,0)/v_e_puck_max;
    
	cm_pos[0]=new_cm_pos[0];
	cm_pos[1]=new_cm_pos[1];
    
    //Performance instant
    met[3]=met[0]*met[1]*met[2];
    
    //Overall Performance
    met[4]+=met[3];
}



/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index
  
	if (argc == 4) { // Get parameters
		offset = atoi(args[1]);
		migrx = atof(args[2]);
		migrz = atof(args[3]);
		//migration goal point comes from the controller arguments. It is defined in the world-file, under "controllerArgs" of the supervisor.
		printf("Migratory instinct : (%f, %f)\n", migrx, migrz);
	} else {
    		offset = 0.;
		migrx = 1.0;
		migrz = 0.0;
		printf("Missing argument\n");
		//return 1;
	}
	
	orient_migr = -atan2f(migrx,migrz);
	
	if (orient_migr<0) {
		orient_migr+=2*M_PI; // Keep value within 0, 2pi
	}
	reset();

	
	// Compute reference fitness values
	
	float fit_cluster;			// Performance metric for aggregation
	float fit_orient;			// Performance metric for orientation
	
	// Compute metrics for project
	float metrics[5]={0.,0.,0.,0.,0.}; 	//Table with the 5 metrics
	float metrics_fl_1[5]={0.,0.,0.,0.,0.};  //Metrics for group 1
        float metrics_fl_2[5]={0.,0.,0.,0.,0.};  //Metrics for group 2
	float cm_pos[2]={0.,0.};
	int offset = 0;
	
	int nb_step = 0;		// Position of center of mass (old position)
	
	for(;;) {
		wb_robot_step(TIME_STEP);
		t += TIME_STEP;
		nb_step++;
		
		for (i=0;i<FLOCK_SIZE;i++) {
			// Get data
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
		}
			//Compute and normalize fitness values
		compute_fitness(&fit_cluster, &fit_orient);
		fit_cluster = fit_cluster_ref/fit_cluster;
		fit_orient = 1-fit_orient/M_PI;
		printf("time:%d, Topology Performance: %f\n", t, fit_cluster);
			
			//compute metric values			
		if (SCENARIO == 1){
            		compute_metric(metrics_fl_1, cm_pos, offset);
                  	offset = FLOCK_SIZE/2;
                  	compute_metric(metrics_fl_2, cm_pos, offset);
                  	offset = 0;
                  	for (i=0; i<5; i++)  {
                              	metrics[i] = (metrics_fl_1[i]+metrics_fl_2[i])/2;
                        }
                  } else {
  			compute_metric(metrics, cm_pos, offset);
  		}
		
        	float over_perf = metrics[4] / nb_step;
		
		if (t % 10 == 0) {
			printf("time:%d, orientation: %f, cohesion: %f, velocity: %f, ins_perf: %f, over_perf: %f\n"
			, t, metrics[0], metrics[1], metrics[2], metrics[3], over_perf);
		}

	}

}
