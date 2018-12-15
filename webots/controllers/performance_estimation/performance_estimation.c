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

#define FLOCK_SIZE 5		// Number of robots in flock
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
float migrx1, migrz1, migrx2, migrz2;	// Migration vectors
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
 * Compute performance metric
 */
void compute_metric(float* met,float* cm_pos,int offset_fl){
	float or_cos=0.; float or_sin=0.;
         float dist=0.; float proj=0.;       //Distance between robot pos and cm, projection of velo of cm onto migratory urge
	float new_cm_pos[2]= {0,0}; 	//Position of center of mass of the swarm
	int i; int j; 
	int f_size = FLOCK_SIZE;
	
	if (SCENARIO == 1) {
          	f_size = FLOCK_SIZE/2;
         }
	for (i=0;i<f_size;i++)
	{	
		or_cos+= cos(loc[i+offset_fl][2]);
		or_sin+= sin(loc[i+offset_fl][2]);
		for (j=0;j<2;j++){
			new_cm_pos[j] += loc[i+offset_fl][j];
		}
	}
	// Compute center of mass of the swarm
	for (j=0;j<2;j++){
		new_cm_pos[j] /= f_size;
	}
	for (i=0;i<f_size;i++){
		dist+=sqrtf(powf(loc[i+offset_fl][0]-new_cm_pos[0],2)+powf(loc[i+offset_fl][1]-new_cm_pos[1],2));
	}
	
	// Orientation metric
	met[0]=sqrtf(powf(or_cos,2)+powf(or_sin,2))/f_size;	
	printf("orient: %f\n",met[0]);
         // Cohesion metric
	met[1]=1/(1+dist/f_size);
	printf("cohesion: %f\n",met[1]);
    
         // Velocity metric
        // Compute projection of the velocity of the center of mass onto the migratory urge vector
        if (SCENARIO == 1){
                  if (offset_fl != 0) {
                            proj = (new_cm_pos[0]-cm_pos[0]) * migrx1 + (new_cm_pos[1]-cm_pos[1]) * migrz1;
                            proj /= sqrtf(powf(migrx1,2)+powf(migrz1,2)) * TIME_STEP / 1000.;
                  } else {
                            proj = (new_cm_pos[0]-cm_pos[0]) * migrx2 + (new_cm_pos[1]-cm_pos[1]) * migrz2;
                            proj /= sqrtf(powf(migrx2,2)+powf(migrz2,2)) * TIME_STEP / 1000.;
                  }
        } else {
                  proj = (new_cm_pos[0]-cm_pos[0]) * migrx1 + (new_cm_pos[1]-cm_pos[1]) * migrz1;
                  proj /= sqrtf(powf(migrx1,2)+powf(migrz1,2)) * TIME_STEP / 1000.; 
        }
        met[2]=fmax(proj,0)/v_e_puck_max;
        printf("velocity: %f, proj: %f\n",met[2], proj);
        cm_pos[0]=new_cm_pos[0];
        cm_pos[1]=new_cm_pos[1];
    
        //Performance instant
        met[3]=met[0]*met[1]*met[2];
        printf("insta perf: %f\n",met[3]);
        //Overall Performance
        met[4]+=met[3];
        printf("overall perf: %f\n",met[3]);
}

/*
 * Main function.
 */
int main(int argc, char *args[]) {
	int i,j;			// Index
  
	if (argc == 4) { // Get parameters
		offset = atoi(args[1]);
		migrx1 = atof(args[2]);
		migrz1 = atof(args[3]);
		//migration goal point comes from the controller arguments. It is defined in the world-file, under "controllerArgs" of the supervisor.
		printf("Migratory instinct : (%f, %f)\n", migrx1, migrz1);
	} else {
    		offset = 0.;
		migrx1 = 1.0;
		migrz1 = 0.0;
		migrx2 = -1.0;
		migrz2 = 0.0;
		printf("Missing argument\n");
		//return 1;
	}
	reset();

	// Compute metrics for project
	float metrics[5]={0.,0.,0.,0.,0.}; 	//Table with the 5 metrics
         float metrics_fl_1[5]={0.,0.,0.,0.,0.};  //Metrics for group 1
         float metrics_fl_2[5]={0.,0.,0.,0.,0.};  //Metrics for group 2
         float over_perf = 0.;   //Metric for the overall
        	float cm_pos_fl_1[2]={0.,0.};   // Position of center of mass of flock 1 (old position)
	float cm_pos_fl_2[2]={0.,0.};   // Position of center of mass of flock 2 (old position)
         int offset_fl = 0;  //Offset to differientiate the two flocks
         float data_fl_1[5000][5];
         float data_fl_2[5000][5];
         int index=0;
	int nb_step = 0;	
		
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
		if(t == TIME_STEP) {
          		if (SCENARIO == 1){
                              	for (i=0;i<(FLOCK_SIZE/2);i++)
                            	{
                                    	for (j=0;j<2;j++){
                            			cm_pos_fl_1[j] += loc[i][j];
                            			cm_pos_fl_2[j] += loc[i+FLOCK_SIZE/2][j];
                  		}
            		}
          		for (i=0;i<2;i++){
                  		cm_pos_fl_1[i] /= (FLOCK_SIZE/2);
                  		cm_pos_fl_2[i] /= (FLOCK_SIZE/2);
                    	}
                  	} else {
                            	for (i=0;i<(FLOCK_SIZE/2);i++)
                            	{
                                    	for (j=0;j<2;j++){
                            			cm_pos_fl_1[j] += loc[i][j];
                            		}
                  		}
                  		for (i=0;i<2;i++){
                            		cm_pos_fl_1[i] /= (FLOCK_SIZE/2);
                            	}	
                    	}
                  }	
		//compute metric values
  		if (SCENARIO == 1){
            		printf("cm_fl_1: %f %f, cm_fl_2: %f %f\n", cm_pos_fl_1[0],cm_pos_fl_1[1], cm_pos_fl_2[0],cm_pos_fl_2[1]);
            		compute_metric(metrics_fl_1, cm_pos_fl_1, offset_fl);
                  	offset_fl = FLOCK_SIZE/2;
                  	compute_metric(metrics_fl_2, cm_pos_fl_2, offset_fl);
                  	printf("cm_fl_1: %f %f, cm_fl_2: %f %f\n", cm_pos_fl_1[0],cm_pos_fl_1[1], cm_pos_fl_2[0],cm_pos_fl_2[1]);
                  	offset_fl = 0;
                  	for (i=0; i<5; i++)  {
                              	metrics[i] = (metrics_fl_1[i]+metrics_fl_2[i])/2;
                           }
                  } else {
  			compute_metric(metrics, cm_pos_fl_1, offset_fl);
  		}
          	over_perf = metrics[4] / nb_step;
		if (t % 10 == 0) {
			printf("time:%d, orientation: %f, cohesion: %f, velocity: %f, ins_perf: %f, over_perf: %f\n"
			, t, metrics[0], metrics[1], metrics[2], metrics[3], over_perf);
			if (SCENARIO == 1){
            			for (i=0;i<3;i++) {
                  			data_fl_1[index][i]=metrics_fl_1[i];
                  			data_fl_2[index][i]=metrics_fl_2[i];
            			}
          			data_fl_1[index][3] = metrics_fl_1[4] / nb_step;  // Overall performance of flock 1
          			data_fl_2[index][3] = metrics_fl_2[4] / nb_step;  // Overall performance of flock 2
          			data_fl_1[index][4] = t;
          			data_fl_2[index][4] = t;
          		} else {
          		         for (i=0;i<3;i++) {
                  			data_fl_1[index][i]=metrics[i];
            			}
          			data_fl_1[index][3] = over_perf;
          			data_fl_1[index][4] = t;
          		}
			index++;
		}
		// Save the first three metric, the overall performance metric and the time
		if (t/1000 == 32) {     //After 32 s of simulation
            		if (SCENARIO == 1){
                  		FILE *fp_fl_1;
                  		FILE *fp_fl_2;
                  		fp_fl_1 = fopen("Scalevstime_fl_1.txt","w");
                  		fp_fl_2 = fopen("Scalevstime_fl_2.txt","w");
                  		if(fp_fl_1 == NULL || fp_fl_2 == NULL) {
                              		printf("Error!");
                            		exit(1);
                            	}
                            	for (i=0;i<index;i++) {
                            		fprintf(fp_fl_1,"%f %f %f %f %f \n", data_fl_1[i][0], data_fl_1[i][1], data_fl_1[i][2], data_fl_1[i][3], data_fl_1[i][4]);
                            		fprintf(fp_fl_2,"%f %f %f %f %f \n", data_fl_2[i][0], data_fl_2[i][1], data_fl_2[i][2], data_fl_2[i][3], data_fl_2[i][4]);
                    		}
                    		fclose(fp_fl_1);
                    		fclose(fp_fl_2);
            		
            		} else {
                  		FILE *fp;
                  		fp = fopen("Scalevstime.txt","w");
                    		if(fp == NULL) {
                            		printf("Error!");
                            		exit(1);
                    		}
                    		for (i=0;i<index;i++) {
                            		fprintf(fp,"%f %f %f %f %f \n", data_fl_1[i][0], data_fl_1[i][1], data_fl_1[i][2], data_fl_1[i][3], data_fl_1[i][4]);
                    		}
                    		fclose(fp);                    		
            		}
		}

	}

}
