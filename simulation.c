#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "spacecraft_structs.h"
#include "navigation.h"
#include "guidance.h"
#include "dynamics.h"
#include "aircraft_control.h"

int waypoint_verification_function(spacecraft_guidance_points guidance_state1, spacecraft_guidance_points guidance_state2, spacecraft_state state,int waypoints_counter){
     
    // computing the distance between the two waypoints
    double distance_between_waypoints;
    distance_between_waypoints = sqrt((guidance_state2.reference_position_x-guidance_state1.reference_position_x)*(guidance_state2.reference_position_x-guidance_state1.reference_position_x)+(guidance_state2.reference_position_y-guidance_state1.reference_position_y)*(guidance_state2.reference_position_y-guidance_state1.reference_position_y));



    // this is equal to the dot product between the k and u vector
    double distance_from_plane_to_waypoint2;
    distance_from_plane_to_waypoint2 = ((state.position_x-guidance_state1.reference_position_x)*(guidance_state2.reference_position_x-guidance_state1.reference_position_x) + (state.position_y-guidance_state1.reference_position_y)*(guidance_state2.reference_position_y-guidance_state1.reference_position_y))/distance_between_waypoints;


    // incrementing or not the waypoint counter 
    if (distance_from_plane_to_waypoint2 >= distance_between_waypoints){
        // if the plane has passed the second waypoint then increment the counter and switch the waypoint to track 
        waypoints_counter += 1;
    }

    //printf("s: %f\n",distance_from_plane_to_waypoint2);
   // printf("length: %f\n",distance_between_waypoints);

    return waypoints_counter; 
}


int main(){

    double time_step = 1;
    double current_time = 0;
    double simulation_time_limit = 4000; //simulation time must be 10000
    double maximum_velocity = 1000; 
    double extension_constant = 1; // maximum_velocity * time_step * 0.1; // 30000

    spacecraft_error_state error_state;

    spacecraft_state state;
    state.position_x = 0;
    state.position_y = 0;
    state.theta = 0.2; //radians 
    state.linear_acceleration=0; 
    state.angular_acceleration=0; 
    state.linear_velocity=0;
    state.angular_velocity=0;

    spacecraft_state navigation_state;
    navigation_state.position_x = 0;
    navigation_state.position_y = 0;
    navigation_state.theta = 0;
    navigation_state.linear_velocity = 0;
 

    spacecraft_control_actions control_actions;
    control_actions.control_thrust = 0;
    control_actions.control_torque = 0;
    control_actions.accumulated_error = 0;

    spacecraft_guidance_points guidance_state;
    guidance_state.reference_position_x = 0;
    guidance_state.reference_position_y = 0;


    spacecraft_guidance_points waypoints[5] = {
          {0.0, 0.0}, {2000.0, 4000.0}, {10000.0, 6000.0}, {17000.0, 2000.0}, {26000.0, 10000.0},
    };
    int num_waypoints =  sizeof(waypoints) / sizeof(waypoints[0]);; 
    int waypoints_counter = 0; 

   
    // Open file for writing
    FILE *file = fopen("aircraft_state.csv", "w");
  
    if (file == NULL) {
        printf("Error opening trajectory.csv\n");
        return 1;
    }

    // CSV header
    fprintf(file, "time,x,y,vel,gx,gy\n");

    printf("The Simulation is beginning...\n");

    while (current_time < simulation_time_limit){

        current_time += time_step;

        // save aircraft state 
        fprintf(file, "%f,%f,%f,%f,%f,%f\n", current_time,state.position_x,state.position_y,state.linear_velocity,guidance_state.reference_position_x,guidance_state.reference_position_y);

        // aircraft dynamics module
        state = spacecraft_dynamics(state, control_actions.control_thrust, control_actions.control_torque, time_step);

        // navigation module 
        navigation_state = navigation_estimate(state);

        // guidance module
       
        waypoints_counter = waypoint_verification_function(waypoints[waypoints_counter], waypoints[waypoints_counter+1],navigation_state,waypoints_counter);

        if (waypoints_counter == num_waypoints-1){
            // in case all of the waypoints have been reached then stop the simulation 
            break; 
        }


        guidance_state = spacecraft_guidance(waypoints[waypoints_counter], waypoints[waypoints_counter+1], state, extension_constant);   

        // error computation
        error_state.error_x = guidance_state.reference_position_x-navigation_state.position_x;
        error_state.error_y = guidance_state.reference_position_y-navigation_state.position_y;
    
        // controller module 
        control_actions = aircraft_controller(control_actions,error_state,navigation_state, time_step);

    }

    fclose(file);
    printf("Simulation finished.\n");

    return 0;
}



