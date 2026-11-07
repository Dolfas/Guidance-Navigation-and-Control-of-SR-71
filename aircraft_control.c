
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "spacecraft_structs.h"
#include "aircraft_control.h"

spacecraft_control_actions aircraft_controller(spacecraft_control_actions control_actions, spacecraft_error_state error_state, spacecraft_state state, double timestep){
    // this implements a PID algorithm 
    // the control actions change: the velocity and the heading angle of the spacecraft 

    double desired_heading;
    double heading_error;

    double angular_velocity;
    double desired_velocity = 500;  // cruising speed 
    double velocity_error;
    double aerodynamic_drag_coefficient = 0.3;

    double proportional_gain_torque = 50000;
    double derivative_gain_torque = 100000;
    double proportional_gain_thrust = 2;
    double integral_gain_thrust = 1; 

    double maximum_thrust = 302e3;
    double maximum_control_torque = 4.918734e6;




    // compute desired heading
    //if (fabs(error_state.error_y) < 1e-7 && fabs(error_state.error_x) < 1e-7){

       // desired_heading = state.theta;

    //}else{
//    }
    desired_heading = atan2(error_state.error_y, error_state.error_x);

    // compute heading error 
    heading_error = desired_heading-state.theta; 
    heading_error = atan2(sin(heading_error), cos(heading_error));


    // compute velocity error 
    velocity_error = desired_velocity - state.linear_velocity;
    control_actions.accumulated_error += velocity_error * timestep;


    // compute control actions 

    // still have to include saturation of actuators 
        
        // control torque 
        // PD controller
    control_actions.control_torque = proportional_gain_torque*heading_error - derivative_gain_torque*state.angular_velocity;
    //printf("\nthe heading error is: %f",heading_error);
        // saturating the control torque to mimick real world control surfaces
    if (control_actions.control_torque > maximum_control_torque){

            control_actions.control_torque = maximum_control_torque;        
    } else if (control_actions.control_torque < -maximum_control_torque){
            control_actions.control_torque = -maximum_control_torque;
    }
    
        // control thrust 
        // PI controller 
    control_actions.control_thrust = proportional_gain_thrust * velocity_error + integral_gain_thrust * control_actions.accumulated_error; 

        // saturating the control thrust to mimick real world engines
    if (control_actions.control_thrust > maximum_thrust){

             control_actions.control_thrust = maximum_thrust;
    } 

    if (control_actions.control_thrust < 0){
            control_actions.control_thrust = 0;
    }  

   // if (control_actions.control_thrust < maximum_thrust){

     //   control_actions.accumulated_error += velocity_error * timestep;  
   //     control_actions.control_thrust = proportional_gain_thrust * velocity_error + integral_gain_thrust * control_actions.accumulated_error;

   // }

   // if (control_actions.control_thrust > maximum_thrust){
            // control_actions.control_thrust = maximum_thrust;

           //  if (velocity_error * timestep < 0){
            //    control_actions.accumulated_error += velocity_error * timestep;  
           //  }
      // } 





    
    


        
    return control_actions;
}