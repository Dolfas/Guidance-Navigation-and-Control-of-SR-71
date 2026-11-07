#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "spacecraft_structs.h"
#include "dynamics.h"


spacecraft_state spacecraft_dynamics(spacecraft_state state, double control_thrust, double control_torque, double time_step){

    // state variable holds the previous position and heading of the spacecraft
    
    // simulate spacecraft dynamics with no error
    // returns position and heading of spacecraft
    
    double pi = 2*acos(0.0);

    // Spacecraft definitions
    double spacecraft_mass = 69000; // kg 
    double spacecraft_inertia = 6148417.5; // kg * m^2

    
    // Coeficcient defintions
    double c_velocity_drag = 0.3; // kg/m
    double c_rotation_drag = 12300000; //kg * m^2 


    // Updating linear acceleration 
    state.linear_acceleration = (control_thrust-c_velocity_drag*state.linear_velocity*state.linear_velocity)/spacecraft_mass;

    // Updating angular acceleration 
    state.angular_acceleration = (control_torque-c_rotation_drag*state.angular_velocity*fabs(state.angular_velocity))/spacecraft_inertia;

    // Updating linear velocity
    state.linear_velocity = state.linear_velocity + state.linear_acceleration * time_step;
    
    // Updating angular velocity
    state.angular_velocity = state.angular_velocity + state.angular_acceleration * time_step;

    // Updating theta
    state.theta = state.theta + state.angular_velocity * time_step;
    //state.theta = (fmod(state.theta,2*pi)); // angle between 0 and 2pi
    //if (state.theta > pi){
      //  state.theta = state.theta - 2*pi;
    //} // the angle is kept between -pi and pi

    state.theta = atan2(sin(state.theta), cos(state.theta)); // keeping the angle theta between -pi and +pi



    // Updating position 
    state.position_x = state.position_x +  state.linear_velocity * cos(state.theta) * time_step; 
    state.position_y = state.position_y +  state.linear_velocity * sin(state.theta) * time_step;

    return state; 
};











