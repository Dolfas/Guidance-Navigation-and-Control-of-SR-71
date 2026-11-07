#include <stdio.h>
#include <stdlib.h>

#include "spacecraft_structs.h"
#include "navigation.h"




spacecraft_state navigation_estimate(spacecraft_state state){
    
    // simualte GPS and Compass readings with error 
    // noisy state estimate of the spacecraft 

    double noise = ((float)rand() / RAND_MAX)*2-1; // this returns values between [-1,1]
    
    state.position_x = state.position_x  + noise * 90; // Adding noise to position 
    state.position_y = state.position_y  + noise * 90;

    state.theta = state.theta + noise*0.01; // adding noise to orientation 

    state.linear_velocity = state.linear_velocity + noise * 2; //adding noise to the linear velocity of the spacecraft

    return state; 
};



