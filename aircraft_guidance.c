#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "spacecraft_structs.h"
#include "guidance.h"



spacecraft_guidance_points spacecraft_guidance(spacecraft_guidance_points guidance_state1, spacecraft_guidance_points guidance_state2, spacecraft_state state, double extension_constant){

    spacecraft_guidance_points result;

    // returning the guidance point 
    result.reference_position_x = (1+extension_constant)*guidance_state2.reference_position_x - extension_constant*guidance_state1.reference_position_x;
    result.reference_position_y = (1+extension_constant)*guidance_state2.reference_position_y - extension_constant*guidance_state1.reference_position_y; 

    return result; 
};
