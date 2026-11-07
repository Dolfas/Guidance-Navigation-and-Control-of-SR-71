#include "spacecraft_structs.h" // for the struct Spacecraft_State

#ifndef GUIDANCE_H
#define GUIDANCE_H

spacecraft_guidance_points spacecraft_guidance(spacecraft_guidance_points guidance_state1, spacecraft_guidance_points guidance_state2, spacecraft_state state, double extension_constant);

#endif