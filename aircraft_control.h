#include "spacecraft_structs.h" // for the struct Spacecraft_State

#ifndef AIRCRAFT_CONTROL_H
#define AIRCRAFT_CONTROL_H

spacecraft_control_actions aircraft_controller(spacecraft_control_actions control_actions, spacecraft_error_state error_state, spacecraft_state state, double timestep);

#endif