#include "spacecraft_structs.h" // for the struct Spacecraft_State

#ifndef SPACECRAFT_DYNAMICS_H
#define SPACECRAFT_DYNAMICS_H

spacecraft_state spacecraft_dynamics(spacecraft_state state, double control_thrust, double control_torque, double time_step);


#endif