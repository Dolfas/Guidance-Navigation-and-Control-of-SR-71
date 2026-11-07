#ifndef MY_TYPES_H
#define MY_TYPES_H

typedef struct {
    
    double position_x;
    double position_y;
    double theta;
    double linear_acceleration; 
    double angular_acceleration; 
    double linear_velocity;
    double angular_velocity;
    

} spacecraft_state;

typedef struct {

    double reference_position_x;
    double reference_position_y;


} spacecraft_guidance_points;


typedef struct {

    double control_torque;
    double control_thrust; 
    double accumulated_error;

} spacecraft_control_actions;


typedef struct {

    double error_x;
    double error_y;

} spacecraft_error_state;



#endif