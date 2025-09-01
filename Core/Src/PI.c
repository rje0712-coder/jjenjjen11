#include "main.h"
#include "tim.h"
#include "PI.h"


float pi_control(pi_motor* motor_now, gain_val*gains, pi_params *params){

float return_output;
const float intg_limit = 30.0f;

float error = motor_now->target_val - motor_now->current_val;



// 적분
motor_now->intg_error += error * params-> dt;
if (motor_now->intg_error >= intg_limit) motor_now->intg_error = intg_limit;
else if(motor_now->intg_error <= -intg_limit) motor_now ->intg_error = -intg_limit;

return_output =(gains->kp * error) + (gains->ki * motor_now ->intg_error);

if (return_output >= params->max_limit) return_output = params->max_limit;
else if (return_output <= params->min_limit) return_output = params->min_limit;


return return_output;}


float PID_control(pi_motor* motor_now, gain_val*gains, pi_params *params){

float return_output;
static float prev_error = 0;

float error = motor_now->target_val - motor_now->current_val;
float derivative = error - prev_error;


// 적분
motor_now->intg_error += error * params-> dt;
if (motor_now->intg_error >= params->intg_limit) motor_now->intg_error = params->intg_limit;
else if(motor_now->intg_error <= -params->intg_limit) motor_now ->intg_error = -params->intg_limit;

return_output =(gains->kp * error) + (gains->ki * motor_now ->intg_error) + (gains->kd *derivative) ;

if (return_output >= params->max_limit) return_output = params->max_limit;
else if (return_output <= params->min_limit) return_output = params->min_limit;
prev_error = error;

return return_output;}

void change_gain_to_easy(gain_val *change_gain,float kp,float ki,float kd){
	change_gain->kp = kp;
	change_gain->ki = ki;
	change_gain->kd = kd;
}


