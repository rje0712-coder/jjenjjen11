#ifndef INC_PI_H_
#define INC_PI_H_

#include "main.h"
#include "tim.h"

/**************type def*********/
typedef struct{
	float dt; // dt
	float max_limit;
	float min_limit;
	float intg_limit;

}pi_params;

typedef struct{
	 uint8_t direction;// 방향
	 float current_val;// error
	 float target_val;// 목표값
	 float intg_error;
}pi_motor;

typedef struct{
	float ki;
	float kp;
	float kd;
}gain_val;


/*************************func*************/
float pi_control(pi_motor* motor_now, gain_val*gains,pi_params *params);
void change_gain_to_easy(gain_val *change_gain,float kp,float ki,float kd);
float PID_control(pi_motor* motor_now, gain_val*gains, pi_params *params);




#endif /* INC_PI_H_ */
