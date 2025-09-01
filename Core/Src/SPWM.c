#include "main.h"
#include "tim.h"
#include "SPWM.h"


void generate_spwm_val(three_phase_volt *input,three_phase_duty *output){
	float f_duty_a =  (0.5f+((input->va/input->input_vdc)*0.5f)) * 999.0f ;
	float f_duty_b =  (0.5f+((input->vb/input->input_vdc)*0.5f)) * 999.0f ;
	float f_duty_c =  (0.5f+((input->vc/input->input_vdc)*0.5f)) * 999.0f ;

	output->duty_a	= (int)f_duty_a;
	output->duty_b	= (int)f_duty_b;
	output->duty_c	= (int)f_duty_c;
}
void Insert_duty(three_phase_duty *insert_duty){
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,insert_duty->duty_a);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,insert_duty->duty_b);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,insert_duty->duty_c);
}

void change_volt_val(three_phase_volt *input,float a,float b,float c,float vdc){
	input->va = a;
	input->vb = b;
	input->vc = c;
	input->input_vdc = vdc;
}
