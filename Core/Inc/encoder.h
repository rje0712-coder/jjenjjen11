
#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_
/*********************** include ****************************/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include"lpf.h"
#include"stuct.h"


/***************************** type def **********************/


 /**************************** function *************************/
  //엔코더 초기화 함수
 void Encoder_Init(TIM_HandleTypeDef *htim, encoder_instance* g_enc_instance_mot,uint16_t ppr,float pulse);
 void Encoder_Update(TIM_HandleTypeDef *htim,encoder_instance* g_enc_instance_mot); // 엔코더 업데이트 함수
 float Encoder_Get_ang_vel(encoder_instance* g_enc_instance_mot,LPF_io* lpf_ang_vel,float Ts); // 각 속도값 반환 함수
 float Encoder_Get_ang(encoder_instance* g_enc_instance_mot);
 void genius_encoder_algorithm(TIM_HandleTypeDef *htim,encoder_instance* g_enc_instance_mot,float timer_ARR_val);
 void update_state(encoder_instance*g_pandulm_instance_mot, encoder_instance* g_encoder_instance,state_struct *state_dot,state_struct *state_hat, state_struct *L_gains,state_struct *Lm_gains,observer_param *observer_param, float u_f,float offset);
 void disturbance_observer(encoder_instance* g_enc_instance_mot, disturbance_param* g_disturbance,observer_param* g_observer_param, DQ_t* g_volt_dq, LPF_io* lpf_ang_vel, LPF_io*lpf_observer , float observer_input);
#endif /* INC_ENCODER_H_ */
