/*********************** include ***************************/
#include "stdio.h"
#include "encoder.h"
#include "usart.h"
#include "tim.h"
#include "math.h"
#include"lpf.h"
#include "foc.h"



/*************************** function ********************************/
//  초기화 함수
void Encoder_Init(TIM_HandleTypeDef *htim, encoder_instance* g_enc_instance_mot,uint16_t ppr,float pulse){
   g_enc_instance_mot->pos = 0;
   g_enc_instance_mot->delta_cnt =0;
   g_enc_instance_mot->prev_cnt =0;
   g_enc_instance_mot->encoder_setteing.Encoder_PPR = ppr;
   g_enc_instance_mot->encoder_setteing.Tim_ARR_val = htim->Init.Period;
   g_enc_instance_mot->encoder_setteing.pulse_ang = pulse;



}

void Encoder_Update(TIM_HandleTypeDef *htim,encoder_instance* g_enc_instance_mot){
   static int update_Flag = 0;
   g_enc_instance_mot->current_cnt = __HAL_TIM_GET_COUNTER(htim);
   if(update_Flag == 0)
   {
      g_enc_instance_mot->delta_cnt =0;
       update_Flag = 1;
       g_enc_instance_mot->prev_cnt = __HAL_TIM_GET_COUNTER(htim);
   }
   else
   {
      if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))//역방향 회전
      {

         g_enc_instance_mot->delta_cnt = g_enc_instance_mot->current_cnt - g_enc_instance_mot->prev_cnt;

         if(g_enc_instance_mot->delta_cnt > 32768.0f)
         {
           g_enc_instance_mot->delta_cnt -= 65535.0f;
         }
         g_enc_instance_mot->direction = -1;
      }
      else // 정방향 회전
      {
         g_enc_instance_mot->delta_cnt = g_enc_instance_mot->current_cnt - g_enc_instance_mot->prev_cnt;

         if(g_enc_instance_mot-> delta_cnt < - 32768.0f)
         {
             g_enc_instance_mot->delta_cnt += 65535.0f;
         }
         g_enc_instance_mot->direction = 1;
      }
   }
   g_enc_instance_mot->pos += g_enc_instance_mot->delta_cnt; // 변화량 만큼 누적 위치 증가
   g_enc_instance_mot->prev_cnt = g_enc_instance_mot->current_cnt; // 과거 cnt를 현재 cnt로 업데이트
}

void genius_encoder_algorithm(TIM_HandleTypeDef *htim,encoder_instance* g_enc_instance_mot,float timer_ARR_val){
   float delta_timer_cnt = __HAL_TIM_GET_COUNTER(htim);
   if(delta_timer_cnt > 50000){
      g_enc_instance_mot->delta_cnt = -(timer_ARR_val - delta_timer_cnt + 1);
      }
   else {
      g_enc_instance_mot->delta_cnt = delta_timer_cnt;
   }
   g_enc_instance_mot->pos += g_enc_instance_mot->delta_cnt;
   __HAL_TIM_SET_COUNTER(&htim4, 0);
}


float Encoder_Get_ang(encoder_instance* g_enc_instance_mot){
   float motor_ang_many = (float)(g_enc_instance_mot->pos) * g_enc_instance_mot->encoder_setteing.pulse_ang;
   float motor_ang = fmod(motor_ang_many,360.0f);
    if(g_enc_instance_mot->direction == 1)
    {
       return fabs(motor_ang);
    }
    else if(g_enc_instance_mot->direction == 0){
    return -fabs(motor_ang);
      }
}

float Encoder_Get_ang_vel(encoder_instance* g_enc_instance_mot,LPF_io* lpf_ang_vel,float Ts)// 각속도
{
   static float  prev_pos = 0;
   lpf_ang_vel->input = (g_enc_instance_mot->pos - prev_pos)*g_enc_instance_mot->encoder_setteing.pulse_ang;
   float pulse_ang_vel = (three_time_lpf(lpf_ang_vel,lpf_ang_vel->input)/Ts) *(M_PI /180.0f);
   prev_pos = g_enc_instance_mot->pos;
   return pulse_ang_vel;
}

//void update_state(encoder_instance* g_pandulm_instance_mot, encoder_instance* g_enc_instance_mot,state_struct *state_dot,state_struct *state_hat, state_struct *L_gains,state_struct *Lm_gains,observer_param *observer_param, float u_f,float offset) // 상태관츠기
//{
//   float m_angle = get_mec_ang(g_enc_instance_mot);
//   float p_angle = constrainAngle(get_mec_ang(g_pandulm_instance_mot) + M_PI);
//   float m_angle_off = constrainAngle(m_angle - offset);
//
//   float e_p = p_angle - state_hat->x1;//_hat
//   float e_m = m_angle_off -  state_hat->x3;//_hat
//
//   const float T_s = observer_param->Ts;
//   state_dot->x1 = state_hat->x2 + L_gains->x1 * e_p + Lm_gains->x1 * e_m;
//   state_dot->x2 = (observer_param->ml * observer_param->g/observer_param->J) * state_hat->x1 + L_gains->x2 * e_p + Lm_gains->x2 * e_m -(observer_param->B_NOM * u_f / observer_param->J);
//   state_dot->x3 = state_hat->x4 + L_gains->x3 *e_p + Lm_gains ->x3 * e_m;
//   state_dot->x4 = (-observer_param->ml * observer_param->g/observer_param->J) * state_hat->x1 + L_gains->x4 * e_p + Lm_gains->x4 * e_m + observer_param->B_NOM * u_f * (1.0f / observer_param->J + 1.0f /observer_param->Jr);
//
//   state_hat->x1 += T_s * state_dot ->x1;
//   state_hat->x2 += T_s * state_dot ->x2;
//   state_hat->x3 += T_s * state_dot ->x3;
//   state_hat->x4 += T_s * state_dot ->x4;
//
//}

void disturbance_observer(encoder_instance* g_enc_instance_mot, disturbance_param* g_disturbance,observer_param* g_observer_param, DQ_t* g_volt_dq, LPF_io* lpf_ang_vel, LPF_io*lpf_observer , float observer_input)// 외란 관측기
{
	float act_accel =0;
	float ideal_accel_volt = 0;
	float raw_error = 0;
	float filltered_error = 0;
	const float max = 3.5f;

	g_disturbance->current_vel = Encoder_Get_ang_vel(g_enc_instance_mot, lpf_ang_vel, g_observer_param->Ts);
	g_disturbance->prev_q = observer_input;

	act_accel = -(g_disturbance->current_vel - g_disturbance->prev_vel);

	ideal_accel_volt = g_disturbance->prev_q;

	raw_error = (act_accel*g_disturbance->Kv_NOMINAL) - ideal_accel_volt; // 실제 가속도*변환계수 ->전압 전압 에러 구하기
	filltered_error = six_time_lpf(lpf_observer, raw_error);

	g_disturbance->disturbance_volt = filltered_error; //더해줘야할 지령
	if( -g_disturbance->disturbance_volt > max){
		g_disturbance->disturbance_volt = -max;
	}

	g_disturbance->prev_vel = g_disturbance ->current_vel;

}

