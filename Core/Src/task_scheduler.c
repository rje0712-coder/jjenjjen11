#include"tim.h"
#include"task_scheduler.h"
#include"gpio.h"
#include"encoder.h"
#include"PI.h"
#include"lpf.h"
#include"usart.h"
#include"SPWM.h"
#include"foc.h"
#include"current_sensing.h"
#include "math.h"
#include"usart.h"
#include"string.h"
#include"stuct.h"

extern encoder_instance g_enc_instance_mot ;
//순서대로 cnt총누적값, cnt이전값, cnt현재값, cnt 변화량, 방향, 센서 사양{ppr ,ARR ,펄스각}
// encoder_instance 자료형 {int, int, int, int, uint8, { uint8 , uint32, f}

DQ_t g_volt_dq = {0.0f};
//순서대로 d, q 자료형 f,f,theta(f)

three_phase_volt g_thr_phase_volt = {0.0f,0.0f,0.0f,14.0f};
//순서대로 삼상 va,vb,vc,넣어주는 vcc 자료형 f,f,f,f
three_phase_duty g_thr_phase_duty = {0.0f,0.0f,0.0f};
//순서대로 삼상 듀티 duty 자료형 f,f,f
three_phase_volt g_start_volt = {0.0f,0.0f,0.0f,14.0f};
three_phase_duty g_start_duty = {0.0f,0.0f,0.0f};

LPF_io g_LPF_ang = {0.0f, 0.0f, 0.001f,55.0f};
LPF_io g_LPF_id = {0.0f, 0.0f, 0.001f, 60.0f};
LPF_io g_LPF_iq = {0.0f, 0.0f, 0.001f, 60.0f};
LPF_io g_LPF_ref = {0.0f, 0.0f, 0.001f, 80.0f};
LPF_io g_LPF_ang_vel = {0.0f, 0.0f, 0.001f, 50.0f};
LPF_io g_LPF_observer = {0.0f, 0.0f, 0.001f, 50.0f};
// 순서대로 입력(연산시 사용), 이전값(연산시 사용), 시간값, 차단 주파수
//LPF_io 자료형 f,f,f,f

pi_params g_pi_param_current ={0.001f, 14.0f, -14.0f,40.0f};
//순서대로 dt ,max 리미트,min 리미트 자료형 : f,f,uint8
pi_motor g_omega_pi_val = {0,0.0f,0.0f,0.0f};
//순서대로 방향(아직 사용 x), 현재 밸류, 타겟 , 인테그랄(연산시 사용)
// pi motor 자료형 uint8,f,f,f

gain_val g_omega_pi_gain = {0.0f, 0.0f,0.0f};
//순서대로 ki,kp 자료형 f,f
disturbance_param g_disturbance ={0,0,0,0,100.0f,0};
observer_param g_observer_param ={0.001f,12.48e-4,0.0265f,0.1350,9.81f,2350.0f};
static uint16_t cnt =0;
float T_s=0;
float filt_Ia =0;
float filt_Ib =0;
float filt_Ic =0;
/*************************************TASK*****************************************/
uint32_t Cnt =0;
void AppTask1ms(void){
/***********************************vari*******************************************/


	Cnt++;
   T_s=Cnt*0.001f;
   float raw_graph1 = 0;
   float filltered_graph1 =0;
   static float off_ang =0;
   static int flag =0;
   float final_volt = 0;
   static float observer_input =0;

   Encoder_Update(&htim4,&g_enc_instance_mot);

   if(T_s < 5.0f){
       change_volt_val(&g_start_volt, 8.0f, 0.0f, 0.0f, 14.0f);
       generate_spwm_val(&g_start_volt, &g_start_duty);
      Insert_duty(&g_start_duty);
   }// 정렬

   else if(5.0f < T_s && T_s<10.0f){
      raw_graph1 = (4.0f*M_PI/5.0f)*(T_s - 5.0f);
      filltered_graph1 = six_time_lpf(&g_LPF_ref, raw_graph1);
      disturbance_observer(&g_enc_instance_mot, &g_disturbance, &g_observer_param, &g_volt_dq, &g_LPF_ang_vel, &g_LPF_observer, observer_input);
      if(flag == 0){
         off_ang = g_enc_instance_mot.pos;
         flag =1;
      }
      else if(flag ==1){

    	  change_gain_to_easy(&g_omega_pi_gain,0.6f,0.4f,0.5f);


         g_omega_pi_val.target_val= filltered_graph1;
         g_omega_pi_val.current_val =  Encoder_Get_ang_vel(&g_enc_instance_mot,&g_LPF_ang_vel,0.001f);

         final_volt = PID_control(&g_omega_pi_val, &g_omega_pi_gain, &g_pi_param_current)- g_disturbance.disturbance_volt;
         if(final_volt > g_pi_param_current.max_limit){
            final_volt = g_pi_param_current.max_limit;
         }



         g_volt_dq.d = 0;
         g_volt_dq.q = final_volt;
         observer_input = final_volt;
         g_volt_dq.theta = get_elec_ang(&g_enc_instance_mot, 11, off_ang);


         Inverse_three_phase(&g_volt_dq,&g_thr_phase_volt,14.0f); // vdc변경
         generate_spwm_val(&g_thr_phase_volt, &g_thr_phase_duty);
         Insert_duty(&g_thr_phase_duty);

      }

   }
   else if(T_s>10.0f){
      raw_graph1 = 4.0f*M_PI;
      change_gain_to_easy(&g_omega_pi_gain,0.6f,0.4f,0.05f);
      disturbance_observer(&g_enc_instance_mot, &g_disturbance, &g_observer_param, &g_volt_dq, &g_LPF_ang_vel, &g_LPF_observer, observer_input);

         filltered_graph1 = six_time_lpf(&g_LPF_ref, raw_graph1);
         g_omega_pi_val.target_val= filltered_graph1;
         g_omega_pi_val.current_val = Encoder_Get_ang_vel(&g_enc_instance_mot,&g_LPF_ang_vel,0.001f);

        final_volt = PID_control(&g_omega_pi_val, &g_omega_pi_gain, &g_pi_param_current)- g_disturbance.disturbance_volt;
        if(final_volt > g_pi_param_current.max_limit){
        	final_volt = g_pi_param_current.max_limit;
        }



         g_volt_dq.d = 0;
         g_volt_dq.q = final_volt;
         observer_input = PID_control(&g_omega_pi_val, &g_omega_pi_gain, &g_pi_param_current);
         g_volt_dq.theta = get_elec_ang(&g_enc_instance_mot, 11, off_ang);


        Inverse_three_phase(&g_volt_dq,&g_thr_phase_volt,14.0); // vdc변경
        generate_spwm_val(&g_thr_phase_volt, &g_thr_phase_duty);
        Insert_duty(&g_thr_phase_duty);

   } // 속도 유지
   volatile float a = final_volt;
   volatile float b =- g_disturbance.current_vel ;
   volatile float c = -g_disturbance.disturbance_volt;

}




void AppTask10ms(void){

}
void AppTask100ms(void){

 }










/*************************************FLAG scheduler*******************************/
SchedulingFlag stSchedulinginfo;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
   if(htim ->Instance==TIM7){
      cnt++;
      if(cnt %1 ==0){
         stSchedulinginfo.u8nuScheduling1msFlag = 1;

      if(cnt %10 ==0){
         stSchedulinginfo.u8nuScheduling10msFlag = 1;

      if(cnt %100 ==0){
         stSchedulinginfo.u8nuScheduling100msFlag = 1;
         if(cnt>10000) cnt=0;
         }
      }
   }
}
}

   void Appscheduling(void){
       // 1ms 태스크 실행 조건 (독립적으로 검사)
       if(stSchedulinginfo.u8nuScheduling1msFlag == 1){
           stSchedulinginfo.u8nuScheduling1msFlag = 0; // 플래그 클리어
           AppTask1ms(); // 1ms 태스크 실행
       }

       // 10ms 태스크 실행 조건 (독립적으로 검사)
       // 1ms 태스크와는 별개로 10ms 플래그만 확인합니다.
       if(stSchedulinginfo.u8nuScheduling10msFlag == 1){
           stSchedulinginfo.u8nuScheduling10msFlag = 0; // 플래그 클리어
           AppTask10ms(); // 10ms 태스크 실행
       }

       // 100ms 태스크 실행 조건 (독립적으로 검사)
       // 10ms 태스크와는 별개로 100ms 플래그만 확인합니다.
       if(stSchedulinginfo.u8nuScheduling100msFlag == 1){
           stSchedulinginfo.u8nuScheduling100msFlag = 0; // 플래그 클리어
           AppTask100ms(); // 100ms 태스크 실행
       }
   }
