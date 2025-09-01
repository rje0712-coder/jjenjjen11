/*
 * stuct.h
 *
 *  Created on: Aug 27, 2025
 *      Author: PSLAB
 */

#ifndef INC_STUCT_H_
#define INC_STUCT_H_

typedef struct{
	float Encoder_PPR;
	uint32_t Tim_ARR_val;
	float pulse_ang;

}encoder_resol;

typedef struct{
	int pos;
    uint32_t prev_cnt; // 과거 cnt 값
    uint16_t current_cnt; // 현재 cnt
    int16_t delta_cnt;// cnt 변화량
   int8_t direction;
   float off_ang;
  encoder_resol encoder_setteing;

 }encoder_instance;

 typedef struct{
    float x1;
    float x2;
    float x3;
    float x4;
 }state_struct;

 typedef struct {
   float Ts;      // 샘플링주기 [s]
   float Jr;      // rotor inertia
   float J;       // link inertia
   float ml;      // m*l (effective)
   float g;       // 9.81
   float B_NOM;   // input scale (토크 상수 등 흡수)
 }observer_param;

 typedef struct{
	 float prev_vel;
	 float current_vel;
	 float prev_q;
	 float Kv_NOMINAL;
	 float disturbance_volt;
 }disturbance_param;

 extern encoder_instance g_enc_instance_mot;
 extern encoder_instance g_pandulm_instance_mot;

 typedef struct{
 	float Ia; // A상 전류
 	float Ib; // B상 전류
 	float Ic; // C상 전류
 }phase_current;


 typedef struct {
 	float alpha;
 	float beta;
 	float theta;
 }Alpha_Beta_t;


 typedef struct {
 	float d;
     float q;
     float theta;
 }DQ_t;

#endif /* INC_STUCT_H_ */
