
#ifndef INC_CURRENT_SENSING_H_
#define INC_CURRENT_SENSING_H_
/*************************define*****************/
#define ADC_TO_CURRENT 0.00161f // ADC_val*(3.3/4095)*5 = ADC_val*0.004293
#define offset_adc_val 2048.0f // 0A일떄 2048 출력 ->2048을 기준으로 전류 방향과 크기르 계산
/*************************Include*****************/

#include"main.h"
#include"encoder.h"
#include"stuct.h"
/*************************typedef*****************/



/**************************** function *************************/
void Sensing_current(phase_current*g_current_now); // 전류 센싱
float get_elec_ang(encoder_instance *g_enc_instance_mot , uint8_t POLE_PAIRS, float offset_ang);
float get_mec_ang(encoder_instance *g_enc_instance_mot, float offset_ang );

//extern volatile uint32_t ADC_val[2];

#endif /* INC_CURRENT_SENSING_H_ */
