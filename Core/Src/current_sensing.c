/*********************** include ***************************/
#include"current_sensing.h"
#include"adc.h"
#include"tim.h"
#include"encoder.h"
#include"math.h"

/**************************** function *************************/
//volatile uint32_t ADC_val[2];

////void Sensing_current(phase_current*g_current_now ){
//	 g_current_now ->Ia = (ADC_val[0]-offset_adc_val)*ADC_TO_CURRENT;
//	 g_current_now ->Ib = (ADC_val[1]-offset_adc_val)*ADC_TO_CURRENT;
//	 g_current_now ->Ic = - (g_current_now ->Ia +  g_current_now ->Ib) ;
// }

float get_elec_ang(encoder_instance *g_enc_instance_mot , uint8_t POLE_PAIRS, float offset_ang){
	float two_pi = 2.0f * M_PI;

	float counts_per_rev = g_enc_instance_mot->encoder_setteing.Encoder_PPR ; //X4 디코딩
	float mechanical_angle = ((g_enc_instance_mot->pos- offset_ang) / counts_per_rev) * two_pi;
	float electrical_angle =(mechanical_angle * POLE_PAIRS);


	return electrical_angle;
}
float get_mec_ang(encoder_instance *g_enc_instance_mot, float offset_ang ){
	float two_pi = 2.0f * M_PI;
	float counts_per_rev = g_enc_instance_mot->encoder_setteing.Encoder_PPR ; //X4 디코딩
	float mechanical_angle = ((g_enc_instance_mot->pos - offset_ang) / counts_per_rev) * two_pi;


	return mechanical_angle ;
}
