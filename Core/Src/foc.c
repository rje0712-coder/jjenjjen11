
#include "foc.h"
#include "math.h"
#include "current_sensing.h"
#include "SPWM.h"
#include "encoder.h"
#include "tim.h"
#include "encoder.h"




void clarke_trans(phase_current *in_i_abc, Alpha_Beta_t *out_i_Alpha_Beta)
{
	out_i_Alpha_Beta->alpha=in_i_abc->Ia;
	out_i_Alpha_Beta->beta = (in_i_abc->Ia + 2.0f * in_i_abc->Ib) / sqrtf(3.0f);
}

void park_trans(Alpha_Beta_t *in_i_Alpha_Beta,DQ_t *out_i_dq ) //theta= 각도
{

	float cos_theta = cosf(in_i_Alpha_Beta->theta);
	float sin_theta = sinf(in_i_Alpha_Beta->theta);

	out_i_dq->d= in_i_Alpha_Beta->alpha * cos_theta + in_i_Alpha_Beta->beta * sin_theta;
	out_i_dq->q = -in_i_Alpha_Beta->alpha * sin_theta + in_i_Alpha_Beta->beta * cos_theta;
}

void Inverse_park_trans(DQ_t *in_v_dq, Alpha_Beta_t *out_v_Alpha_Beta){

	 float cos_theta = cosf(out_v_Alpha_Beta->theta);
	 float sin_theta = sinf(out_v_Alpha_Beta->theta);

	 out_v_Alpha_Beta->alpha =in_v_dq->d * cos_theta - in_v_dq->q * sin_theta;
	 out_v_Alpha_Beta->beta= in_v_dq->d * sin_theta + in_v_dq->q * cos_theta;
 }

void Inverse_clarke_trans(Alpha_Beta_t *in_v_Alpha_Beta,three_phase_volt *out_thr_phase){



	out_thr_phase->va =  in_v_Alpha_Beta->alpha;
	out_thr_phase->vb =  (- in_v_Alpha_Beta->alpha + sqrtf(3.0f) * in_v_Alpha_Beta->beta) / 2.0f;
	out_thr_phase->vc =  (- in_v_Alpha_Beta->alpha - sqrtf(3.0f) * in_v_Alpha_Beta->beta) / 2.0f;


 }// 역변환

void Inverse_three_phase(DQ_t *in_v_dq, three_phase_volt *out_thr_phase,float vdc){
	const float TWO_PI_THIRD = 2.0f * M_PI / 3.0f;
	out_thr_phase->va = in_v_dq->d * cosf(in_v_dq->theta) - in_v_dq->q * sinf(in_v_dq->theta);
	out_thr_phase->vb = in_v_dq->d * cosf(in_v_dq->theta - TWO_PI_THIRD) - in_v_dq->q * sinf(in_v_dq->theta - TWO_PI_THIRD);
	out_thr_phase->vc = in_v_dq->d * cosf(in_v_dq->theta + TWO_PI_THIRD) - in_v_dq->q * sinf(in_v_dq->theta + TWO_PI_THIRD);
	out_thr_phase->input_vdc =vdc;
}





//   float offset_theta;
//   offset_theta = Encoder_Get_ang(g_enc_instance_mot);

