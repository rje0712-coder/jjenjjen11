
#ifndef INC_FOC_H_
#define INC_FOC_H_


#include "foc.h"
#include "current_sensing.h"
#include "SPWM.h"
#include"stuct.h"





void clarke_trans(phase_current *in_i_abc, Alpha_Beta_t *out_i_Alpha_Beta);
void park_trans(Alpha_Beta_t *in_i_Alpha_Beta,DQ_t *out_i_dq );
void Inverse_park_trans(DQ_t *in_v_dq, Alpha_Beta_t *out_v_Alpha_Beta);
void Inverse_clarke_trans(Alpha_Beta_t *in_v_Alpha_Beta,three_phase_volt *out_thr_phase);
void Inverse_three_phase(DQ_t *in_v_dq, three_phase_volt *out_thr_phase,float vdc);

extern float offset_theta;

#endif /* INC_FOC_H_ */
