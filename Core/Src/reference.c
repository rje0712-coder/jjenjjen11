#include "stdio.h"
#include "reference.h"

float generate_reference (ref_compare *ref){
	float out_ref;
	ref->cnt_ref += ref->t_ref;
	out_ref = ref->cnt_ref * ref->gain_ref;
	if(out_ref >= ref->taget_ref) out_ref = ref->taget_ref;
	else if(out_ref <= 0) out_ref = 0;
	return out_ref;
}
