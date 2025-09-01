#ifndef INC_SPWM_H_
#define INC_SPWM_H_
typedef struct{
	float va;
	float vb;
	float vc;
	float input_vdc;
}three_phase_volt;

typedef struct{
int duty_a;
int duty_b;
int duty_c;

}three_phase_duty;

void generate_spwm_val(three_phase_volt *input,three_phase_duty *output);

void Insert_duty(three_phase_duty *start_duty);

void change_volt_val(three_phase_volt *input,float a,float b,float c,float vdc);
#endif /* INC_SPWM_H_ */
