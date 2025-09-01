#ifndef INC_ENCODER_TO_GPIO_H_
#define INC_ENCODER_TO_GPIO_H_

typedef struct{
	GPIO_PinState pin_1;
	GPIO_PinState pin_2;
}encoder_pin_state;

typedef struct{
	int8_t dir;
	float pos;
	float delta_cnt;
	float ang_deg;
	float ang_rad;
}motor_state_gpio;

typedef struct{
	float Encoder_PPR;
	uint32_t Tim_ARR_val;
	float pulse_ang;

}encoder_resol_gpio;
void Encoder_update_gpio_ver(encoder_pin_state *pin_state_now,motor_state_gpio *motor_state,encoder_resol_gpio *resol);

extern encoder_resol_gpio g_encoder_resol_gpio;
extern encoder_pin_state g_pin_state_gpio;
extern motor_state_gpio g_motor_now_gpio;

#endif /* INC_ENCODER_TO_GPIO_H_ */
