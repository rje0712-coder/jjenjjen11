#include"tim.h"
#include"task_scheduler.h"
#include"gpio.h"
#include"PI.h"
#include"lpf.h"
#include"usart.h"
#include"SPWM.h"
#include"foc.h"
#include"current_sensing.h"
#include"adc.h"
#include "math.h"
#include "encoder_to_gpio.h"
#include "math.h"

void Encoder_update_gpio_ver(encoder_pin_state *pin_state_now,motor_state_gpio *motor_state,encoder_resol_gpio *resol){
	static GPIO_PinState prev_pin_1 = GPIO_PIN_RESET;
	static GPIO_PinState prev_pin_2 = GPIO_PIN_RESET;
	static float prev_pos = 0;

	pin_state_now->pin_1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
    pin_state_now->pin_2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
    // 엔코더 chA가 상승 엣지일 때, 회전 방향 측정
    if (pin_state_now->pin_1 != prev_pin_1)
     {
         // 1번 핀과 2번 핀의 상태를 비교하여 방향 결정
         motor_state->dir = (pin_state_now->pin_1 != pin_state_now->pin_2) ? 1 : -1;
         motor_state->pos += motor_state->dir;
     }
     // 2번 핀(CHB)의 엣지 발생 시 방향 판단 및 위치 업데이트
     else if (pin_state_now->pin_2 != prev_pin_2)
     {
         // 1번 핀과 2번 핀의 상태를 비교하여 방향 결정
         motor_state->dir = (pin_state_now->pin_1 == pin_state_now->pin_2) ? 1 : -1;
         motor_state->pos += motor_state->dir;
     }

     // delta_cnt 계산은 위치 업데이트 후 수행
     motor_state->delta_cnt = motor_state->pos - prev_pos;

     // 다음 함수 호출을 위해 현재 핀 상태와 위치를 저장
     prev_pin_1 = pin_state_now->pin_1;
     prev_pin_2 = pin_state_now->pin_2;
     prev_pos = motor_state->pos;

     // 틱 수를 이용해 각도 계산
     motor_state->ang_deg = fmodf((motor_state->pos * resol->pulse_ang), 360.0f);
     if (motor_state->ang_deg < 0)
     {
         motor_state->ang_deg += 360.0f; // 음수 각도 보정
     }
     motor_state->ang_rad = motor_state->ang_deg * (M_PI / 180.0f);
 }

