
#ifndef INC_LPF_H_
#define INC_LPF_H_

#include "stdint.h" // 정밀하게 비트 크기가 정의된 정수형 타입 제공
#include "stdbool.h" // bool 타입과 true/false 매크로 제공
#include "main.h"

typedef struct{
	float input;
	float prevlpf;
	float t_s;
	float cut_fre;
}LPF_io;

// 저역 통과 필터 LPF
static inline float lowPassFilter(LPF_io* lpf_omega) // lowPassFilter 함수 정의, 사용시 값 할당 해주기
{

	float result = (lpf_omega->prevlpf + (lpf_omega->t_s * lpf_omega->cut_fre * lpf_omega->input)) / (1 + lpf_omega->t_s * lpf_omega->cut_fre);
    lpf_omega->prevlpf = result;
	return result;

}

float three_time_lpf(LPF_io* lpf_omega , float first_input);
float six_time_lpf(LPF_io* lpf_omega , float first_input);
float nine_time_lpf(LPF_io* lpf_omega , float first_input);
// 이정도면 그만해라
#endif /* INC_LPF_H_ */
