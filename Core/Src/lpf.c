#include "lpf.h"
#include "stdio.h"

//매개변수 : (패스필터 정보 구조체, 입력값)
float three_time_lpf(LPF_io* lpf_omega , float first_input){
	lpf_omega->input = first_input;
	float first = lowPassFilter(lpf_omega);
	lpf_omega->input = first;
	float second = lowPassFilter(lpf_omega);
	lpf_omega->input = second;
	float third = lowPassFilter(lpf_omega);
	return third;
}
float six_time_lpf(LPF_io* lpf_omega , float first_input){
	lpf_omega->input = first_input;
	float first = lowPassFilter(lpf_omega);
	lpf_omega->input = first;
	float second = lowPassFilter(lpf_omega);
	lpf_omega->input = second;
	float third = lowPassFilter(lpf_omega);
	lpf_omega->input = third;
	float fourth = lowPassFilter(lpf_omega);
	lpf_omega->input = fourth;
	float fifth = lowPassFilter(lpf_omega);
	lpf_omega->input = fifth;
	float sixth = lowPassFilter(lpf_omega);
	return sixth;
}
float nine_time_lpf(LPF_io* lpf_omega , float first_input){
	lpf_omega->input = first_input;
	float first = lowPassFilter(lpf_omega);
	lpf_omega->input = first;
	float second = lowPassFilter(lpf_omega);
	lpf_omega->input = second;
	float third = lowPassFilter(lpf_omega);
	lpf_omega->input = third;
	float fourth = lowPassFilter(lpf_omega);
	lpf_omega->input = fourth;
	float fifth = lowPassFilter(lpf_omega);
	lpf_omega->input = fifth;
	float sixth = lowPassFilter(lpf_omega);
	lpf_omega->input = sixth;
	float seventh = lowPassFilter(lpf_omega);
	lpf_omega->input = seventh;
	float eighth = lowPassFilter(lpf_omega);
	lpf_omega->input = eighth;
	float ninth = lowPassFilter(lpf_omega);
	return ninth;
}


