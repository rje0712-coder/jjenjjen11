#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "main.h"

typedef struct{
	uint8_t u8nuScheduling1msFlag ;
	uint8_t u8nuScheduling10msFlag ;
	uint8_t u8nuScheduling100msFlag ;
}SchedulingFlag;

extern SchedulingFlag stSchedulinginfo;

void AppTask1ms(void);
void AppTask10ms(void);
void AppTask100ms(void);

void Appscheduling(void);

#endif
