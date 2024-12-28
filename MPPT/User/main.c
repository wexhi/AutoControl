#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "AD.h"
#include "Serial.h"

#define PWM_FREQUENCY 10000    // PWM频率
#define MAX_PWM_VALUE 1000     // PWM最大值


#define PWM_PERIOD 3000  // PWM周期为1000个计数单位

#define STEP_SIZE 10

float input_voltage = 0;
float input_current = 0;
float current_power = 0;
float max_power = 0;
float last_power = 0;
float dutyCycle = 0;

void Get_Input_voltage(void)
{
	// 15.17
	float tmp = 0;
	tmp = (float)AD_GetValue(ADC_Channel_4)*3.3/4096 - 0.1f;
	input_voltage = tmp*1050*20000/2500/1000; // 16
}

void Get_Input_current(void)
{
	input_current = (float)AD_GetValue(ADC_Channel_6)*3.3/4096 - 0.4f;
	if (input_current <= 0 )
	{
		input_current = 0;
	}
}

void Cal_Power(void)
{
	current_power = input_voltage * input_current;
	if (current_power > last_power){
		dutyCycle += STEP_SIZE;       // 增大占空比
	}else {
		dutyCycle -= STEP_SIZE;       // 减小占空比
	}
	
	if (current_power > max_power)
	{
		max_power = current_power;
	}
	
	if (dutyCycle >= PWM_PERIOD * 0.8)
	{
		dutyCycle = PWM_PERIOD * 0.8;
	}
	
	if (dutyCycle <= PWM_PERIOD * 0.2)
	{
		dutyCycle = PWM_PERIOD * 0.2;
	}
	last_power = current_power;
	
	PWM_SetCompare1((uint16_t)(dutyCycle));
}




int main(void)
{
	OLED_Init();
	PWM_Init();
	AD_Init();
	Delay_ms(10);
    NVIC_Configuration();  // ?? NVIC ??
	Serial_Init();  // 初始化串口

	while (1)
	{
		Get_Input_voltage();
		Get_Input_current();
		Cal_Power();
		Delay_ms(10);
	}
}
