#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
	//开启RCC时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //开启ADC时钟，ADC都是APB2上的外设
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //开启 GPIOA 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //开启 GPIOB 时钟
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div2); //ADC的分频，这里是6分频12MHz
	
	//初始化 PA4、PB0、PB1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入模式，ADC专属模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //配置 PA4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //配置 PB0 和 PB1
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//配置ADC通道
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //独立工作模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //外部触发源选择，这个地方是内部软件触发
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);
	
	//校准
	ADC_ResetCalibration(ADC1); //复位校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
}

uint16_t AD_GetValue(uint8_t channel)
{
	//根据指定通道选择ADC通道
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
