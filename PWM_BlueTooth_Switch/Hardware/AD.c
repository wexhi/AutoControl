#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
	//����RCCʱ�� 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //����ADCʱ�ӣ�ADC����APB2�ϵ�����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //���� GPIOA ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //���� GPIOB ʱ��
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div2); //ADC�ķ�Ƶ��������6��Ƶ12MHz
	
	//��ʼ�� PA4��PB0��PB1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ������ģʽ��ADCר��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //���� PA4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //���� PB0 �� PB1
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//����ADCͨ��
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //��������ģʽ
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //�Ҷ���
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //�ⲿ����Դѡ������ط����ڲ��������
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);
	
	//У׼
	ADC_ResetCalibration(ADC1); //��λУ׼
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
}

uint16_t AD_GetValue(uint8_t channel)
{
	//����ָ��ͨ��ѡ��ADCͨ��
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
