#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "AD.h"
#include "Serial.h"

#define PWM_FREQUENCY      10000 // PWMƵ��
#define MAX_PWM_VALUE      1000  // PWM���ֵ

#define PWM_PERIOD         3000 // PWM����Ϊ1000��������λ
#define KP                 0.1  // ��������
#define KI                 0.01 // ��������
#define KD                 0.01

#define NUM_READINGS       1
#define INTEGRAL_LUX_LIMIT 20.0f // �����������û����޷�
#define INTEGRAL_LIMIT     0.5f  // �����������û����޷�

float voltage_ref    = 0;   // �趨�������ѹ
float voltage_fb     = 0.0; // ������ʵ�������ѹ
float duty_cycle     = 0.0; // ռ�ձ�
float error          = 0.0; // ���
float integral       = 0.0; // ������
float control_signal = 0.0; // �����ź�
// ����ȫ�ֱ���
float last_error = 0.0; // ��һ�ε����

float derivative  = 0.0; // ΢����
int sample_index  = 0;   // ����������
float trueVoltage = 0.0; // ʵ�������ѹֵ
float VREF;
uint32_t time_counter = 0; // ���ڼ�¼ʱ��ļ�����
uint8_t voltage_state = 0; // 0 ��ʾ��ѹΪ8V��1 ��ʾ��ѹΪ10V
uint16_t ADValue;
float Voltage;
float i;

extern volatile uint8_t DataAvailable;  // ��־λ����ʾ���ݽ������
extern volatile char ReceivedData[100]; // ���ջ�����

#define KP_lux 0.3  // ��������
#define KI_lux 0.0 // ��������
#define KD_lux 0

float Lux_fb         = 0.0; // range (0 - 100)
float Target_Lux     = 85.0;
float error_lux      = 0.0;
float integral_lux   = 0.0; // ������
float last_error_lux = 0.0; // ��һ�ε����
float derivative_lux = 0.0; // ΢����

float control_signal_lux = 0.0; // �����ź�

float Get_Lux(void)
{
    float total_voltage = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        total_voltage += ((float)AD_GetValue(ADC_Channel_9) * 3.3f) / 4095.0f; // �ۼӲ���ֵ
    }
    float average_voltage = total_voltage / NUM_READINGS;
    Lux_fb                = (3.3f - average_voltage) / 3.3f * 100; // ��ȷ������հٷֱ�
    return Lux_fb;                                                 // ���ط�ΧΪ (0 - 100) �Ĺ���ֵ
}

void control_buck(void)
{
    // ��ȡADC��ֵ

    voltage_fb = (float)AD_GetValue(ADC_Channel_4) * 3.3 / 4096 - 0.28f;
    // ����ʵ�ʵ�ѹ (�����ⲿ��·����)
    trueVoltage = voltage_fb * 1050 * 20000 / 2500 / 1000;
    // �������
    error = voltage_ref * 2500 * 1000 / 1050 / 20000 - voltage_fb;
    // ���ּ���
    integral += error;

    //	if (integral >  INTEGRAL_LIMIT){
    //		integral =  INTEGRAL_LIMIT;
    //	} else if (integral <  -INTEGRAL_LIMIT){
    //		integral =  -INTEGRAL_LIMIT;
    //	}

    // ΢�ּ���
    derivative = error - last_error;
    // PID�������
    control_signal = KP * error + KI * integral + KD * derivative;
    if (control_signal > 0.8) {
        control_signal = 0.8;
    } else if (control_signal < 0.2) {
        control_signal = 0.2;
    }
    // ����ռ�ձ�
    duty_cycle = control_signal * PWM_PERIOD;
    i          = (int)duty_cycle;

    // TIM2->CCR1 = (uint16_t)control_signal;
    // TIM2->CCR1 = (uint16_t)duty_cycle;
    PWM_SetCompare1(i);
    // Serial_Printf("%d,%.2f\r\n", sample_index, trueVoltage);  // ����CSV��ʽ�����ݰ�
    sample_index++;
}

float control_light(void)
{
    Lux_fb    = Get_Lux();
    error_lux = Target_Lux - Lux_fb;
    integral_lux += error_lux;

    if (integral_lux > INTEGRAL_LUX_LIMIT) {
        integral_lux = INTEGRAL_LUX_LIMIT;
    } else if (integral_lux < -INTEGRAL_LUX_LIMIT) {
        integral_lux = -INTEGRAL_LUX_LIMIT;
    }

    derivative_lux     = error_lux - last_error_lux;
    control_signal_lux = KP_lux * error_lux + KI_lux * integral_lux + KD_lux * derivative_lux;

    if (control_signal_lux > 10) {
        control_signal_lux = 10;
    } else if (control_signal_lux < 0) {
        control_signal_lux = 0;
    }

    return control_signal_lux;
}

void update_voltage_reference(void)
{
    // �ۼ�ʱ�䣬ÿ���ӳٺ���ô˺���
    time_counter += 10; // ������ѭ����ʱΪ10ms��ÿ���ۼ�10ms

    // ÿ10�� (10000ms) �л�һ�ε�ѹ�ο�ֵ
    if (time_counter >= 3000) {
        time_counter = 0; // ���ü�����

        // �л��ο���ѹ
        if (voltage_state == 0) {
            voltage_ref   = 10.0; // �л���10V
            voltage_state = 1;    // ����״̬
        } else {
            voltage_ref   = 6.0; // �л���8V
            voltage_state = 0;   // ����״̬
        }
    }
}

typedef enum {
    OFF        = 0,
    ON         = 1,
    BRIGHTNESS = 2,
    LUX        = 3
} Command;

Command command = OFF;

void get_command(void)
{
    if (ReceivedData[1] == '1' && ReceivedData[2] == 's') {
        command = ON;
    } else if (ReceivedData[1] == '3' && ReceivedData[2] == 's') {
        command = BRIGHTNESS;
    } else if (ReceivedData[1] == '4' && ReceivedData[2] == 's') {
        command = LUX;
    } else {
        command = OFF;
    }
}

float brightness  = 8.f; // ���ȱ���
uint8_t direction = 1;   // ����1 ��ʾ������0 ��ʾ�䰵

void breath_light(void)
{
    // ��������
    if (direction) {
        brightness += 0.008f;
        if (brightness >= 10.f) {
            direction = 0; // �ﵽ������ȣ���ʼ�䰵
        }
    } else {
        brightness -= 0.008f;
        if (brightness <= 6.5f) {
            direction = 1; // �ﵽ��С���ȣ���ʼ����
        }
    }

    voltage_ref = brightness;
}

int main(void)
{
    OLED_Init();
    PWM_Init();
    AD_Init();
    Delay_ms(10);
    NVIC_Configuration(); // ?? NVIC ??
    Serial_Init();        // ��ʼ������

    while (1) {

        get_command();

        switch (command) {
            case ON:
                voltage_ref = 10.f;
                break;
            case BRIGHTNESS:
                breath_light();
                break;
            case LUX:
                voltage_ref = control_light();
                break;
            case OFF:
                voltage_ref = 3.f;
                break;
            default:
                voltage_ref = 3.f;
                break;
        }

        // update_voltage_reference();
        control_buck();

        Delay_ms(10);
    }
}
