#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "AD.h"
#include "Serial.h"

#define PWM_FREQUENCY      10000 // PWM频率
#define MAX_PWM_VALUE      1000  // PWM最大值

#define PWM_PERIOD         3000 // PWM周期为1000个计数单位
#define KP                 0.1  // 比例增益
#define KI                 0.01 // 积分增益
#define KD                 0.01

#define NUM_READINGS       1
#define INTEGRAL_LUX_LIMIT 20.0f // 根据需求设置积分限幅
#define INTEGRAL_LIMIT     0.5f  // 根据需求设置积分限幅

float voltage_ref    = 0;   // 设定的输出电压
float voltage_fb     = 0.0; // 反馈的实际输出电压
float duty_cycle     = 0.0; // 占空比
float error          = 0.0; // 误差
float integral       = 0.0; // 积分项
float control_signal = 0.0; // 控制信号
// 定义全局变量
float last_error = 0.0; // 上一次的误差

float derivative  = 0.0; // 微分项
int sample_index  = 0;   // 样本计数器
float trueVoltage = 0.0; // 实际输出电压值
float VREF;
uint32_t time_counter = 0; // 用于记录时间的计数器
uint8_t voltage_state = 0; // 0 表示电压为8V，1 表示电压为10V
uint16_t ADValue;
float Voltage;
float i;

extern volatile uint8_t DataAvailable;  // 标志位，表示数据接收完成
extern volatile char ReceivedData[100]; // 接收缓冲区

#define KP_lux 0.3  // 比例增益
#define KI_lux 0.0 // 积分增益
#define KD_lux 0

float Lux_fb         = 0.0; // range (0 - 100)
float Target_Lux     = 85.0;
float error_lux      = 0.0;
float integral_lux   = 0.0; // 积分项
float last_error_lux = 0.0; // 上一次的误差
float derivative_lux = 0.0; // 微分项

float control_signal_lux = 0.0; // 控制信号

float Get_Lux(void)
{
    float total_voltage = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        total_voltage += ((float)AD_GetValue(ADC_Channel_9) * 3.3f) / 4095.0f; // 累加采样值
    }
    float average_voltage = total_voltage / NUM_READINGS;
    Lux_fb                = (3.3f - average_voltage) / 3.3f * 100; // 正确计算光照百分比
    return Lux_fb;                                                 // 返回范围为 (0 - 100) 的光照值
}

void control_buck(void)
{
    // 读取ADC的值

    voltage_fb = (float)AD_GetValue(ADC_Channel_4) * 3.3 / 4096 - 0.28f;
    // 计算实际电压 (根据外部电路参数)
    trueVoltage = voltage_fb * 1050 * 20000 / 2500 / 1000;
    // 计算误差
    error = voltage_ref * 2500 * 1000 / 1050 / 20000 - voltage_fb;
    // 积分计算
    integral += error;

    //	if (integral >  INTEGRAL_LIMIT){
    //		integral =  INTEGRAL_LIMIT;
    //	} else if (integral <  -INTEGRAL_LIMIT){
    //		integral =  -INTEGRAL_LIMIT;
    //	}

    // 微分计算
    derivative = error - last_error;
    // PID控制输出
    control_signal = KP * error + KI * integral + KD * derivative;
    if (control_signal > 0.8) {
        control_signal = 0.8;
    } else if (control_signal < 0.2) {
        control_signal = 0.2;
    }
    // 更新占空比
    duty_cycle = control_signal * PWM_PERIOD;
    i          = (int)duty_cycle;

    // TIM2->CCR1 = (uint16_t)control_signal;
    // TIM2->CCR1 = (uint16_t)duty_cycle;
    PWM_SetCompare1(i);
    // Serial_Printf("%d,%.2f\r\n", sample_index, trueVoltage);  // 发送CSV格式的数据包
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
    // 累计时间，每次延迟后调用此函数
    time_counter += 10; // 假设主循环延时为10ms，每次累加10ms

    // 每10秒 (10000ms) 切换一次电压参考值
    if (time_counter >= 3000) {
        time_counter = 0; // 重置计数器

        // 切换参考电压
        if (voltage_state == 0) {
            voltage_ref   = 10.0; // 切换到10V
            voltage_state = 1;    // 更新状态
        } else {
            voltage_ref   = 6.0; // 切换到8V
            voltage_state = 0;   // 更新状态
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

float brightness  = 8.f; // 亮度变量
uint8_t direction = 1;   // 方向：1 表示变亮，0 表示变暗

void breath_light(void)
{
    // 调节亮度
    if (direction) {
        brightness += 0.008f;
        if (brightness >= 10.f) {
            direction = 0; // 达到最大亮度，开始变暗
        }
    } else {
        brightness -= 0.008f;
        if (brightness <= 6.5f) {
            direction = 1; // 达到最小亮度，开始变亮
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
    Serial_Init();        // 初始化串口

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
