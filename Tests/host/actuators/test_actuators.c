#include "main.h"
#include "drivers/Servo.h"
#include "drivers/Switches.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

GPIO_TypeDef gpio;
TIM_TypeDef timer;
TIM_HandleTypeDef htim4 = {&timer, HAL_TIM_STATE_READY};
static uint32_t pclk = 16000000, divider = 1;
static unsigned running, starts;
static int fail_channel = -1;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int high)
{
    if (high) port->ODR |= pin; else port->ODR &= ~pin;
}
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency)
{
    clocks->APB1CLKDivider = divider; *latency = 0;
}
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *tim, uint32_t channel)
{
    assert(tim == &htim4); ++starts;
    if ((int)channel == fail_channel) return 1;
    running |= 1U << channel; return HAL_OK;
}
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *tim, uint32_t channel)
{
    assert(tim == &htim4); running &= ~(1U << channel); return HAL_OK;
}
int main(void)
{
    ServoData servos;
    SwitchState switches;
    assert(!Servo_GetData(&servos));
    Servo_Enable(); assert(running == 0);
    gpio.ODR = 0xFFFF;
    assert(Switches_Init());
    assert(gpio.ODR == (0xFFFFU & ~0x1F00U)); /* Preserve unrelated outputs. */
    Switches_LED(true); assert(gpio.ODR & LED_Switch_Pin);
    Switches_Buzzer(true); Switches_DVR(true); Switches_HDRM(true); Switch_Capture(true);
    Switches_Update(&switches);
    assert(switches.led && switches.buzzer && switches.dvr && switches.hdrm && switches.capture);
    Switches_LED(false); Switches_Buzzer(false); Switches_DVR(false);
    Switches_HDRM(false); Switch_Capture(false);
    Switches_Update(&switches);
    assert(!switches.led && !switches.buzzer && !switches.dvr && !switches.hdrm && !switches.capture);
    Switches_Update(NULL);
    timer.PSC = 15; timer.ARR = 19999;
    timer.CCMR1 = TIM_OCMODE_PWM1 | (TIM_OCMODE_PWM1 << 8);
    assert(Servo_Init());
    assert(Servo_GetData(&servos) && !servos.enabled && servos.servo1Angle == 90);
    assert(timer.CCR[0] == 1450 && timer.CCR[1] == 1450 && !running);
    Servo_SetAngles(-20, 200);
    assert(timer.CCR[0] == 500 && timer.CCR[1] == 2400);
    Servo_Enable(); assert(running == 3);
    unsigned previous_starts = starts;
    Servo_Enable(); assert(starts == previous_starts);
    timer.CNT = 1234; timer.EGR = 0;
    Servo_SetAngles(45, 135);
    assert(timer.CCR[0] == 975 && timer.CCR[1] == 1925);
    assert(timer.CNT == 1234 && timer.EGR == 0); /* No mid-frame restart. */
    Servo_SetAngles(NAN, 0); Servo2_SetAngle(INFINITY);
    assert(timer.CCR[0] == 975 && timer.CCR[1] == 1925);
    Servo1_SetAngle(90); assert(timer.CCR[0] == 1450 && timer.CCR[1] == 1925);
    Servo_Disable(); assert(!running);
    fail_channel = TIM_CHANNEL_2;
    Servo_Enable(); assert(!running);
    assert(Servo_GetData(&servos) && !servos.enabled);
    fail_channel = -1; Servo_Enable(); assert(running == 3);
    timer.PSC = 0; assert(!Servo_Init() && !running);
    assert(!Servo_GetData(&servos));
    pclk = 42000000; divider = 4; timer.PSC = 83;
    assert(Servo_Init()); /* APB timer clock doubling: 84 MHz. */
    assert(!Servo_GetData(NULL));
    timer.ARR = 10000; assert(!Servo_Init());
    puts("Actuator tests passed: output isolation, PWM mapping, bounds, invalid angles, startup/disable, start failure, clock validation");
    return 0;
}
