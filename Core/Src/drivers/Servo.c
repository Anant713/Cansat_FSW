#include "drivers/Servo.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

extern TIM_HandleTypeDef htim4;
static bool initialized;
static ServoData data;

static float clamp_angle(float angle)
{
    if (angle < 0.0f) return 0.0f;
    if (angle > 180.0f) return 180.0f;
    return angle;
}

static uint32_t pulse_us(float angle)
{
    return (uint32_t)(500.0f + angle * (1900.0f / 180.0f) + 0.5f);
}

static void load_angles(void)
{
    /* Both compare preloads must latch at the same frame boundary. UDIS
     * suppresses update events without stopping or resetting the counter. */
    uint32_t previous_udis = htim4.Instance->CR1 & TIM_CR1_UDIS;
    SET_BIT(htim4.Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us(data.servo1Angle));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_us(data.servo2Angle));
    if (!previous_udis) CLEAR_BIT(htim4.Instance->CR1, TIM_CR1_UDIS);
}

static bool configuration_valid(void)
{
    if (htim4.Instance != TIM4 || htim4.State != HAL_TIM_STATE_READY) return false;
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) timer_clock *= 2U;
    return timer_clock >= 1000000U && timer_clock % 1000000U == 0U &&
           htim4.Instance->PSC == timer_clock / 1000000U - 1U &&
           htim4.Instance->ARR == 19999U &&
           !(htim4.Instance->CR1 & (TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_UDIS | TIM_CR1_OPM)) &&
           (htim4.Instance->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M)) == TIM_OCMODE_PWM1 &&
           (htim4.Instance->CCMR1 & (TIM_CCMR1_CC2S | TIM_CCMR1_OC2M)) == (TIM_OCMODE_PWM1 << 8U) &&
           !(htim4.Instance->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P));
}

bool Servo_Init(void)
{
    if (initialized) Servo_Disable();
    initialized = false;
    data = (ServoData){0};
    if (!configuration_valid()) return false;
    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    SET_BIT(htim4.Instance->CCMR1, TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    data.servo1Angle = 90.0f;
    data.servo2Angle = 90.0f;
    load_angles();
    /* Load PSC/ARR/CCR preloads before any PWM output is enabled. */
    htim4.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    initialized = true;
    return true;
}

void Servo_Enable(void)
{
    if (!initialized || data.enabled) return;
    load_angles();
    htim4.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        return;
    }
    data.enabled = true;
}

void Servo_Disable(void)
{
    if (!initialized) return;
    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    data.enabled = false;
}

void Servo_SetAngles(float first, float second)
{
    if (!initialized || !isfinite(first) || !isfinite(second)) return;
    data.servo1Angle = clamp_angle(first);
    data.servo2Angle = clamp_angle(second);
    load_angles();
}

void Servo1_SetAngle(float angle) { Servo_SetAngles(angle, data.servo2Angle); }
void Servo2_SetAngle(float angle) { Servo_SetAngles(data.servo1Angle, angle); }

bool Servo_GetData(ServoData *output)
{
    if (!initialized || output == NULL) return false;
    *output = data;
    return true;
}
