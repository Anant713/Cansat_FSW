#ifndef ACTUATOR_MOCK_MAIN_H
#define ACTUATOR_MOCK_MAIN_H
#include <stdint.h>
typedef struct { uint32_t ODR; } GPIO_TypeDef;
extern GPIO_TypeDef gpio;
#define LED_Switch_GPIO_Port (&gpio)
#define Buzzer_Switch_GPIO_Port (&gpio)
#define DVR_Switch_GPIO_Port (&gpio)
#define HDRM_Switch_GPIO_Port (&gpio)
#define DVR_Control_GPIO_Port (&gpio)
#define LED_Switch_Pin (1U << 12)
#define Buzzer_Switch_Pin (1U << 11)
#define DVR_Switch_Pin (1U << 10)
#define HDRM_Switch_Pin (1U << 8)
#define DVR_Control_Pin (1U << 9)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, int);
typedef struct { uint32_t CR1, PSC, ARR, CCMR1, CCER, EGR, CNT, SR, CCR[2]; } TIM_TypeDef;
extern TIM_TypeDef timer;
#define TIM4 (&timer)
typedef struct { TIM_TypeDef *Instance; int State; } TIM_HandleTypeDef;
typedef struct { uint32_t APB1CLKDivider; } RCC_ClkInitTypeDef;
#define RCC_HCLK_DIV1 1U
#define HAL_TIM_STATE_READY 1
#define HAL_OK 0
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CR1_UDIS 2U
#define TIM_CR1_OPM 8U
#define TIM_CR1_DIR 16U
#define TIM_CR1_CMS 96U
#define TIM_CCMR1_CC1S 3U
#define TIM_CCMR1_CC2S (3U << 8)
#define TIM_CCMR1_OC1M 112U
#define TIM_CCMR1_OC2M (112U << 8)
#define TIM_CCMR1_OC1PE 8U
#define TIM_CCMR1_OC2PE (8U << 8)
#define TIM_OCMODE_PWM1 96U
#define TIM_CCER_CC1P 2U
#define TIM_CCER_CC2P 32U
#define TIM_EGR_UG 1U
#define TIM_FLAG_UPDATE 1U
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define __HAL_TIM_SET_COMPARE(h, ch, value) ((h)->Instance->CCR[ch] = (value))
#define __HAL_TIM_SET_COUNTER(h, value) ((h)->Instance->CNT = (value))
#define __HAL_TIM_CLEAR_FLAG(h, flag) ((h)->Instance->SR &= ~(flag))
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *, uint32_t *);
uint32_t HAL_RCC_GetPCLK1Freq(void);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
#endif
