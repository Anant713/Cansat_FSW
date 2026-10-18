#include "drivers/Switches.h"
#include "main.h"
#include <stddef.h>

static bool initialized;

bool Switches_Init(void)
{
    /* CubeMX owns mode/clock configuration. Reset only our output bits. */
    HAL_GPIO_WritePin(LED_Switch_GPIO_Port, LED_Switch_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Buzzer_Switch_GPIO_Port, Buzzer_Switch_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DVR_Switch_GPIO_Port, DVR_Switch_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HDRM_Switch_GPIO_Port, HDRM_Switch_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DVR_Control_GPIO_Port, DVR_Control_Pin, GPIO_PIN_RESET);
    initialized = true;
    return true;
}

static void write_output(GPIO_TypeDef *port, uint16_t pin, bool enable)
{
    if (initialized) HAL_GPIO_WritePin(port, pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Switches_LED(bool enable) { write_output(LED_Switch_GPIO_Port, LED_Switch_Pin, enable); }
void Switches_Buzzer(bool enable) { write_output(Buzzer_Switch_GPIO_Port, Buzzer_Switch_Pin, enable); }
void Switches_DVR(bool enable) { write_output(DVR_Switch_GPIO_Port, DVR_Switch_Pin, enable); }
void Switches_HDRM(bool enable) { write_output(HDRM_Switch_GPIO_Port, HDRM_Switch_Pin, enable); }
void Switch_Capture(bool enable) { write_output(DVR_Control_GPIO_Port, DVR_Control_Pin, enable); }

void Switches_Update(SwitchState *state)
{
    if (state == NULL) return;
    *state = (SwitchState){0};
    if (!initialized) return;
    state->led = (LED_Switch_GPIO_Port->ODR & LED_Switch_Pin) != 0;
    state->buzzer = (Buzzer_Switch_GPIO_Port->ODR & Buzzer_Switch_Pin) != 0;
    state->dvr = (DVR_Switch_GPIO_Port->ODR & DVR_Switch_Pin) != 0;
    state->hdrm = (HDRM_Switch_GPIO_Port->ODR & HDRM_Switch_Pin) != 0;
    state->capture = (DVR_Control_GPIO_Port->ODR & DVR_Control_Pin) != 0;
}
