#pragma once
#include "stm32f4xx_hal.h"
typedef struct { uint32_t CYCCNT, CTRL; } MockDWT;
typedef struct { uint32_t DEMCR; } MockCoreDebug;
extern MockDWT mockDwt;
extern MockCoreDebug mockDebug;
extern uint32_t SystemCoreClock;
#define DWT (&mockDwt)
#define CoreDebug (&mockDebug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __NOP() (++mockDwt.CYCCNT)
