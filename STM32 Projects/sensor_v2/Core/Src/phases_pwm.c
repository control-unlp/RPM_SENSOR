/*
 * phases_pwm.c
 *
 *  Created on: Sep 16, 2025
 *      Author: thiag
 */

#include "phases_pwm.h"

extern TIM_HandleTypeDef htim1;   // declarado en main.c

// Tu función de PWM
// extern void PWM_SetPulse_us(uint16_t pulse_us);
volatile uint16_t currentPulse = 1000;

void PWM_SetPulse_us(uint16_t pulse_us) {
    // CCR = pulse_us / 1 µs * factor de timer
    // Si TIM1 está a 72 MHz con prescaler= 71, entonces 1 tick = 1 µs
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
    currentPulse = pulse_us;
}

// Estado actual
static Phase_t currentPhase = PHASE_SAFE;
static uint32_t phaseStart = 0;

void Phases_Start(void)
{
    currentPhase = PHASE_SAFE;
    phaseStart = HAL_GetTick();
    PWM_SetPulse_us(1000);   // seguro
}

void Phases_Update(void)
{
    uint32_t elapsed = HAL_GetTick() - phaseStart;

    switch (currentPhase) {
        case PHASE_SAFE:
            if (elapsed > 1000) {    // 1s
                currentPhase = PHASE_INICIAL;
                phaseStart = HAL_GetTick();
                PWM_SetPulse_us(1200);
            }
            break;

        case PHASE_INICIAL:
            if (elapsed > 2000) {    // 2s
                currentPhase = PHASE_ESCALON;
                phaseStart = HAL_GetTick();
                PWM_SetPulse_us(1500);
            }
            break;

        case PHASE_ESCALON:
            if (elapsed > 2000) {    // 2s
                currentPhase = PHASE_RALENTI;
                phaseStart = HAL_GetTick();
                PWM_SetPulse_us(1100);
            }
            break;

        case PHASE_RALENTI:
            // si querés que quede fijo hasta que lo cortes, no hagas nada acá
            break;

        case PHASE_FINAL:
            if (elapsed > 1000) {   // 1s
                currentPhase = PHASE_DONE;
                PWM_SetPulse_us(1000);
            }
            break;

        case PHASE_DONE:
            // nada más que hacer
            break;
    }
}
