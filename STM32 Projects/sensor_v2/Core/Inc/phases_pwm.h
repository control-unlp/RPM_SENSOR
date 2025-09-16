/*
 * phases_pwm.h
 *
 *  Created on: Sep 16, 2025
 *      Author: thiag
 */

#ifndef PHASES_PWM_H
#define PHASES_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PHASES_H
#define PHASES_H

#include "main.h"

typedef enum {
    PHASE_SAFE = 0,
    PHASE_INICIAL,
    PHASE_ESCALON,
    PHASE_RALENTI,
    PHASE_FINAL,
    PHASE_DONE
} Phase_t;

void PWM_SetPulse_us(uint16_t pulse_us);
void Phases_Start(void);
void Phases_Update(void);

#endif // PHASES_H

#ifdef __cplusplus
}
#endif

#endif /* PHASES_PWM_H */
