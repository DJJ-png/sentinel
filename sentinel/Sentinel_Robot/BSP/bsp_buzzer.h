#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
extern uint8_t buzzer_state; //µ±Ç°·äÃùÆ÷²¥·Å×´Ì¬

extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
extern void PlayingSong(const uint16_t *song, uint16_t len);
extern void SetBuzzerFrequence(uint16_t freq);

#endif
