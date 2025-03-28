#ifndef US_H_
#define US_H_

#include "stm32l4xx_hal.h"
#include "tim.h"  // Assurez-vous d'inclure `tim.h` pour accéder à `htim2`

void US_Init(void);          // Démarrer uniquement TIM2, sans réinitialiser son initialisation
void delay_us(uint32_t us);  // Délai en microsecondes
uint32_t getMicros(void);    // Obtenir le temps actuel (en μs)

#endif /* US_H_ */
