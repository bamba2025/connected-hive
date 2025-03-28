#include "us.h"

extern TIM_HandleTypeDef htim2;  // Utilisation directe du handle TIM2 défini dans tim.c

// **1️⃣ Démarrage du chronomètre TIM2**
void US_Init(void) {
    HAL_TIM_Base_Start(&htim2);  // Démarre TIM2 sans réinitialiser son initialisation
}

// **2️⃣ Obtenir le temps actuel (en microsecondes)**
uint32_t getMicros(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);  // Lit la valeur du compteur de TIM2
}

// **3️⃣ Délai en microsecondes**
void delay_us(uint32_t us) {
    uint32_t start_time = getMicros();  // Enregistre le temps de départ
    while ((getMicros() - start_time) < us);  // Attente pendant la durée spécifiée en microsecondes
}
