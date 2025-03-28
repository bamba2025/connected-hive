#ifndef HX711_H
#define HX711_H

#include "stm32l4xx_hal.h"

// Structure HX711
typedef struct {
    GPIO_TypeDef* dout_GPIO;  // Port GPIO de la ligne de données
    uint16_t dout_Pin;        // Broche GPIO de la ligne de données
    GPIO_TypeDef* sck_GPIO;   // Port GPIO de la ligne d'horloge
    uint16_t sck_Pin;         // Broche GPIO de la ligne d'horloge
    int32_t offset;           // Offset du capteur
    float scale;              // Facteur d'échelle du capteur
} HX711;

// Déclarations de fonctions
void HX711_Init(HX711 *hx, GPIO_TypeDef* dout_GPIO, uint16_t dout_Pin,
                GPIO_TypeDef* sck_GPIO, uint16_t sck_Pin);
int32_t HX711_Read(HX711 *hx);
int32_t HX711_GetWeight(HX711 *hx);
void HX711_Tare(HX711 *hx, uint8_t times);
void HX711_SetScale(HX711 *hx, float scale);
void HX711_SetOffset(HX711 *hx, int32_t offset);

#endif // HX711_H
