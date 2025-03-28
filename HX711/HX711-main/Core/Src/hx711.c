#include "hx711.h"
#include "stm32l4xx_hal.h"
#include "us.h"

#define HX711_DELAY_US 1  // Délai entre les impulsions SCK (microsecondes)

// **Initialisation du HX711**
void HX711_Init(HX711 *hx, GPIO_TypeDef* dout_GPIO, uint16_t dout_Pin,
                GPIO_TypeDef* sck_GPIO, uint16_t sck_Pin) {
    hx->dout_GPIO = dout_GPIO;
    hx->dout_Pin = dout_Pin;
    hx->sck_GPIO = sck_GPIO;
    hx->sck_Pin = sck_Pin;
    hx->offset = 0;
    hx->scale = 209.28; //209.28

    // Mise à l'état bas de SCK
    HAL_GPIO_WritePin(hx->sck_GPIO, hx->sck_Pin, GPIO_PIN_RESET);
}

// **Lecture des données sur 24 bits**
int32_t HX711_Read(HX711 *hx) {
    uint32_t data = 0;

    // Attente que le HX711 soit prêt (niveau bas sur DT)
    while (HAL_GPIO_ReadPin(hx->dout_GPIO, hx->dout_Pin));

    // Lecture des données sur 24 bits
    for (uint8_t i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(hx->sck_GPIO, hx->sck_Pin, GPIO_PIN_SET);
        delay_us(HX711_DELAY_US);
        data = (data << 1) | HAL_GPIO_ReadPin(hx->dout_GPIO, hx->dout_Pin);
        HAL_GPIO_WritePin(hx->sck_GPIO, hx->sck_Pin, GPIO_PIN_RESET);
        delay_us(HX711_DELAY_US);
    }
    // Envoi d'une impulsion supplémentaire, sélection du canal A, gain 128
    HAL_GPIO_WritePin(hx->sck_GPIO, hx->sck_Pin, GPIO_PIN_SET);
    delay_us(HX711_DELAY_US);
    HAL_GPIO_WritePin(hx->sck_GPIO, hx->sck_Pin, GPIO_PIN_RESET);
    delay_us(HX711_DELAY_US);

    // Conversion en complément à deux (nombre signé)
    if (data & 0x800000) {
        data |= 0xFF000000;  // Extension du bit de signe
    }

    return (int32_t)data;
}

// **Obtention du poids**
int32_t HX711_GetWeight(HX711 *hx) {
    return (HX711_Read(hx) - hx->offset) / hx->scale;
}

// **Tarage**
void HX711_Tare(HX711 *hx, uint8_t times) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += HX711_Read(hx);
    }
    hx->offset = sum / times;
}

// **Définir le facteur d'échelle**
void HX711_SetScale(HX711 *hx, float scale) {
    hx->scale = scale;
}

// **Définir l'offset**
void HX711_SetOffset(HX711 *hx, int32_t offset) {
    hx->offset = offset;
}
