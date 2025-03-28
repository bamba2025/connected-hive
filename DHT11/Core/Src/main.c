/* USER CODE BEGIN Includes */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define DHT11_PORT GPIOB  // Port utilisé
#define DHT11_PIN GPIO_PIN_5 // Pin utilisé

/* Private variables ---------------------------------------------------------*/
uint8_t Presence, Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
uint8_t TEMP, RH;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void delay_us(uint16_t time);
void Display_Temp(uint8_t Temp);
void Display_Rh(uint8_t Rh);

/* USER CODE BEGIN 0 */
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay_us(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < time);
}

void DHT11_Start(void)
{
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
    delay_us(18000);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t Check_Response(void)
{
	//uint8_t Response = 0;
    uint32_t timeout = 10000; // Timeout pour éviter une boucle infinie

    delay_us(40);
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout > 0) {
        timeout--;
        delay_us(1);
    }

    if (timeout == 0) {
        return -1;  // Le capteur ne répond pas
    }

    timeout = 10000; // Réinitialisation du timeout
    delay_us(80);
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout > 0) {
        timeout--;
        delay_us(1);
    }

    if (timeout == 0) {
        return -1;  // Mauvaise réponse du capteur
    }

    return 1; // Capteur détecté
}


uint8_t DHT11_Read(void)
{
    uint8_t i = 0, j;
    for (j = 0; j < 8; j++)
    {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
        delay_us(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
        {
            i &= ~(1 << (7 - j));
        }
        else i |= (1 << (7 - j));
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
    }
    return i;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM6_Init();

    HAL_TIM_Base_Start(&htim6);

    rgb_lcd lcd_data;
    lcd_init(&hi2c1, &lcd_data);

    lcd_send_string("INIT DHT11...");
    HAL_Delay(2000);
    clearlcd();

    while (1)
    {
        DHT11_Start();
        Presence = Check_Response();

        if (Presence != 1)
        {
            lcd_put_cur(1, 0);
            lcd_send_string("Capteur loading  ");
            HAL_Delay(2000);
            continue;
        }

        Rh_byte1 = DHT11_Read();
        Rh_byte2 = DHT11_Read();
        Temp_byte1 = DHT11_Read();
        Temp_byte2 = DHT11_Read();
        SUM = DHT11_Read();

        if ((Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2) != SUM)
        {
            lcd_put_cur(1, 0);
            lcd_send_string("Checksum err ");
            HAL_Delay(2000);
            continue;
        }

        TEMP = Temp_byte1;
        RH = Rh_byte1;

        clearlcd();  // Ajoute cette ligne pour effacer l'ancien affichage**
        Display_Temp(TEMP);
        HAL_Delay(1500);
        clearlcd();
        Display_Rh(RH);

        HAL_Delay(3000);
    }


}

/* USER CODE BEGIN 4 */
void Display_Temp(uint8_t Temp)
{
    char str[16] = {0};
    lcd_put_cur(0, 0);
    sprintf(str, "Temp : %dC   ", Temp);
    lcd_send_string(str);
}


void Display_Rh(uint8_t Rh)
{
    char str[16] = {0};
    lcd_put_cur(1, 0);
    sprintf(str, "Hum: %d %%    ", Rh);
    lcd_send_string(str);
}

/* USER CODE END 4 */

/**
  * @brief  System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
