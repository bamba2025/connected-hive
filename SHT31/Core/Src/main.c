/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SHT31_ADDR (0x44 << 1)  // Adresse I2C du SHT31 (dépend du câblage ADDR)
uint8_t cmd[2] = {0x24, 0x00};  // Commande pour lancer une mesure
uint8_t data[6];  // Réponse du capteur (6 octets)
rgb_lcd lcd_config;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Initialisation de l'écran LCD
  lcd_init(&hi2c1, &lcd_config);

  // Exemple 1 : Affichage de texte simple
  lcd_position(&hi2c1, 0, 0);  // Positionnement du curseur (ligne 0, colonne 0)
  lcd_print(&hi2c1, "BUREAU D'ETUDE!"); // Affichage du message "Hello, World!"

  // Exemple 2 : Affichage sur la deuxième ligne
  lcd_position(&hi2c1, 0, 1);  // Positionnement du curseur (ligne 1, colonne 0)
  lcd_print(&hi2c1, "STM32");  // Affichage du message "STM32 + LCD"

  // Exemple 3 : Changer la couleur du rétroéclairage
  reglagecouleur(255, 0, 0); // Rouge
  HAL_Delay(1000); // Attendre 1 seconde
  reglagecouleur(0, 255, 0); // Vert
  HAL_Delay(1000); // Attendre 1 seconde
  reglagecouleur(0, 0, 255); // Bleu
  HAL_Delay(1000); // Attendre 1 seconde
  //reglagecouleur(255, 255, 255); // Blanc

  // Exemple 4 : Effacer l'écran
  HAL_Delay(2000); // Attendre 2 secondes
  clearlcd(); // Efface tout ce qui s'affiche

  //Capteur

  HAL_Delay(2000); // Attendre 2 secondes
  clearlcd(); // Efface tout ce qui s'affiche

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buffer_temp[50];
  char buffer_humi[50];

  while (1) {
      // 1️⃣ Envoi de la commande de mesure
      if (HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, cmd, 2, HAL_MAX_DELAY) != HAL_OK) {
          lcd_position(&hi2c1, 0, 0);
          lcd_print(&hi2c1, "I2C TX Error");
          HAL_Delay(2000);
          continue;
      }

      HAL_Delay(100);

      // 2️⃣ Réception des données
      if (HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, data, 6, HAL_MAX_DELAY) != HAL_OK) {
          lcd_position(&hi2c1, 0, 0);
          lcd_print(&hi2c1, "I2C RX Error");
          HAL_Delay(2000);
          continue;
      }

      // 3️⃣ Vérification des données reçues
      if (data[0] == 0x00 && data[1] == 0x00 && data[3] == 0x00 && data[4] == 0x00) {
          lcd_position(&hi2c1, 0, 0);
          lcd_print(&hi2c1, "Capteur NOK");
          HAL_Delay(2000);
          continue;
      }

      // 4️⃣ Traitement des données reçues
      uint16_t ligne_temperature = (data[0] << 8) | data[1];
      uint16_t ligne_humidite = (data[3] << 8) | data[4];

      int temp_int = (-45 * 100 + (175 * (int)ligne_temperature) / 655);  // Multiplié par 100
      int humi_int = (100 * (int)ligne_humidite) / 655;                   // Multiplié par 100

      char buffer_temp[10];
      char buffer_humi[10];

      sprintf(buffer_temp, "%d.%d", temp_int / 100, temp_int % 100);
      sprintf(buffer_humi, "%d.%d", humi_int / 100, humi_int % 100);

      // 5️⃣ Affichage sur LCD
      lcd_position(&hi2c1, 0, 0);
      lcd_print(&hi2c1, "TEMP: ");

      lcd_position(&hi2c1, 6, 0);
      lcd_print(&hi2c1, buffer_temp);

      lcd_position(&hi2c1, 12, 0);
      lcd_print(&hi2c1, "C");

      lcd_position(&hi2c1, 0, 1);
      lcd_print(&hi2c1, "HUMI:");

      lcd_position(&hi2c1, 6, 1);
      lcd_print(&hi2c1, buffer_humi);

      lcd_position(&hi2c1, 12, 1);
      lcd_print(&hi2c1, "%");

      HAL_Delay(3000);
  }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
