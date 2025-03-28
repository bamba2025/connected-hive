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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFF_SIZE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rxBuffer[RX_BUFF_SIZE] = {0};

/* Variables globales pour le message reçu */
volatile uint8_t newMessageFlag = 0;
char messageBuffer[RX_BUFF_SIZE] = {0};
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

void hexToString(const char *input, char *output)
{
  while(*input && *(input+1))
  {
    char buf[3] = { input[0], input[1], '\0' };
    *output = (char) strtol(buf, NULL, 16);
    output++;
    input += 2;
  }
  *output = '\0';
}

static void extractAfterRxQuote(const char *input, char *output)
{
  // Chercher la sous-chaîne "RX \""
  const char *rxPos = strstr(input, "RX \"");
  if (rxPos != NULL)
  {
    // Avancer le pointeur juste après 'RX "'
    rxPos += 4; // 4 caractères : R(0) X(1) (espace)(2) "(3)

    // Trouver le prochain guillemet fermant
    const char *endQuote = strchr(rxPos, '"');
    if (endQuote != NULL)
    {
      size_t len = endQuote - rxPos;
      // S'assurer que la longueur ne dépasse pas RX_BUFF_SIZE
      if (len < RX_BUFF_SIZE)
      {
        strncpy(output, rxPos, len);
        output[len] = '\0'; // Fin de chaîne
      }
    }
  }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)  // Réception depuis le module LoRa
  {
    // Terminer la chaîne reçue
    if (Size < RX_BUFF_SIZE)
      rxBuffer[Size] = '\0';
    else
      rxBuffer[RX_BUFF_SIZE - 1] = '\0';

    // 1) Afficher TOUT le contenu reçu sur le moniteur (USART2) pour deboggage !
    HAL_UART_Transmit(&huart2, rxBuffer, strlen((char*)rxBuffer), 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 100);

    // 2) Extraire UNIQUEMENT la partie après "RX \"" jusqu'au guillemet suivant
    //    et la stocker dans messageBuffer
    extractAfterRxQuote((char*)rxBuffer, messageBuffer);
    if (strlen(messageBuffer) > 0)
    {
      // On a trouvé quelque chose après RX "
      newMessageFlag = 1;
    }

    // Réinitialiser rxBuffer et relancer la réception
    memset(rxBuffer, 0, RX_BUFF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, RX_BUFF_SIZE);
  }
}

void lcd_clear_line(uint8_t line)
{
    // Positionner le curseur en début de ligne
    lcd_position(&hi2c1, 0, line);

    // Écrire des espaces sur toute la ligne (16 caractères pour un LCD standard 16x2)
    for (int i = 0; i < 16; i++)
    {
        lcd_write(&hi2c1, ' ');
    }

    // Revenir au début de la même ligne
    lcd_position(&hi2c1, 0, line);
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Initialisation du LCD
  rgb_lcd myLCD;
  myLCD._displayfunction = 0;
  lcd_init(&hi2c1, &myLCD);

  // Message d'accueil sur le moniteur
  //char initMsg[] = "Waiting for LoRa messages...\n";
  //HAL_UART_Transmit(&huart2, (uint8_t *)initMsg, strlen(initMsg), 100);

  // Mettre le module LoRa en mode réception (commande AT selon votre module)
  //char atCmdRx[] = "AT+RESET";//Depanner le module //NECESSAIRE #1
  //char atCmdRx[] = "AT+MODE=TEST"; //Lancer le mode Test NECESSAIRE #2
  char atCmdRx[] = "AT+TEST=RXLRPKT"; //NECESSAIRE#3 La réception en rx
  HAL_UART_Transmit(&huart1, (uint8_t*)atCmdRx, strlen(atCmdRx), 100);

  // Démarrer la réception DMA
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, RX_BUFF_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    // Si un nouveau message a été extrait, on l'affiche sur le LCD
	  if (newMessageFlag)
	  {
	    // Convertir la chaîne hex en ASCII (ex. "53616C7574" devient "Salut" ou plus)
	    char asciiMsg[RX_BUFF_SIZE] = {0};
	    hexToString(messageBuffer, asciiMsg);

	    // Affichage sur le moniteur uniquement le message utile converti
	    HAL_UART_Transmit(&huart2, (uint8_t *)asciiMsg, strlen(asciiMsg), 100);
	    HAL_UART_Transmit(&huart2, (uint8_t *)"\n", 1, 100);

	    // Effacer le LCD (la ligne 0 et la ligne 1)
	    lcd_clear_line(0);
	    lcd_clear_line(1);

	    uint16_t len = strlen(asciiMsg);
	    if (len <= 16)
	    {
	        // Si le message tient sur une ligne, l'afficher sur la ligne 0
	        lcd_position(&hi2c1, 0, 0);
	        lcd_print(&hi2c1, asciiMsg);
	        // La deuxième ligne reste effacée
	    }
	    else
	    {
	        // Copier les 16 premiers caractères pour la première ligne
	        char line1[17] = {0};
	        char line2[17] = {0};
	        strncpy(line1, asciiMsg, 16);
	        line1[16] = '\0';

	        // Copier la suite (jusqu'à 16 caractères) pour la deuxième ligne
	        strncpy(line2, asciiMsg + 16, 16);
	        line2[16] = '\0';

	        lcd_position(&hi2c1, 0, 0);
	        lcd_print(&hi2c1, line1);
	        lcd_position(&hi2c1, 0, 1);
	        lcd_print(&hi2c1, line2);
	    }

	    newMessageFlag = 0;
	    memset(messageBuffer, 0, RX_BUFF_SIZE);
	  }




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
