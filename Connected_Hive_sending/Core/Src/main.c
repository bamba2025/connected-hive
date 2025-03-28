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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "stdlib.h"
#include "hx711.h"
#include "us.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFF_SIZE 100 //LoRa
#define DHT11_PORT GPIOB  // Port utilisé
#define DHT11_PIN GPIO_PIN_5 // Pin utilisé
#define SHT31_ADDR (0x44 << 1)  // Adresse I2C du SHT31 (dépend du câblage ADDR)
uint8_t cmd[2] = {0x24, 0x00};  // Commande pour lancer une mesure
uint8_t data[6];  // Réponse du capteur (6 octets)
rgb_lcd lcd_config;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rxBuff[RX_BUFF_SIZE] = {0}; //LoRa
uint8_t sequence = 0;
char sensorData[150] = {0};
char atCommand[200] = {0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Presence, Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
uint8_t TEMP, RH;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void delay_us1(uint16_t time);
void Display_Temp(uint8_t Temp);
void Display_Rh(uint8_t Rh);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HX711 hx711;

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

void delay_us1(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < time);
}

void DHT11_Start(void)
{
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
    delay_us1(18000);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t Check_Response(void)
{
	//uint8_t Response = 0;
    uint32_t timeout = 10000; // Timeout pour éviter une boucle infinie

    delay_us1(40);
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout > 0) {
        timeout--;
        delay_us1(1);
    }

    if (timeout == 0) {
        return -1;  // Le capteur ne répond pas
    }

    timeout = 10000; // Réinitialisation du timeout
    delay_us1(80);
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout > 0) {
        timeout--;
        delay_us1(1);
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
        delay_us1(40);
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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Transmit(&huart2, (uint8_t*)"USART2 Initialise ! Pret a communiquer avec PuTTY...\r\n", 54, 20);
  US_Init();
  HX711_Init(&hx711, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1);
  HX711_Tare(&hx711, 10);  // opération de tare
  uint32_t lastDisplay = 0;

  HAL_TIM_Base_Start(&htim6);
  rgb_lcd lcd_data;
  lcd_init(&hi2c1, &lcd_data);

  lcd_send_string("   LA RUCHE !!!");
  HAL_Delay(2000);
  clearlcd();

  void Display_Temp(uint8_t Temp)
  {
      char str[16] = {0};
      lcd_put_cur(0, 0);
      sprintf(str, "TEMP : %dC   ", Temp);
      lcd_send_string(str);
  }


  void Display_Rh(uint8_t Rh)
  {
      char str[16] = {0};
      lcd_put_cur(0, 1);
      sprintf(str, "HUM : %d %%    ", Rh);
      lcd_send_string(str);
  }

  //char buffer_temp[50];
  //char buffer_humi[50];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //DHT11 debut

      DHT11_Start();
      Presence = Check_Response();

      if (Presence != 1)
      {
          lcd_put_cur(1, 0);
          lcd_send_string("PATIENCE !  ");
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
      lcd_position(&hi2c1, 0, 0);  // Positionnement du curseur (ligne 0, colonne 0)
      lcd_print(&hi2c1, "DONNEES INTERNES"); // Affichage du message "Hello, World!"
      HAL_Delay(1500);
      clearlcd();
      HAL_Delay(100);

      Display_Temp(TEMP);
      Display_Rh(RH);
      HAL_Delay(3000);
      clearlcd();
      HAL_Delay(100);
      //DHT11 fin

      //SHT31 debut
      lcd_position(&hi2c1, 0, 0);  // Positionnement du curseur (ligne 0, colonne 0)
      lcd_print(&hi2c1, "DONNEES EXTERNES"); // Affichage du message "Hello, World!"
      HAL_Delay(1500);
      clearlcd();
      HAL_Delay(100);
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
      lcd_print(&hi2c1, "HUM:");

      lcd_position(&hi2c1, 6, 1);
      lcd_print(&hi2c1, buffer_humi);

      lcd_position(&hi2c1, 12, 1);
      lcd_print(&hi2c1, "%");


      HAL_Delay(1500);
      //clearlcd();
      //HAL_Delay(100);
      //SHT31 fin

	  //HX711 debut // A commenter si la deuxième ligne du lcd ne fonctionne pas (debut-fin)
	  // Mesure et transmission du poids sur USART2 (vous pouvez conserver cette partie si nécessaire)
	    int32_t weight = HX711_GetWeight(&hx711);
	    char buf_usart[50];
	    sprintf(buf_usart, "Poids : %ld g\r\n", weight);
	    HAL_UART_Transmit(&huart2, (uint8_t *)buf_usart, strlen(buf_usart), HAL_MAX_DELAY);

	    // Vérifier si 1 minute (60000 ms) s'est écoulée depuis le dernier affichage sur le LCD
	    if (HAL_GetTick() - lastDisplay >= 60000)
	    {
	        lastDisplay = HAL_GetTick();

	        // Préparer la chaîne à afficher sur le LCD
	        char buf_lcd[16];
	        sprintf(buf_lcd, "Poids : %ld g", weight);
	        clearlcd();
	        // Positionner le curseur et afficher le poids sur le LCD
	        lcd_position(&hi2c1, 0, 0);
	        lcd_print(&hi2c1, buf_lcd);

	        // Laisser le temps à l'utilisateur de lire (par exemple 3 secondes)
	        HAL_Delay(3000);
	        clearlcd();
	        HAL_Delay(100);
	    }
	    //HX711 fin


      //debut d'envoie LoRa


      switch(sequence)
      {
        case 0:
            // Envoyer le poids uniquement
            sprintf(sensorData, "Poids: %ld g", weight);
            break;
        case 1:
            // Envoyer les données internes (DHT11)
            sprintf(sensorData, "Ti:%dC,Hi:%d%%", TEMP, RH);
            break;
        case 2:
            // Envoyer les données externes (SHT31)
            sprintf(sensorData, "Tx:%sC,Hx:%s%%", buffer_temp, buffer_humi);
            break;



        case 3:
            // Envoyer l'état du capteur PIR
            // On lit l'état de la broche PB4. Si HIGH, on considère que le mouvement est détecté.
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET)
                sprintf(sensorData, "TOR:motion");
            else
                sprintf(sensorData, "TOR:No motion");
            break;


        default:
            break;
      }

      // Construire la commande AT avec le message construit
      sprintf(atCommand, "AT+TEST=TXLRSTR, \"%s\"", sensorData);
      //sprintf(atCommand, "AT+MODE=TEST"); //Necessaire pour commencer la transmission


      // Affichage sur le moniteur pour vérification
      HAL_UART_Transmit(&huart2, (uint8_t *)"Sending command: ", 17, 100);
      HAL_UART_Transmit(&huart2, (uint8_t *)atCommand, strlen(atCommand), 100);
      HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);

      // Envoi de la commande AT via le module LoRa (USART1)
      HAL_UART_Transmit(&huart1, (uint8_t *)atCommand, strlen(atCommand), 100);


      // Optionnel : réception et affichage de la réponse du module LoRa sur le moniteur
      HAL_UART_Receive(&huart1, rxBuff, RX_BUFF_SIZE, 2000);
      HAL_UART_Transmit(&huart2, rxBuff, strlen((char *)rxBuff), 100);

      // Attendre avant de passer à la séquence suivante
      HAL_Delay(1000);

      // Passer à la séquence suivante (0->1->2->0...)
      sequence = (sequence + 1) % 4;

     // HAL_Delay(3000);
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
