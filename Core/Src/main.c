/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include <stdbool.h>
#include <stdio.h>
#include "gps_neo6.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SPI_HandleTypeDef hspi1;
int8_t data_rec[6];
int16_t x, y, z;
float xg, yg, zg;
uint8_t komunikat[40];//zwracany komunikat
uint16_t dl_kom;

uint8_t Message[64];
uint8_t MessageLength;

struct lcd_disp disp;

uint8_t counter = 1 ;//1 aklcelerometr, 2 lokalizajca, 3 predkosc
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_write(uint8_t address, uint8_t value){
	uint8_t data[2];
	data[0] = address | 0x40;
	data[1] = value;
	HAL_GPIO_WritePin(GPIOB, LIS_SPI_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, data, 2, 100);
	HAL_GPIO_WritePin(GPIOB, LIS_SPI_Pin, GPIO_PIN_SET);
}


void adxl_read(uint8_t address){
	address |= 0x80;
	address |= 0x40;
	HAL_GPIO_WritePin(GPIOB, LIS_SPI_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Receive(&hspi1, data_rec, 6, 100);
	HAL_GPIO_WritePin(GPIOB, LIS_SPI_Pin, GPIO_PIN_SET);
}

void adxl_init(void){
	adxl_write(0x31, 0x01); // data_format range 4g
	adxl_write(0x2d, 0x08); // reset all bits
	adxl_write(0x2d, 0x08); // power_cntl measure and wake up 8 hz


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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //docelowo inicjalizacja wyswietlacza
  disp.addr = (0x27 << 1);
  disp.addr = (0x27 << 1);
  disp.bl = true;
  lcd_init(&disp);
  sprintf((char *) disp.f_line, "INICJALIZACJA");
  sprintf((char *) disp.s_line, "AKCELEROMETR");
  lcd_display(&disp);
  HAL_Delay(1000);
  adxl_init();
  dl_kom = sprintf(komunikat, "START\n");
  HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);

  NEO6_Init(&GpsState, &huart1);
  uint32_t Timer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    NEO6_Task(&GpsState);

    	  if((HAL_GetTick() - Timer) > 1000)
    	  {
    	//	  MessageLength = sprintf((char*)Message, "\033[2J\033[;H"); // Clear terminal and home cursor
    		//  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    		  if(NEO6_IsFix(&GpsState))
    		  {

    			  MessageLength = sprintf((char*)Message, "UTC Time: %02d:%02d:%02d\n\r", GpsState.Hour, GpsState.Minute, GpsState.Second);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Date: %02d.%02d.20%02d\n\r", GpsState.Day, GpsState.Month, GpsState.Year);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Latitude: %.2f %c\n\r", GpsState.Latitude, GpsState.LatitudeDirection);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Longitude: %.2f %c\n\r", GpsState.Longitude, GpsState.LongitudeDirection);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Altitude: %.2f m above sea level\n\r", GpsState.Altitude);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Speed: %.2f knots, %f km/h\n\r", GpsState.SpeedKnots, GpsState.SpeedKilometers);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Satelites: %d\n\r", GpsState.SatelitesNumber);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Dilution of precision: %.2f\n\r", GpsState.Dop);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Horizontal dilution of precision: %.2f\n\r", GpsState.Hdop);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);

    			  MessageLength = sprintf((char*)Message, "Vertical dilution of precision: %.2f\n\r", GpsState.Vdop);
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);


    			  if(counter == 2){
						//HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
						sprintf((char *) disp.f_line, "Latitud:%.2f%c", GpsState.Latitude, GpsState.LatitudeDirection);
						sprintf((char *) disp.s_line, "Longitu:%.2f%c", GpsState.Longitude, GpsState.LongitudeDirection);
						lcd_display(&disp);
    			  }

    			  if(counter == 3){
						//HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
						sprintf((char *) disp.f_line, "Speed: %.1f km/h", GpsState.SpeedKilometers);
						sprintf((char *) disp.s_line, "TIME:%02d:%02d:%02d", GpsState.Hour, GpsState.Minute, GpsState.Second);
						lcd_display(&disp);
					}

    			  if(counter == 1){
    			  adxl_read(0x32);
    			  	x = (( data_rec[1] << 8)  | data_rec[0]);
    			  	y = (( data_rec[3] << 8 ) | data_rec[2]);
    			  	z = (( data_rec[5] << 8 ) | data_rec[4]);

    			  	//convert to 'g'
    			  	xg = x * .0078;
    			  	yg = y * .0078;
    			  	zg = z * .0078;

    			  	//display
    			  	dl_kom = sprintf(komunikat, "xg: %.3f, yg: %.3f, zg: %.3f \n", xg, yg, zg);
				    HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
				    sprintf((char *) disp.f_line, "x:%.2f", xg);
				    sprintf((char *) disp.s_line, "y:%.2f,z:%.2f", yg, zg);
				    lcd_display(&disp);
    			  }

    		  }
    		  else
    		  {

    			  MessageLength = sprintf((char*)Message, "No Fix\n\r");
    			  HAL_UART_Transmit(&huart2, Message, MessageLength, 1000);


    			  adxl_read(0x32);
    			  	x = (( data_rec[1] << 8)  | data_rec[0]);
    			  	y = (( data_rec[3] << 8 ) | data_rec[2]);
    			  	z = (( data_rec[5] << 8 ) | data_rec[4]);

    			  	//convert to 'g'
    			  	xg = x * .0078;
    			  	yg = y * .0078;
    			  	zg = z * .0078;

    			  	//display
    			  	if(counter == 1){
    			  	dl_kom = sprintf(komunikat, "xg: %.3f, yg: %.3f, zg: %.3f \n", xg, yg, zg);
    			      HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
    			      sprintf((char *) disp.f_line, "x:%.2f", xg);
    			      sprintf((char *) disp.s_line, "y:%.2f,z:%.2f", yg, zg);
    			      lcd_display(&disp);
    			  }
    			  	if(counter == 2){
    			  	    			  	dl_kom = sprintf(komunikat, "xg: %.3f, yg: %.3f, zg: %.3f \n", xg, yg, zg);
    			  	    			      HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
    			  	    			      sprintf((char *) disp.f_line, "NO FIX");
    			  	    			      sprintf((char *) disp.s_line, "NO LOCATION ");
    			  	    			      lcd_display(&disp);
    			  	    			  }
    			  	if(counter == 3){
    			  	    			  	dl_kom = sprintf(komunikat, "xg: %.3f, yg: %.3f, zg: %.3f \n", xg, yg, zg);
    			  	    			      HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
    			  	    			      sprintf((char *) disp.f_line, "NO FIX");
    			  	    			      sprintf((char *) disp.s_line, "NO SPEED/TIME");
    			  	    			      lcd_display(&disp);
    			  	    			  }

    		  }

    		  Timer = HAL_GetTick();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == GpsState.neo6_huart)
	{
		NEO6_ReceiveUartChar(&GpsState);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == Button_Pin){
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		counter++;
		if(counter > 3)
				counter = counter / 3;
		dl_kom = sprintf(komunikat, "%d", counter);
		HAL_UART_Transmit(&huart2, komunikat, dl_kom, 1000);
	}
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
