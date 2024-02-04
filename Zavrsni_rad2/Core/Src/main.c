/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <lcd.h>
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
uint8_t rxPodatak;
const float BrzinaZvuka = 0.0343 / 2;
float udaljenost1, udaljenost2;
char Buffer[100];
char buffer2[100];
float popunjenost;
uint8_t Flag1 = 0, Flag2 = 0;
uint8_t Capture_indeks1 = 0, Capture_indeks2 = 0;
uint32_t Brid1Vrijeme_1 = 0, Brid2Vrijeme_1 = 0, Brid1Vrijeme_2= 0,Brid2Vrijeme_2 = 0;
uint8_t popunjenost_int;
uint32_t Vrijeme_Zadnjeg_Slanja = 0;
uint32_t Vrijeme_Zadnjeg_Slanja2 = 0;
#define Delay_ms 2000
#define Delay_ms2 2000
uint8_t kanta_stanje=0;
uint8_t aplikacija_stanje=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Delay_us(uint32_t us);

void Ultrazvucni_senzor1(void) {
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);   // Postavljanje TRIG pin-a u nisko stanje (low) na 3us
	Delay_us(3);
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_SET);     // Postavljanje TRIG pin-a u visoko stanje (high) na 10us
	Delay_us(10);
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);   // Vraćanje TRIG pin-a u nisko stanje (low)

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);                    // Pokretanje input capture Timera 3, channel 1 s interruptom
	uint32_t startniTick1 = HAL_GetTick();
	do {
		if (Flag1) break;                           // Provjerava se vrijednost Flag-a, ako je vrijednost Flag-a "1" prekida se petlja (zna�?i da smo dobili vrijeme prvog i drugog brida) iz Callback funkcije
	} while ((HAL_GetTick() - startniTick1) < 500);                  // Ako puls nije došao unutar 500 ms (iz nekog razloga) stopira se timer kako program ne bi zapeo
	Flag1 = 0;
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);                     // Stopiranje timera

	if (Brid2Vrijeme_1 > Brid1Vrijeme_1) {
		udaljenost1 = ((Brid2Vrijeme_1 - Brid1Vrijeme_1) + 0.0f) * BrzinaZvuka; // Izračun udaljenosti u cm pomoću dobivenih vrijednosti bridova i formule  te pretvaranje u float varijablu
	}
	else {
		udaljenost1 = 0.0f;
	}
}
void Servo_pomicanje() {
    if (udaljenost1 != 0.0 && udaljenost1 < 20.0 && aplikacija_stanje==0) {
    	lcd_ocisti();
    	lcd_postavi_kursor(0, 0);
    	lcd_slanje_stringa("Kanta otvorena");
        kanta_stanje = 1;
        htim2.Instance->CCR1 = 95;  // Faktor vođenja = 9,5%
        HAL_Delay(6000);

        htim2.Instance->CCR1 = 65;  // Faktor vođenja = 6,5%
        HAL_Delay(2000);

        kanta_stanje = 0;
    }
}
void Ultrazvucni_senzor2(void){
	//Drugi senzor HCSR-04
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);  // Postavljanje TRIG pin-a u nisko stanje (low) na 3us
	Delay_us(4);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);    // Postavljanje TRIG pin-a u visoko stanje (high) na 10us
	Delay_us(10);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);  // Vraćanje TRIG pin-a u nisko stanje (low)

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);                     //Pokretanje input capture Timera 3, channel 2 s interruptom
	uint32_t startniTick2 = HAL_GetTick();                            //Po�?etni tick
	do {
		if (Flag2) break;                           // Provjerava se vrijednost Flag-a, ako je vrijednost Flag-a "1" prekida se petlja (zna�?i da smo dobili vrijeme prvog i drugo brida) iz Callback funkcije
	} while ((HAL_GetTick() - startniTick2) < 500);                   // Ako puls nije došao unutar 500 ms (iz nekog razloga) stopira se timer kako program nebi zapeo
	Flag2 = 0;
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);                      // Stopiranje timera

	if (Brid2Vrijeme_2 > Brid1Vrijeme_2) {
		udaljenost2 = ((Brid2Vrijeme_2 - Brid1Vrijeme_2) + 0.0f) * BrzinaZvuka;  //Izračun udaljenosti u cm pomoću dobivenih vrijednosti bridova i formule  te pretvaranje u float varijablu
		popunjenost=((1.0 - 0.0) / (3.0 - 24.7)) * (udaljenost2 - 24.7) * 100.0;

		popunjenost_int = (uint8_t)(round(popunjenost / 5.0) * 5);

	}
}
void UART_slanje_postotka(uint8_t* podatak, size_t velicina) {
	uint16_t Trenutno_Vrijeme = HAL_GetTick();

	if ((Trenutno_Vrijeme - Vrijeme_Zadnjeg_Slanja) >= Delay_ms ) {

		HAL_UART_Transmit(&huart3, podatak, velicina, 100);


		Vrijeme_Zadnjeg_Slanja = Trenutno_Vrijeme;
	}
}

void Lcd_prikaz(uint8_t popunjenost_int) {
	uint16_t Trenutno_Vrijeme2 = HAL_GetTick();


	if (Trenutno_Vrijeme2 - Vrijeme_Zadnjeg_Slanja2 >= Delay_ms2 && kanta_stanje == 0) {
		lcd_ocisti();
		if (popunjenost_int < 85) {
		    sprintf(buffer2, "Popunjeno %d%%", popunjenost_int);
		} else if (popunjenost_int >= 85 ) {
		    sprintf(buffer2, "Isprazni kantu!");
		    lcd_postavi_kursor(1, 0);
		    lcd_slanje_stringa("Kanta je puna!");
		}


		lcd_postavi_kursor(0, 0);
		lcd_slanje_stringa(buffer2);

		Vrijeme_Zadnjeg_Slanja2 = Trenutno_Vrijeme2;

	}
}

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
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3,&rxPodatak,1);
	lcd_inicijalizacija ();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	lcd_ocisti();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		Ultrazvucni_senzor1();
		Servo_pomicanje();
		Ultrazvucni_senzor2();
		//Ispisivanje na RealTerm
		sprintf(Buffer, "popunjenost= %.1f, popunjenost{int} = %d %, udaljenost 2 (cm) = %.1f, udaljenost 1 (cm) = %.1f\r\n ",popunjenost, popunjenost_int, udaljenost2, udaljenost1);
		HAL_UART_Transmit(&huart2, (uint8_t *)Buffer, strlen(Buffer), 100);


		Lcd_prikaz(popunjenost_int);
		UART_slanje_postotka(&popunjenost_int, sizeof(popunjenost_int));


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 90;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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
	if(huart->Instance==USART3)
	{
		if(rxPodatak==79) // Ascii 79 je "O"
		{
			kanta_stanje=1;
			aplikacija_stanje=1;
			htim2.Instance->CCR1 = 95;
			lcd_ocisti();
			lcd_postavi_kursor(0, 0);
			lcd_slanje_stringa("Kanta otvorena");



		}
		else if (rxPodatak==88) // Ascii 88 je "X"
		{
			htim2.Instance->CCR1 = 65;
			kanta_stanje=0;
			aplikacija_stanje=0;
		}
		HAL_UART_Receive_IT(&huart3,&rxPodatak,1);
	}
}
void Delay_us(uint32_t us)
{
	if(us < 2) us = 2;
	TIM4->ARR = us - 1;
	TIM4->EGR = 1;
	TIM4->SR &= ~1;
	TIM4->CR1 |= 1;
	while((TIM4->SR&0x0001) != 1);
	TIM4->SR &= ~(0x0001);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// Input capture callback funkcija za Timer 3, channel 1 i channel 2, oba ultrazvu�?na senzora
	if (htim->Instance == TIM3) {    // Provjera da li se interrupt dogodio na Timeru 3
		// Prvi ultrazvucni senzor kojem je echo pin spojen na TIM3_CHANNEL_1
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {                         // Provjerava se channel timera na kojem se događa interrupt
			if (Capture_indeks1 == 0) {                                            //Provjera da li  indeks capture ima vrijednost 0
				Brid1Vrijeme_1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1); // Dohvaćanje vrijednosti prvog brida
				Capture_indeks1 = 1;                                                 //Postavljanje capture indeksa u 1
			} else if (Capture_indeks1== 1) {                                      //Provjera da li  indeks capture ima vrijednost 1
				Brid2Vrijeme_1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1); // Dohvaćanje vrijednosti drugog brida
				Capture_indeks1 = 0;                                                 // Vraćanje capture indeksa u 0
				Flag1 = 1;                                                           // Postavljanje flag-a u 1
			}
		}
		//Drugi ultrazvucni senzor kojem je echo pin spojen na TIM3_CHANNEL_2
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {                         // Provjerava se channel timera na kojem se događa interrupt
			if (Capture_indeks2 == 0) {                                            //Provjera da li  indeks capture ima vrijednost 0
				Brid1Vrijeme_2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2); // Dohvaćanje vrijednosti prvog brida
				Capture_indeks2 = 1;                                                 //Postavljanje capture indeksa u 1
			} else if (Capture_indeks2 == 1) {                                     //Provjera da li  indeks capture ima vrijednost 1
				Brid2Vrijeme_2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2); // Dohvaćanje vrijednosti drugog brida
				Capture_indeks2 = 0;                                                 // Vraćanje capture indeksa u 0
				Flag2 = 1;                                                           // Postavljanje flag-a u 1
			}
		}
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
