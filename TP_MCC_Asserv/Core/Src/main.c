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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <myShell.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENC_TICKS_PER_REV 4000
#define ENC_FREQ_ECH 50
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t power = 0; // État de l'etage de puissance
uint8_t it_btn_ready = 0;

int32_t ticks = 0;

uint32_t adc_value[3] = {0,0,0};
float red_current = 0, yel_current = 0, diff_current = 0;

float consigneU = 0, consigneI = 2.0;
float sI_moins = 0, erreurI_moins;
float s = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * pinout : Affiche toutes les broches connectées et leur fonction
 * @param argc Nombre d'éléments de la ligne de commande
 * @param argv Éléments sous la forme d'un tableau de chaînes de caractères
 * @return 0
 */
int pinout(int argc, char ** argv){
	printf(" +-------------------------------------+ \r\n");
	printf(" |               PINOUT                | \r\n");
	printf(" +-------------------------------------+ \r\n");
	printf(" | STM32                | HACHEUR      | \r\n");
	printf(" +-------------------------------------+ \r\n");
	printf(" | TIM1_CH2  | PA9(D8)  | YEL_TOP | 12 | \r\n");
	printf(" | TIM1_CH1  | PA8(D7)  | RED_TOP | 13 | \r\n");
	printf(" | ADC1_IN9  | PC3      | YEL_HAL | 16 | \r\n");
	printf(" | TIM1_CH2N | PB0(A3)  | YEL_BOT | 30 | \r\n");
	printf(" | TIM1_CH1N | PA7(D11) | RED_BOT | 31 | \r\n");
	printf(" | GPIO EXTI | PC0(A5)  | ISO_RST | 33 | \r\n");
	printf(" | ADC1_IN8  | PC2      | RED_HAL | 35 | \r\n");
	printf(" | GPIO EXTI | PC13(BTN)|         |    |\r\n");
	printf(" +-------------------------------------+ \r\n");
	printf(" | STM32                | ENCODEUR     | \r\n");
	printf(" +-------------------------------------+ \r\n");
	printf(" | TIM2_CH1  | PA0(A0)  | A            | \r\n");
	printf(" | TIM2_CH2  | PA1(A1)  | B            | \r\n");
	printf(" | GPIO EXTI | PB8(D15) | Z            | \r\n");
	printf(" +-------------------------------------+ \r\n");
	printf(" |                 /!\\                 | \r\n");
	printf(" | Les indications des broches sur la  | \r\n");
	printf(" | carte interface sont erronees.      | \r\n");
	printf(" +-------------------------------------+ \r\n");

	return 0;
}

/**
 * start : Allume l’étage de puissance du moteur
 * @param argc Nombre d'éléments de la ligne de commande
 * @param argv Eléments sous la forme d'un tableau de chaînes de caractères
 * @return 0
 */
int start(int argc, char ** argv){
	printf("Power ON\r\n");
	power = 1;

	// Il faut mettre la broche ISO RESET à l'état haut pendant 2µs min
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);

	// Démarrage du timer pour la génération de signaux PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	// Signal pour le déclenchement de la mesure
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);

	return 0;
}

/**
 * stop : Éteint l’étage de puissance du moteur en coupant les signaux de commande
 * @param argc Nombre d'éléments de la ligne de commande
 * @param argv Eléments sous la forme d'un tableau de chaînes de caractères
 * @return 0
 */
int stop(int argc, char ** argv){
	printf("Power OFF\r\n");
	power = 0;

	// Extinction du timer pour la génération de signaux PWM
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

	HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3);

	return 0;
}

/**
 * sh_speed : Modification du rapport cyclique depuis le shell
 * @param argc Nombre d'éléments de la ligne de commande
 * @param argv Eléments sous la forme d'un tableau de chaînes de caractères
 * @return 0
 */
int sh_speed(int argc, char ** argv){
	if(argc == 2){
		float duty_cycle = atof(argv[1]);
		printf("Rapport cyclique = %f\r\n",duty_cycle);

		if(duty_cycle < 0) duty_cycle = 0;
		else if(duty_cycle > 80) duty_cycle = 80;

		float cmd = 1023 * (duty_cycle/100);
		TIM1->CCR1 = (int)cmd;
		printf("cmd  = %d\r\n",(int)cmd);

		float cmdn = 1023 - cmd;
		TIM1->CCR2 = (int)cmdn;
		printf("cmdn = %d\r\n",(int)cmdn);
	}

	return 0;
}

/**
 * speed : Modification du rapport cyclique depuis le code
 * @param duty_cycle Nouveau rapport cyclique
 * @return 0
 */
int speed(float duty_cycle){
	if(duty_cycle < 0) duty_cycle = 0;
	else if(duty_cycle > 80) duty_cycle = 80;

	float cmd = 1023 * (duty_cycle/100);
	TIM1->CCR1 = (int)cmd;

	float cmdn = 1023 - cmd;
	TIM1->CCR2 = (int)cmdn;

	return 0;
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
	MX_LPUART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM6_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */

	/* Configuration et démarrage du Shell --------------*/
	shell_init(&hlpuart1);
	shell_add("pinout", pinout, "Broches utilisees avec leur fonction");
	shell_add("start", start, "Allumage etage de puissance du moteur");
	shell_add("stop", stop, "Extinction etage de puissance du moteur");
	shell_add("speed", sh_speed, "(Test) Modification de la vitesse du moteur");

	/* Initialisation des PWM à 60% ---------------------*/
	TIM1->CCR1 = 614;
	TIM1->CCR2 = 1023-614;

	/* Initialisation du CCR pour Channel 3 du TIM1 -----*/
	TIM1->CCR3 = 64;

	/* Démarrage des timers génériques ------------------*/
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	/* Démarrage du Timer 2 en encoder mode -------------*/
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 || TIM_CHANNEL_2);

	/* Démarrage de l'ADC avec le DMA -------------------*/
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Traitement du shell ------------------------------*/
		if(it_uart_rx_ready){
			shell_char_received();
			it_uart_rx_ready = 0;
		}

		/* Traitement de l'appui bouton bleu ----------------*/
		if(it_btn_ready){
			if(power == 1)
				stop(0, NULL);
			else
				start(0, NULL);
			it_btn_ready = 0;
		}

		/* (Test) Calcul du courant traversant le moteur ----*/
		//red_current = (((float)adc_value[0] * (3.3/4096.0)) - 2.5) * 12.0;
		//yel_current = (((float)adc_value[1] * (3.3/4096.0)) - 2.5) * 12.0;
		//diff_current = (yel_current - red_current * (-1)) / 2 + red_current * (-1);
		//printf("red = %.2fA\tyel = %.2fA\tdif = %.2fA\r\n", red_current, yel_current, diff_current);
		//printf("dif = %.2fA\ts = %.2f\r\n", diff_current, s);

		HAL_Delay(100);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		it_btn_ready = 1;
	}

	if(GPIO_Pin == GPIO_PIN_8){
		/*
		// Mesure du nombre de ticks par tour ---------------
		ticks = TIM2->CNT;
		TIM2->CNT = 0;
		if(hacheurStart) printf("ticks/tour = %d\r\n",(int)ticks);
		 */
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC1){
		// Empty
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM7){
		/* Boucle de courant --------------------------------*/

		// Conversion de la mesure de l'ADC en courant
		red_current = (((float)adc_value[0] * (3.3/4096.0)) - 2.5) * 12.0;
		yel_current = (((float)adc_value[1] * (3.3/4096.0)) - 2.5) * 12.0;
		diff_current = (yel_current - red_current * (-1)) / 2 + red_current * (-1);

		// Calcul de la commande
		float erreurI = consigneI - diff_current;
		s = erreurI * 0.8 +
				sI_moins + 0.1 * (erreurI + erreurI_moins) * (1 / 2 * 500);

		// Saturation de la commande
		if (fabs(s) >= 70) s = 70;

		// Mémorisation des valeurs calculées
		sI_moins = s;
		erreurI_moins = erreurI;

		// Mise à jour des signaux PWM avec la nouvelle commande
		speed(s);
	}

	if(htim->Instance == TIM6){
		/* Boucle de vitesse --------------------------------*/
		// Ne fait que la mesure de vitesse

		ticks = TIM2->CNT;
		TIM2->CNT = 0;

		float vit = ((float)ticks * 2 * M_PI * ENC_FREQ_ECH) / (float)ENC_TICKS_PER_REV;

		if(power){
			//printf("ticks = %d\r\n",ticks);
			//printf("vit = %f rad/s\r\n",vit);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == LPUART1){
		// La fonction d'interruption ne fait que modifier it_uart_rx_ready
		it_uart_rx_ready = 1;
		HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&c, 1);
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
