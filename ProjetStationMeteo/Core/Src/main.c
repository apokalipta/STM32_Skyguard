/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <max7219_Yncrea2.h>
#include "iks01a3_env_sensors.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct displayFloatToInt_s
{
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUF_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float Temperature;
float humidity;
float pression;
uint8_t buzzerstate = 0;
uint32_t analogValue = 0;
uint8_t flag_temp = 0;
uint8_t flag_hum = 0;
uint8_t flag_pres = 0;

char temp_seq[]= {'T',' ','.','C'};
char hum_seq[]={'H','U','N',' '};
char pres_seq[]={'P','R','E','S'};
char menu_seq[]={'N','E','N','U'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Fonction permettant de découper le float en int
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
  {
    out_value->sign = 1;
    in = -in;
  }

  in = in + (0.5 / pow(10, dec_prec));
  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

//Fonction qui active et désactive le moteur en fonction du paramètre qu'on lui donne
void motor(uint8_t motorstate){
	if(motorstate){
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
		printf("Motor activated\r\n");
	}
	else{
		HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		printf("Motor disable\r\n");
	}

}

//Pareil que pour le moteur mais avec le buzzer
void buzzer (uint8_t buzzerState)
{
	if(buzzerState){
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	}else {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	}
}

//Ici on configure l'initialisation de nos capteurs avec les fonctions de l'IKS
void conf_capteur(void){

	IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE);
	IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_HUMIDITY);
	IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE);

}

//Création d'un affichage menu
void affichage_menu(void){

	MAX7219_DisplayChar('1',menu_seq[0]);
	MAX7219_DisplayChar('2',menu_seq[1]);
	MAX7219_DisplayChar('3',menu_seq[2]);
	MAX7219_DisplayChar('4',menu_seq[3]);
}

//Pareil pour la température
void menu_temp(void){

	MAX7219_DisplayChar('1',temp_seq[0]);
	MAX7219_DisplayChar('2',temp_seq[1]);
	MAX7219_DisplayChar('3',temp_seq[2]);
	MAX7219_DisplayChar('4',temp_seq[3]);
}

//Pareil pour l'humidité
void menu_hum(void){

	MAX7219_DisplayChar('1',hum_seq[0]);
	MAX7219_DisplayChar('2',hum_seq[1]);
	MAX7219_DisplayChar('3',hum_seq[2]);
	MAX7219_DisplayChar('4',hum_seq[3]);
}

//Pareil pour la pression
void menu_pres(void){

	MAX7219_DisplayChar('1',pres_seq[0]);
	MAX7219_DisplayChar('2',pres_seq[1]);
	MAX7219_DisplayChar('3',pres_seq[2]);
	MAX7219_DisplayChar('4',pres_seq[3]);
}

//Ici on récupère la valeur de notre température ensuite on déclare un char de 4 pour
//stocker la valeur dedans et on utilise notre fonction nous permettant de la transformer
//et à l'aide d'un sprintf on transforme notre float en int
void display_temp(void){

	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &Temperature);
	char buff[4];
	displayFloatToInt_t out_value;
	floatToInt(Temperature, &out_value, 2);
	sprintf(buff, "%d", (int)out_value.out_int);
	MAX7219_DisplayChar('1',buff[0]);
	MAX7219_DisplayChar('2',buff[1]);
	MAX7219_DisplayChar('3','.');
	sprintf(buff, "%d", (int)out_value.out_dec);
	MAX7219_DisplayChar('4',buff[0]);
}

//Exactement pareil mais pour l'humidité
void display_hum(void)
{

	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_HUMIDITY, &humidity);
	char buff[4];
	displayFloatToInt_t out_value;
	floatToInt(humidity, &out_value, 2);
	sprintf(buff, "%d", (int)out_value.out_int);
	MAX7219_DisplayChar('1',buff[0]);
	MAX7219_DisplayChar('2',buff[1]);
	MAX7219_DisplayChar('3','.');
	sprintf(buff, "%d", (int)out_value.out_dec);
	MAX7219_DisplayChar('4',buff[0]);
}

//La même chose pour la pression sauf qu'on ne met pas de virgule dans notre char
void display_pression(void)
{

	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_PRESSURE, &pression);
	char buff[10];
	displayFloatToInt_t out_value;
	floatToInt(pression, &out_value, 2);
	sprintf(buff, "%d", (int)out_value.out_int);
	MAX7219_DisplayChar('1',buff[0]);
	MAX7219_DisplayChar('2',buff[1]);
	MAX7219_DisplayChar('3',buff[2]);
	MAX7219_DisplayChar('4',buff[3]);

}

//Fonction permettant la création des différentes notes
int frequenceNote(Note note){
	int freq;
		switch(note){

		case DO:
			freq = 261;
			break;

		case RE:
			freq = 294;
			break;

		case MI:
			freq = 329;
			break;

		case FA:
			freq = 349;
			break;

		case SOL:
			freq = 392;
			break;

		case LA:
			freq = 440;
			break;

		case SI:
			freq = 493;
			break;

		case SIB:
			freq = 466;
			break;
		}
	return freq;
}

//Fonction qui vient exploiter les notes et calculer la fréquence et ensuite on rentre
//la valeur dans le registre
void buzzerFunction(Note note, int tempsMs, int pause){

	uint32_t calc = 32000/(frequenceNote(note));
	TIM3->PSC = calc;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Démarrage du Buzzer
	HAL_Delay(tempsMs);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Arret du buzzer
	HAL_Delay(pause);
}


//Musique d'acceuil quand on lance notre application qui reprend la fonction du dessus
//pour la création de la musique
void MusiqueAmbiance(void){

	buzzerFunction(DO,  500,  100);
	buzzerFunction(MI,  500,  100);
	buzzerFunction(SOL,  500,  100);
	buzzerFunction(DO,  500,  300);

	buzzerFunction(SOL,  500,  100);
	buzzerFunction(LA,  500,  100);
	buzzerFunction(SI,  500,  100);
	buzzerFunction(SOL,  500,  300);

	buzzerFunction(DO,  500,  100);
	buzzerFunction(MI,  500,  100);
	buzzerFunction(SOL,  500,  100);
	buzzerFunction(DO,  500,  300);

}

//Affichage du menu
void menu(void){

	MAX7219_DisplayChar('1',menu_seq[0]);
	MAX7219_DisplayChar('2',menu_seq[1]);
	MAX7219_DisplayChar('3',menu_seq[2]);
	MAX7219_DisplayChar('4',menu_seq[3]);
}

void MusiqueAlerte(void){
	for(int i = 0; i < 5; i++) { // Augmenter le i pour un effet continu
		buzzerFunction(DO,  500,  50); // Note basse
		buzzerFunction(SI,  500,  50); // Note haute
	}
}

//Dans la fonction ADC on viendra selon la valeur de notre ADC incrémenter la valeur
//de notre moteur ainsi qu'allumer des leds et changer l'écran de notre afficheur
//pour préciser les vitesses en cours
void adcFunction(){
	HAL_ADC_Start_IT(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	analogValue = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop_IT(&hadc);

	printf("ADC value = %lu\r\n",analogValue);
	//HAL_Delay(1000);
	if(analogValue>500){
		HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_SET);
		TIM3->CCR1+=1500;	//On rentre 1500 dans notre registre pour le démarrage du
		MAX7219_Clear();	//moteur, il démarre à moins mais ce n'est pas optimal
		MAX7219_DisplayChar('1', 'S'); 	//pour notre utilisation
		MAX7219_DisplayChar('2', 'P');
		MAX7219_DisplayChar('3', '.');
		MAX7219_DisplayChar('4', '1');
	}
		else{
			HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_RESET);
		}
	if(analogValue>1000){
		HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_SET);
		TIM3->CCR1+=150;
		MAX7219_Clear();
		MAX7219_DisplayChar('1', 'S');
		MAX7219_DisplayChar('2', 'P');
		MAX7219_DisplayChar('3', '.');
		MAX7219_DisplayChar('4', '2');

	}
		else{
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_RESET);

		}
	if(analogValue>2000){
		HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_SET);
		TIM3->CCR1+=150;
		MAX7219_Clear();
		MAX7219_DisplayChar('1', 'S');
		MAX7219_DisplayChar('2', 'P');
		MAX7219_DisplayChar('3', '.');
		MAX7219_DisplayChar('4', '3');
	}
		else{
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_RESET);

		}
	if(analogValue>3000){
		HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_SET);
		TIM3->CCR1+=150;
		MAX7219_Clear();
		MAX7219_DisplayChar('1', 'S');
		MAX7219_DisplayChar('2', 'P');
		MAX7219_DisplayChar('3', '.');
		MAX7219_DisplayChar('4', '4');

	}
		else{
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_RESET);

		}
	if(analogValue>4000){
		TIM3->CCR1+=500;
		MAX7219_Clear();
		MAX7219_DisplayChar('1', 'S');
		MAX7219_DisplayChar('2', 'P');
		MAX7219_DisplayChar('3', '.');
		MAX7219_DisplayChar('4', '5');

	}

}

//Dans cette fonction on compare la valeur que l'on récupère de notre capteur avec une
//valeur maximale que l'on a fixé pour déclencher le moteur qu'à parti de cette valeur
void check_temperature_motor(float temp){

	uint8_t motorstate = 0;
	float max_temp = 32.00;
	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &temp);
	printf("Temperature %.2f\r\n",temp);
		if(temp>max_temp){
			  motor(!motorstate);
				TIM3->CCR1 = 2000;
		  }else{
			  motor(motorstate);
			  TIM3->CCR1 = 0;
		  }
}

//Ici on fait basqiquement la même chose avec le buzzer sauf qu'on précise deux valeurs
//pour éviter que le buzzer continue d'être actif quand le moteur se lance
void check_temperature(float temp){

	float temp_motor =32.00;
	float max_temp = 30.00;
	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &temp);
	if(temp>max_temp && temp<= temp_motor){
		buzzer(!buzzerstate);
		MusiqueAlerte();

	}else if(temp<max_temp){
		buzzer(buzzerstate);

	}
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
  MX_TIM3_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  MAX7219_Init();				//Initialisation des fonctions de début et on vérifie
  MAX7219_DisplayTestStart();	//que notre afficheur fonctionne bien
  HAL_Delay(2000);
  MAX7219_DisplayTestStop();
  conf_capteur();
  affichage_menu();
  MusiqueAmbiance();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(flag_temp == 1){		//Quand le flag est à 1 on attend une seconde puis on
		  HAL_Delay(1000);		//affiche notre température, on récupère la valeur du
		  display_temp();		//flag dans le fichier d'IT
		  flag_temp=0;
	  }

	  if(flag_hum == 1){		//Pareil avec l'humidité
		  HAL_Delay(1000);
		  display_hum();
		  flag_hum=0;
	  }

	  if(flag_pres == 1){		//Pareil avec la pression
		  HAL_Delay(1000);
		  display_pression();
		  flag_pres=0;
	  }

	  check_temperature(Temperature);	//On check la température en continue
	  adcFunction();
	  check_temperature_motor(Temperature);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 500;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2278;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1139;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L0_Pin|L1_Pin|L2_Pin|L3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin BTN4_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = B1_Pin|BTN4_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : L0_Pin L1_Pin L2_Pin L3_Pin */
  GPIO_InitStruct.Pin = L0_Pin|L1_Pin|L2_Pin|L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
	return(ch);
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
