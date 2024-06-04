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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//-------------------------------------------//
// Declaracao de variaveis para a leitura ADC//
//-------------------------------------------//
float pot_malha_aberta = 0.0f;
float pot_corrente = 0.0f;
float pot_tensao = 0.0f;
float corrente = 0.0f;
float tensao = 0.0f;
uint16_t ref_pwm = 0;

float ganho_pot_malha_aberta = 0.439560439f;//1800/4095
float ganho_pot_corrente = 0.0001663f; // 0.681A/4095
float ganho_pot_tensao = 3.663e-3f; // 15V/4095
float ganho_corrente = 1.25031e-3f; // 5.12A/4095
float ganho_tensao = 9.76801e-3f; // 40V/4095

float pot_adc = 0.0f;
float corrente_adc = 0.0f;
float tensao_adc = 0.0f;

volatile uint8_t FLAG_READY=0;

volatile uint16_t ADC_buffer[ADC_BUF_SIZE];

//-------Variaveis para comunicacao serial--//
uint16_t SERIAL_DELAY = 1000;
uint8_t SERIAL_TIMES = 0;
//------------------------------------------//


//--------------------------------------------//
// Declaracao variaveis controle              //
//-------------------------------------------//
static float uik0, uik1;
static float eik0, eik1;

static float uvk0, uvk1;
static float evk0, evk1;

static float iL, iLref, Vo, Voref;


//Ganhos do compensador de corrente
const float ai1 = 3.64182649500560e+003;
const float ai2 = -3.41997350499440e+003f;

//Ganhos do compensador de tensao
const float av1 = 44.8243789261330e-003f;
const float av2 = -44.5436210738670e-003f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //Calibracao do ADC
  HAL_ADCEx_Calibration_Start(&hadc1);

  //Inicializa o sinal pwm e a interrupcao pelo timer3
  HAL_TIM_Base_Start_IT(&htim3); // inicializa a interrupção pelo timer 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // inicializa o PWM


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_5_Pin|LED3_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_5_Pin LED3_0_Pin */
  GPIO_InitStruct.Pin = LED1_5_Pin|LED3_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//----------------------------------------------//
//    Funcao que executa a leitura dos ADCs     //
//----------------------------------------------//
void read_ADC()
{

	// Ler ADCs
	FLAG_READY = 0; // Zerar a FLAG de "leitura dos adcs"


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_buffer, ADC_BUF_SIZE); // Dispara a leitura da leitura do ADC
//	while(FLAG_READY == 0); // Espera a flag do "leitura dos adcs" ser 1 (leitura realizada)

	ref_pwm = ADC_buffer[0];
	pot_adc = ADC_buffer[0];
	corrente_adc = ADC_buffer[1];
	tensao_adc = ADC_buffer[2];

	pot_malha_aberta = pot_adc*ganho_pot_malha_aberta; //multiplica o sinal lido pelo pino adc do potenciometro pelo seu ganho para utilizar em malha aberta
	pot_corrente = pot_adc*ganho_pot_corrente; //multiplica o sinal lido pelo pino adc do potenciometro pelo seu ganho para utilizar como referencia de corrente
	pot_tensao = pot_adc*ganho_pot_tensao; //multiplica o sinal lido pelo pino adc do potenciometro pelo seu ganho para utilizar como referencia de tensao
	corrente  = corrente_adc*ganho_corrente; //multiplica o sinal lido pelo pino adc da corrente pelo seu ganho definido no inicio do código
	tensao  = tensao_adc*ganho_tensao; //multiplica o sinal lido pelo pino adc da tensao pelo seu ganho definido no inicio do código

}

// Funcao de debug atraves do protocolo usart para leitura na serial dos valores dos adcs
void print_MSG()
{

	char mensagemUart[500];
	sprintf(mensagemUart, "%4.2f %4.2f %4.2f \r\n", pot_malha_aberta, corrente, tensao);
	HAL_UART_Transmit(&huart1, (const uint8_t*) mensagemUart, strlen(mensagemUart), 1000);
}

//Função para printar uma variável qualquer na serial como debug atraves do protocolo usart
void debug_msg(float mensagem)
{
	char mensagemUart[500];
	sprintf(mensagemUart, "%4.2f \r\n", mensagem);
	HAL_UART_Transmit(&huart1, (const uint8_t*) mensagemUart, strlen(mensagemUart), 1000);
}

//
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  //HAL_ADC_Start_IT(&hadc1);
  FLAG_READY = 1;
}


//Funcao utilizada para controlar o pwm
void modulacao(float ref){
	TIM3->CCR2 = ref; //Atualiza o registrador do timer 3 que define o duty cycle.
	                  //o valor maximo do pwm configurado é 3599, d = ref/3599.
}

void malha_aberta(){
	uint16_t aux2 = 1080;
	if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_11)){aux2 = 1800;}
	modulacao(aux2);
	//modulacao(pot_malha_aberta); //Referencia vira o potenciometro
}

void controle_malha_fechada(){


	/*

	 A declaracao das variaveis de controle foi feita no inicio
	 do codigo de forma a economizar tempo de processamento dentro
	 desta funcao

	 */

	//--------------------------------------------//
	// Malha de tensao                            //
	//--------------------------------------------//
	// Referencia de tensao                       //
	//-------------------------------------------//
	Voref = 15.0f;
	//Voref = pot_tensao;//A referencia vira o potenciometro


    //if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_11)){Voref = 15.0f;}//aumenta a ref de tensao quando aperta-se o botao presente na placa


	//--------------------------------------------//
	// Conversao AD                               //
	//--------------------------------------------//
	// Leitura da tensao
	Vo = tensao;

	//--------------------------------------------//
	// Calculo do erro                            //
	//--------------------------------------------//
	evk0 = Voref - Vo;

	//debug_msg(evk0);

	//--------------------------------------------//
	// Atualizaçao da açao de controle      //
	//-------------------------------------------//
	uvk0 = uvk1 + (av1*evk0) + (av2*evk1);

	//--------------------------------------------//
	// Rotaçao das variaveis de tensao            //
	//--------------------------------------------//
	evk1 = evk0;
	uvk1 = uvk0;


	//--------------------------------------------//
	// Malha de corrente                          //
	//--------------------------------------------//
	// Referencia de corrente                    //
	//-------------------------------------------//
	iLref = uvk0;
	//iLref = 0.340f;
	//iLref = pot_corrente; // A referencia vira o potenciometro
	//if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_11)){iLref = 0.680f;} //aumenta a referencia de corrente quando aperta-se o botao presente na placa

	//--------------------------------------------//
	// Conversao AD                                   //
	//--------------------------------------------//
	// Leitura da corrente
	iL = corrente;
	//--------------------------------------------//
	// Calculo do erro                                 //
	//--------------------------------------------//
	eik0 = iLref - iL;
    //debug_msg(eik0);

	//--------------------------------------------//
	// Atualizaçao da açao de controle      //
	//-------------------------------------------//
	uik0 = uik1 + (ai1*eik0) + (ai2*eik1);

	//Limitar a acao de controle de corrente por seguranca
	//if(uik0 > 2520.0f){uik0=2520.0f;}
	//if(uik0 < 10.0f){uik0=10.0f;}

	//--------------------------------------------//
	// Rotaçao das variaveis de corrent           //
	//--------------------------------------------//
	eik1 = eik0;
	uik1 = uik0;

	modulacao(uik0);
}

// AQUI EH A FUNCAO CHAMADA NA INTERRUPCAO DO TIMER3
//     ||||||||||
//     VVVVVVVVVV

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);


	//---------------------------//
	//Leitura dos ADCs e controle//
	//---------------------------//
    read_ADC(); // Funcao que executa a leitura dos adcs
    malha_aberta(); // Funcao que faz o conversor funcionar em malha aberta
    //controle_malha_fechada(); // Funcao que faz o conversor funcionar em malha fechada
    //---------------------------//


    //Funcoes que permites o debug atraves da porta serial//
    //debug_msg(pot);
    //print_MSG();
    //----------------------------------------------------//

    // Uma pista visual que o botao foi acionado e que que a referencia do controle foi alterada//
    if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_11))
    {
        // Aciona o LED quando o botao da placa de instrumentacao é acionado
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else
    {
        // Desliga o LED quando o botao da instrumentacao é desacionado
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    //-----------------------------------------------------------------------//

    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
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
	  /*
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  HAL_Delay(500);*/
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
