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
#define NE 0
#define NH 1
#define NM 2
#define NS 3
#define NT 4
#define Z  5
#define PT 6
#define PS 7
#define PM 8
#define PH 9
#define PE 10

#define Extreme_Duty 55
#define High_Duty 35
#define Medium_DUty 20
#define Small_Duty 10
#define Tiny_Duty 1
#define Zero_Duty  0
int adc_96V_ref = 2204;
int adc_error = 0;
int adc_prevError =0;


int fuzzy_calculator(int adcvalue)
{
int fuzzy_value=0;
if(adcvalue>500)
{
fuzzy_value = PE;
} 
else if((adcvalue>350) && (adcvalue<=500))
{
fuzzy_value = PH;
}
else if((adcvalue>180) && (adcvalue<=350))
{
fuzzy_value = PM;
}
else if((adcvalue>80) && (adcvalue<=180))
{
fuzzy_value = PS;
}
else if((adcvalue>5) && (adcvalue<=80))
{
fuzzy_value = PT;
}
else if((adcvalue>-5) && (adcvalue<=5))
{
fuzzy_value = Z;
}
else if((adcvalue>-80) && (adcvalue<=-5))
{
fuzzy_value = NT;
}
else if((adcvalue>-180) && (adcvalue<=-80))
{
fuzzy_value = NS;
}
else if((adcvalue>-350) && (adcvalue<=-180))
{
fuzzy_value = NM;
}
else if((adcvalue>-500) && (adcvalue<=-350))
{
fuzzy_value = NH;
}
else if((adcvalue<=-500))
{
fuzzy_value = NE;
}
return fuzzy_value;
}



 int FuzzyRules[11][11] = {
        {Z,  NT, NS, NM, NM, NH, NH, NE, NE, NE, NE},
        {PT, Z,  NT, NS, NM, NM, NH, NH, NE, NE, NE},
        {PS, PT, Z,  NT, NS, NM, NM, NH, NH, NE, NE},
        {PM, PS, PT, Z,  NT, NS, NM, NM, NH, NH, NE},
        {PM, PM, PS, PT, Z,  NT, NS, NM, NM, NH, NH},
        {PH, PM, PM, PS, PT, Z,  NT, NS, NM, NM, NH},
        {PH, PH, PM, PM, PS, PT, Z,  NT, NS, NM, NM},
        {PE, PH, PH, PM, PM, PS, PT, Z,  NT, NS, NM},
        {PE, PE, PH, PH, PM, PM, PS, PT, Z,  NT, NS},
        {PE, PE, PE, PH, PH, PM, PM, PS, PT, Z,  NT},
        {PE, PE, PE, PE, PH, PH, PM, PM, PS, PT, Z}
    };
int return_Output(int error,int errorChange)
{
int out_put = (int)FuzzyRules[error][10-errorChange];

return out_put;
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile int adc_value = 0;
 volatile float adc_voltage = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
uint32_t pwm_Val=0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint32_t get_adc_voltage()
{

adc_value=0;
    for(int i = 0; i<5; i++)
    {
      	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, 1);
    	   adc_value +=  HAL_ADC_GetValue(&hadc2);
    }
adc_value = adc_value/5;

adc_voltage = ((float)adc_value) * (3300.0f / 4095.0f);
return (uint32_t)adc_voltage;
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
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
   int changeError = 0;
   volatile int err;
volatile int changee ;
volatile int output = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {

	  
  adc_error = adc_96V_ref-get_adc_voltage();
   changeError = adc_error - adc_prevError;
  err = fuzzy_calculator(adc_error);
  changee =  fuzzy_calculator(changeError);
  
  output = return_Output(err,changee);
switch(output) {
        case NE:
            if(pwm_Val>=50)
            {
              pwm_Val = pwm_Val -50;
            }
            break;
        case NH:
            if(pwm_Val>=38)
            {
              pwm_Val = pwm_Val -38;
            }
            break;
        case NM:
            if(pwm_Val>=25)
            {
              pwm_Val = pwm_Val -25;
            }
            break;
        case NS:
            if(pwm_Val>=12)
            {
              pwm_Val = pwm_Val -12;
            }
            break;
        case NT:
            if(pwm_Val>=5)
            {
              pwm_Val = pwm_Val -5;
            }
            break;
        case Z:
            pwm_Val = pwm_Val;
            break;
        case PT:
            if(pwm_Val<=994)
            {
              pwm_Val = pwm_Val + 5;
            }
            break;
        case PS:
            if(pwm_Val<=987)
            {
              pwm_Val = pwm_Val + 12;
            }
            break;
        case PM:
            if(pwm_Val<=974)
            {
              pwm_Val = pwm_Val + 25;
            }
            break;
        case PH:
            if(pwm_Val<=961)
            {
              pwm_Val = pwm_Val + 38;
            }
            break;
        case PE:
            if(pwm_Val<=949)
            {
              pwm_Val = pwm_Val + 50;
            }
            break;
        default:
        
            break;
    }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_Val);
  adc_prevError = adc_error;

 // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 350);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
