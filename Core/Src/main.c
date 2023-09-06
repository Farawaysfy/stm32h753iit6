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

IWDG_HandleTypeDef hiwdg1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint8_t machine_sta = 0;//machine status
static uint8_t plane_sta=1,spray_sta=1;
static uint8_t pul_sta=1,dir_sta=1,ena_sta=1;
static uint32_t set_speed = 10;//设定吸附轮转速为set_speed，单位r/min，减速机减速比为10， 电机输出轴转速为set_speed * 10
static uint32_t delay_period = 10000-1; //定时器延时周期数，一个周期0.1us

static uint8_t Gate1_seed_num = 0, Gate2_seed_num = 0;//seed number of each gate
static uint32_t receive_seed_num = 0b0;
static uint8_t receive_usart1[USART_REC_LEN_1];
static uint8_t receive_usart3[USART_REC_LEN_3];

static uint8_t LEDRED_Status= 0, LEDGREEN_Status = 0;

////FDCAN_TxHeaderTypeDef TxHeader;
//uint8_t *TxData;
//
////FDCAN_RxHeaderTypeDef RxHeader;
//uint8_t *RxData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void reload_tim1(uint32_t Period);//设置重装载的值，重新设置定时器时间

void motor_init();

uint32_t speed2Period(uint32_t speed);//speed r/min

void dif_fluoresent_seed(uint8_t num_Light_Gate2);

//uint8_t FDCAN1_Send_Msg(uint8_t * msg,uint32_t len);
//uint8_t FDCAN1_Receive_Msg(uint8_t *buf);
uint8_t Global_Status_set(uint8_t command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define Plane(n) (n ? HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET)) // plane PG1(挡板接口)

#define Spray_valve(n) (n ? HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_RESET)) // Spray valve PG0(喷阀接口)

#define Light_Gate1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) //Light_Gate1  PA15（光电门1信号）
#define Light_Gate2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) //Light_Gate2  PB3（光电门2信号）

#define PUL_minus(n) (n ? HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET)) // Pulse signal PE5（电机脉冲信号）
#define overturn_PUL HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5) // 翻转电机脉冲信号
//#define DIR_minus(n) (n ? HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET): \
//                HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET)) // Direction signal PE6, unuseful（正反转，弃用）
#define ENA_minus(n) (n ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)) // Enable signal PB12（电机启停接口)
#define overturn_ENA HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_12) // 翻转电机启停接口

#define led1 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5) // LED1 PE5  通信指示灯
#define led2 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6) // LED2 PE6 光电门1指示灯
#define led3 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12) // LED3 PB12 光电门2指示灯
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
  MX_IWDG1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

    motor_init();
//    printf("Motor control, pul_sta = %d, ena_sta = %d\r\n",pul_sta,ena_sta);//实验
    HAL_UART_Transmit(&huart3, (uint8_t *)"\nMotor control, pul_sta = ", 26, 0xFFFF);
    HAL_UART_Transmit(&huart3, (uint8_t *)&pul_sta, 1, 0xFFFF);
    HAL_UART_Transmit(&huart3, (uint8_t *)", ena_sta = ", 12, 0xFFFF);
    HAL_UART_Transmit(&huart3, (uint8_t *)&ena_sta, 1, 0xFFFF);
    HAL_UART_Transmit(&huart3, (uint8_t *)"\n", 1, 0xFFFF);
//    HAL_UART_Receive_IT(&huart3, (uint8_t *)receive_usart3, USART_REC_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_IWDG_Refresh(&hiwdg1);
//      HAL_UART_Receive_IT(&huart3, (uint8_t *)receive_usart3, USART_REC_LEN);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
//步进电机控制定时器
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

    HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
//喷阀启动定时器
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 299;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

    HAL_TIM_Base_Start_IT(&htim3);  //开启定时器3中断
  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN_1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

    HAL_UART_Receive_IT(&huart3, (uint8_t *)receive_usart3, USART_REC_LEN_3);
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE5 PE6 PE13 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_15: {//light gate1 detect
            if (Light_Gate1 == 0) {
//                printf("No. %d passed! \r\n",Gate1_seed_num); //重定向仅作实验用
                HAL_UART_Transmit(&huart3, (uint8_t *)"Gate1 Number ", 13, 0xFFFF);
                HAL_UART_Transmit(&huart3, (uint8_t *)(&Gate1_seed_num), 1, 0xFFFF);
                HAL_UART_Transmit(&huart3, (uint8_t *)"seed passed!\n", 13, 0xFFFF);
                ++Gate1_seed_num;
                if(Gate1_seed_num > 32)
                {
                    Gate1_seed_num = 1;
                }
            }
            break;
        }case GPIO_PIN_3: {//light gate2 detect
            if (Light_Gate2 == 0) {
//                printf("No. %d passed\r\n",Gate2_seed_num); //重定向仅作实验用
                HAL_UART_Transmit(&huart3, (uint8_t *)"\nGate2 Number ", 14, 0xFFFF);
                HAL_UART_Transmit(&huart3, (uint8_t *)(&Gate1_seed_num), 1, 0xFFFF);
                HAL_UART_Transmit(&huart3, (uint8_t *)"seed passed!\n", 13, 0xFFFF);
                led3;//光电门2指示灯
                if((Gate2_seed_num > Gate1_seed_num) & (receive_seed_num>>Gate2_seed_num)) //判断是否为同意轮次的编号
                {
//                    printf("Error, Gate2_seed_num > Gate1_seed_num\r\n");//重定向仅作实验用
                    HAL_UART_Transmit(&huart3, (uint8_t *)"Error, Gate2_seed_num > Gate1_seed_num\n", 39, 0xFFFF);
                    break;
                }
                dif_fluoresent_seed(Gate2_seed_num);
                ++Gate2_seed_num;
                if(Gate2_seed_num > 32)
                {
                    Gate2_seed_num = 1;
                    receive_seed_num=0b0;
                }
            }
            break;
        }
//        case GPIO_PIN_13: {//key2
//            if (KEY2 == 0) {// start/pause the motor,disable
//                printf("key2 is touched, motor status changes\r\n");
//                ena_sta = !ena_sta;
//                ENA_minus(ena_sta);
//                LED0RED(ena_sta);
//                LED1GREEN(!ena_sta);
//                printf("ena_sta is %d\r\n",ena_sta);
//            }
//            break;
//        }//key2调试使用
        default:
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance==USART1)//如果是串口1//串口1接收中断，接收来自的信息格式：从左往右共三位，第一、二位为荧光种子编号，第三位为回车
    {
        if(receive_usart1[2] == 0xd)//如果接收到的数据是回车
        {
            if( receive_usart1[0]>='0' && receive_usart1[0]<='9' &&receive_usart1[1]>= '0' && receive_usart1[1]<='9')
            {
                receive_usart1[2] = '\0'; 
                uint8_t temp = (receive_usart1[0]-'0')*10 + (receive_usart1[1]-'0');
//                printf("receive_usart3 = %s\r\n",receive_usart3);//重定向仅作实验用
                receive_seed_num=receive_seed_num|(0b1<<(temp-1));
                led2;//与TX2连接的LED
//                reload_tim3(speed2Period(set_speed));//设置脉冲信号频率，不能在此处设置，可能定时器处于工作中期
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)receive_usart1, USART_REC_LEN_1);
    }
    if(huart->Instance==USART3)//如果是串口3//串口3接收中断，接收来自工控屏的信息格式：从左往右共三位，第一位：状态（0：启动，1：暂停，2：终止），第二、三位为吸附轮速度，第四位为回车
    {
        if(receive_usart3[3] == 0xd)//如果接收到的数据是回车
        {
            if(Global_Status_set(receive_usart3[0] != '0'))
            {
//                printf("Status stop/pause!");//重定向仅作实验用
                HAL_UART_Transmit(&huart3, (uint8_t *)"\nStatus stop/pause!\n", 20, 0xFFFF);
            }
            else
            if( receive_usart3[1]>='0' && receive_usart3[1]<='9' &&receive_usart3[2]>= '0' && receive_usart3[2]<='9')
            {
                receive_usart3[3] = '\0';
//                printf("receive_usart3 = %s\r\n",receive_usart3);//重定向仅作实验用
                HAL_UART_Transmit(&huart3, (uint8_t *)"\nreceive_usart3 = ", 18, 0xFFFF);
                HAL_UART_Transmit(&huart3, (uint8_t *)receive_usart3, 4, 0xFFFF);
                HAL_UART_Transmit(&huart3, (uint8_t *)"\n", 1, 0xFFFF);
                set_speed = (receive_usart3[0]-'0')*10 + (receive_usart3[1]-'0');
                led1;//与上位机通信指示灯
//                reload_tim3(speed2Period(set_speed));//设置脉冲信号频率，不能在此处设置，可能定时器处于工作中期
            }
        }
        HAL_UART_Receive_IT(&huart3, (uint8_t*)receive_usart3, USART_REC_LEN_3);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//脉冲信号定时器中断
{
    if(htim==(&htim3) && ena_sta == 1)//送出脉冲信号
    {
          overturn_PUL;
//        PUL_minus(pul_sta);
//        pul_sta = !pul_sta;
        reload_tim1(speed2Period(set_speed));//周期结束重设定时器
    }
}

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//    if(hfdcan == &hfdcan1) {//如果是CAN1,接收上位机发送的荧光种子代码，格式：从左往右数共三位，第一、二位：荧光种子代码，第三位为回车
//        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
//            Error_Handler();
//        }
//        uint8_t buf[8];
//        uint8_t len;
//        len = FDCAN1_Receive_Msg(buf);
//        if (len) {
//            printf("receive data from can success\r\n");
//            uint8_t temp_num;
//            if (buf[2] == 0xd && buf[0] >= '0' && buf[0] <= '9'
//                && buf[1] >= '0' && buf[1] <= '9')//如果接收到的数据是回车
//            {
//                buf[2] = '\0';
//                printf("receive_can = %s\r\n", buf);
//                temp_num = (buf[0] - '0') * 10 + (buf[1] - '0');
//                if (temp_num <= 32 && temp_num > 0) {
//                    receive_seed_num |= (0b1 << (temp_num - 1));
//                    printf("receive_seed_num = %lu\r\n", receive_seed_num);
//                }
//            }
//        }
//    }
//}

uint32_t speed2Period(uint32_t speed)//speed 1r/min == 1/60000r/ms, 1ms = 1000 us
{
    if(speed==0)
    {
        return 1000 - 1;
    }
    uint16_t step = 6400; //check figure, current status is sw5 off, sw6 on, sw7 off, sw8 on.
    delay_period =(uint32_t)(10e7 * 60.0 / (step * speed * 10.0) + 0.5);//0.1us为一个单位
    if(delay_period < 31) //最短周期为3us
        delay_period = 31;
    return delay_period - 1;
}//速度转为脉冲信号周期

void motor_init()
{
    PUL_minus(pul_sta);
    ENA_minus(ena_sta);
}

void reload_tim1(uint32_t Period)//Tout溢出时间us = (Prescaler+1) / Tlck *10000000us * (Period+1); Tlck= 200Mhz
{
    htim1.Init.Period = Period;//周期数，计时周期数+1，即多少1us = （199+1）/200Mhz * 1000000000us
}

void dif_fluoresent_seed(uint8_t num_Light_Gate2)
{
    uint8_t temp = 0b1;
    temp = temp<<num_Light_Gate2;
    if(receive_seed_num&temp)
    {
        spray_sta = 1;
        Spray_valve(spray_sta);
        printf("Match successfully!\r\n");//use Spray valve here
    }
    else
    {
        printf("Fail to match seed!\r\n");
        spray_sta = 0;
    }
}

//uint8_t FDCAN1_Send_Msg(uint8_t * msg,uint32_t len)
//{
//    TxHeader.Identifier=0x12;                           //32位ID
//    TxHeader.IdType=FDCAN_STANDARD_ID;                  //标准ID
//    TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
//    TxHeader.DataLength=len;                            //数据长度
//    TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
//    TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
//    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
//    TxHeader.MessageMarker=0;
//
//    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,msg)!=HAL_OK) return 1;//发送
//    return 0;
//}
//uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
//{
//    if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,buf)!=HAL_OK)
//        return 0;//接收数据
//    return RxHeader.DataLength>>16;
//}

uint8_t Global_Status_set(uint8_t command)
{
    switch (command) {
        case 0:{
            ENA_minus(1);
        }
        case 1:{
            ENA_minus(0);
        }
        case 2:{
            if(Gate1_seed_num && Gate2_seed_num == 0)//if Gate1 and Gate2 are empty for a while, then stop the motor
                ENA_minus(0);
        }
    }
    return command;
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
