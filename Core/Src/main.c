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

FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t machine_sta = 0;//machine status
static uint8_t plane_sta=1,spray_sta=1;
static uint8_t pul_sta=1,dir_sta=1,ena_sta=1;
static uint32_t set_speed = 100;//set speed is 100 actually 70r/min motor output without reducer
static uint32_t delay_time = 1000000;

static uint8_t Gate1_seed_num = 0, Gate2_seed_num = 0;//seed number of each gate
static uint32_t receive_seed_num = 0b0;
static uint8_t receive_usart1[USART_REC_LEN];

static uint8_t LEDRED_Status= 0, LEDGREEN_Status = 0;

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t *TxData;

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t *RxData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_IWDG1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void reload_tim1(uint32_t Period);//������װ�ص�ֵ���������ö�ʱ��ʱ��

void motor_init();

uint32_t speed2Period(uint32_t speed);//speed r/min

void dif_fluoresent_seed(uint8_t num_Light_Gate2);

uint8_t FDCAN1_Send_Msg(uint8_t * msg,uint32_t len);
uint8_t FDCAN1_Receive_Msg(uint8_t *buf);
uint8_t Global_Status_set(uint8_t command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define Plane(n) (n ? HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET)) // plane PG1(����ӿ�)

#define Spray_valve(n) (n ? HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_RESET)) // Spray valve PG0(�緧�ӿ�)

#define Light_Gate1 HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3) //Light_Gate1  PG3�������1�źţ�
#define Light_Gate2 HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_4) //Light_Gate2  PG4�������2�źţ�

#define PUL_minus(n) (n ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)) // Pulse signal PB0����������źţ�
//#define DIR_minus(n) (n ? HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET): \
//                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET)) // Direction signal PF14, unuseful������ת�����ã�
#define ENA_minus(n) (n ? HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET)) // Enable signal PF13�������ͣ�ӿڣ�
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
  MX_FDCAN1_Init();
  MX_IWDG1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    motor_init();
    printf("Motor control, pul_sta = %d, ena_sta = %d\r\n",pul_sta,ena_sta);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_IWDG_Refresh(&hiwdg1);
      HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
    FDCAN_FilterTypeDef hfdcan1_filter;
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
    hfdcan1_filter.IdType = FDCAN_STANDARD_ID;
    hfdcan1_filter.FilterIndex = 0;
    hfdcan1_filter.FilterType = FDCAN_FILTER_MASK;
    hfdcan1_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    hfdcan1_filter.FilterID1 = 0x0000;
    hfdcan1_filter.FilterID2 = 0x0000;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &hfdcan1_filter) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* USER CODE END FDCAN1_Init 2 */

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
    HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN);
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
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

  /*Configure GPIO pins : PE13 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PG4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_6: {//light gate1 detect
            if (Light_Gate1 == 0) {
                printf("No. %d passed! \r\n",Gate1_seed_num);
                ++Gate1_seed_num;
                if(Gate1_seed_num > 32)
                {
                    Gate1_seed_num = 1;
                }
            }
            break;
        }case GPIO_PIN_7: {//light gate2 detect
            if (Light_Gate2 == 0) {
                printf("No. %d passed\r\n",Gate2_seed_num);
                if(Gate2_seed_num > Gate1_seed_num){
                    printf("Error, Gate2_seed_num > Gate1_seed_num\r\n");
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
//        }//key2����ʹ��
        default:
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance==USART1)//����Ǵ���1//����1�����жϣ��������Թ���������Ϣ��ʽ���������ҹ���λ����һλ��״̬��0��������1����ͣ��2����ֹ�����ڶ�����λΪ�������ٶȣ�����λΪ�س�
    {
        if(receive_usart1[3] == 0xd)//������յ��������ǻس�
        {
            if(Global_Status_set(receive_usart1[0] != '0'))
            {
                printf("Status stop/pause!");
            }
            else
            if( receive_usart1[1]>='0' && receive_usart1[1]<='9' &&receive_usart1[2]>= '0' && receive_usart1[2]<='9')
            {
                receive_usart1[3] = '\0';
                printf("receive_usart1 = %s\r\n",receive_usart1);
                set_speed = (receive_usart1[0]-'0')*10 + (receive_usart1[1]-'0');
//                reload_tim3(speed2Period(set_speed));//���������ź�Ƶ�ʣ������ڴ˴����ã����ܶ�ʱ�����ڹ�������
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)receive_usart1, USART_REC_LEN);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//�����źŶ�ʱ���ж�
{
    if(htim==(&htim1) && ena_sta == 1)//�ͳ������ź�
    {
        PUL_minus(pul_sta);
        pul_sta = !pul_sta;
        reload_tim1(speed2Period(set_speed));//���ڽ������趨ʱ��
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1) {//�����CAN1,������λ�����͵�ӫ�����Ӵ��룬��ʽ����������������λ����һ����λ��ӫ�����Ӵ��룬����λΪ�س�
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
            Error_Handler();
        }
        uint8_t buf[8];
        uint8_t len;
        len = FDCAN1_Receive_Msg(buf);
        if (len) {
            printf("receive data from can success\r\n");
            uint8_t temp_num;
            if (buf[2] == 0xd && buf[0] >= '0' && buf[0] <= '9'
                && buf[1] >= '0' && buf[1] <= '9')//������յ��������ǻس�
            {
                buf[2] = '\0';
                printf("receive_can = %s\r\n", buf);
                temp_num = (buf[0] - '0') * 10 + (buf[1] - '0');
                if (temp_num <= 32 && temp_num > 0) {
                    receive_seed_num |= (0b1 << (temp_num - 1));
                    printf("receive_seed_num = %lu\r\n", receive_seed_num);
                }
            }
        }
    }
}

uint32_t speed2Period(uint32_t speed)//speed 1r/min == 1/60000r/ms, 1ms = 1000 us
{
    if(speed==0)
    {
        return 1000000;
    }
    uint16_t step = 6400; //check figure, current status is sw5 off, sw6 on, sw7 off, sw8 on.
    delay_time =(uint32_t)(500.0 * 60000.0/(step * speed) + 0.5);//1usΪһ����λ
    if(delay_time<3)
        delay_time = 3;
    return delay_time - 1;
}//�ٶ�תΪ�����ź�����

void motor_init()
{
    PUL_minus(pul_sta);
    ENA_minus(ena_sta);
}

void reload_tim1(uint32_t Period)//Tout���ʱ��us = (Prescaler+1) / Tlck *10000000us * (Period+1); Tlck= 200Mhz
{
    htim1.Init.Period = Period;//����������ʱ������+1��������1us = ��199+1��/200Mhz * 1000000000us
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

uint8_t FDCAN1_Send_Msg(uint8_t * msg,uint32_t len)
{
    TxHeader.Identifier=0x12;                           //32λID
    TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    TxHeader.DataLength=len;                            //���ݳ���
    TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    TxHeader.MessageMarker=0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,msg)!=HAL_OK) return 1;//����
    return 0;
}
uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{
    if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,buf)!=HAL_OK)
        return 0;//��������
    return RxHeader.DataLength>>16;
}

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
