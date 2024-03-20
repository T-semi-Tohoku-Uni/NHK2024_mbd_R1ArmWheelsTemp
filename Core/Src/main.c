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

#include "stdio.h"
#include "string.h"

#include "R1CANIDList.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WHEEL_DIAMETER 100 //[mm]
#define WHEELBASE_LEN 300 //[mm]
#define TREAD_LEN 200//[mm]
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
FDCAN_FilterTypeDef FDCAN1_sFilterConfig;

FDCAN_TxHeaderTypeDef FDCAN3_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN3_RxHeader;
FDCAN_FilterTypeDef FDCAN3_sFilterConfig;

uint8_t FDCAN1_TxData[4] = {0};
uint8_t FDCAN1_RxData[4] = {0};
uint32_t FDCAN1_TxMailbox;


uint8_t FDCAN3_RxData[8] = {0};
uint32_t FDCAN3_TxMailbox;

int goal_list[3][4] = {{2500,1500,2500,1400},
                       {2000,1500,2500,2000},
                       {2500,2000,2000,1400}};

int move[4][4] = {{10,10,10,10},
                  {-500,500,500,-500},
                  {500,500,500,500},
                  {-500,-500,-500,-500}};


struct pid_data{
    double t;
    double i[4];
    double error[4];
    double pre_error[4];
    int u[4];
    int goal[4];
};

typedef struct{
	float actPos[3];
	float trgPos[3];
	float outPos[3];
	float actVel[3];
	float trgVel[3];
	float outVel[3];
} robotPosStatus;

typedef struct{
	uint16_t CANID;
	uint8_t motorID;
	float trgVel;
	float actVel;
	float outVel;
	float angle;//[rad]
}motor;

robotPosStatus gRobotPos;
motor gMotor[4];

struct pid_data pid;
int flag = 3;

double p_gain[4] = {4.5,4.5,4.5,4.5};
double i_gain[4] = {0.5,0.5,0.5,0.5};
double d_gain[4] = {0.5,0.5,0.5,0.5};

uint16_t adc[4] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void InverseKinematics(robotPosStatus *robotPos, motor wheelMotor[]){
	//座標変換行�??

	float wheelParam = (WHEELBASE_LEN + TREAD_LEN) / 2;
	uint8_t gain = 6;
	const float A[4][3] = {
			{ 200, -200, wheelParam},
			{ 200,  200, wheelParam},
			{-200,  200, wheelParam},
			{-200, -200, wheelParam}
	};

	for(uint8_t i=0; i<4; i++){
		wheelMotor[i].trgVel = 0;

		for(uint8_t j=0; j<3; j++){
			wheelMotor[i].trgVel += gain * A[i][j] * robotPos->trgVel[j] / WHEEL_DIAMETER;
		}
		//printf("%d:%f\r\n", i, wheelMotor[i].trgVel);
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&hlpuart1,(uint8_t *)ptr,len,10);
    return len;
}


int clamp(int value){
    if(18000 < value)
        return 18000;
    else if(value < -18000)
        return -18000;
    return value;
}

void pid_cul(){
    double p[4] = {0};
    double d[4] = {0};

    for(int k = 0; k < 4; k++) {
        pid.error[k] = adc[k] - pid.goal[k];
        p[k] = p_gain[k] * pid.error[k];
        pid.i[k] += i_gain[k] * (pid.error[k] + pid.pre_error[k]) * pid.t / 2;
        d[k] = d_gain[k] * (pid.error[k] - pid.pre_error[k]) / pid.t;
        pid.pre_error[k] = pid.error[k];
        pid.u[k] = clamp((int) (p[k] + pid.i[k] + d[k]));
    }
    //printf("%d\n\r", adc[3]);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

    if(htim == &htim7) {
        pid_cul();
        memmove(pid.pre_error, pid.error, sizeof(pid.error));
        uint8_t FDCAN3_TxData[8] = {0};
        for (int k = 0; k < 4; k++){
			FDCAN3_TxData[k * 2] = pid.u[k] >> 8;
			FDCAN3_TxData[k * 2 + 1] = pid.u[k];
		}

        FDCAN3_TxHeader.Identifier = 0x200;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &FDCAN3_TxHeader, FDCAN3_TxData) != HAL_OK) {
            Error_Handler();
        }
    }
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {


    if (hfdcan == &hfdcan1){
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, FDCAN1_RxData) != HAL_OK) {
                Error_Handler();
            }

            //Arm
            if (FDCAN1_RxHeader.Identifier == CANID_ARM1) {
            	printf("Arm:%d\r\n", FDCAN1_RxData[0]);
            	//回収
            	if (FDCAN1_RxData[0] == 0x01){
                    for (int k = 0; k < 4; k++){
                        pid.goal[k] = goal_list[0][k];
                        pid.error[k] = adc[k] - pid.goal[k];
                        pid.i[k] = 0;
                    }
                }

            	//外の2つを置く
                if (FDCAN1_RxData[0] == 0x02){
                    for (int k = 0; k < 4; k++){
                        pid.goal[k] = goal_list[1][k];
                        pid.error[k] = adc[k] - pid.goal[k];
                        pid.i[k] = 0;
                    }
                }

                //内の2つを置く
                if (FDCAN1_RxData[0] == 0x03) {
                    for (int k = 0; k < 4; k++){
                        pid.goal[k] = goal_list[2][k];
                        pid.error[k] = adc[k] - pid.goal[k];
                        pid.i[k] = 0;
                    }
                }

            }

            if (FDCAN1_RxHeader.Identifier == 0x10A) {
                for (int k = 0; k < 4; k++) {
                    adc[k] = (FDCAN1_RxData[2*k]<<8)|FDCAN1_RxData[2*k];
                }

                static uint8_t times = 0;
                times++;
                times%50;
                if(times==0)printf("%d\n\r", adc[0]);
            }

            //足回り

            if (FDCAN1_RxHeader.Identifier == CANID_ROBOT_VEL) {
            	for(uint8_t i=0; i<3; i++){
            		gRobotPos.trgVel[i] = (float)(FDCAN1_RxData[i] - 127);
            	}

            	printf("x:%f, y:%f, r:%f\r\n", gRobotPos.trgVel[0], gRobotPos.trgVel[1], gRobotPos.trgVel[2]);
            	InverseKinematics(&gRobotPos, gMotor);

            	uint8_t FDCAN3_TxData[8] = {0};

            	for(uint8_t i=0; i<4; i++){
            		int16_t vel = gMotor[i].trgVel;
            		FDCAN3_TxData[i*2] = vel>>8;
            		FDCAN3_TxData[i*2+1] = vel & 0x00FF;
            		printf("%d:%d", i, vel);
            	}
            	printf("\r\n");
            	FDCAN3_TxHeader.Identifier = 0x1FF;

				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &FDCAN3_TxHeader, FDCAN3_TxData) != HAL_OK) {
					Error_Handler();
				}
				//printf("robot vel %d\r\n")
                //printf("%d %d %d %d\n\r", FDCAN3_TxData[1], FDCAN3_TxData[3], FDCAN3_TxData[5], FDCAN3_TxData[7]);
            }
        }
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
  MX_LPUART1_UART_Init();
  MX_FDCAN3_Init();
  MX_FDCAN1_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  pid.t = 1 / pow(10, 3);
  for (int k = 0; k < 4; k++){
	  pid.goal[k] = goal_list[0][k];
	  pid.error[k] = adc[k] - pid.goal[k];
	  pid.i[k] = 0;
  }
  printf("Initialized\r\n");

    FDCAN1_TxHeader.Identifier = 0x000;
    FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    FDCAN1_TxHeader.FDFormat = FDCAN_FD_CAN;
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN1_TxHeader.MessageMarker = 0;

    FDCAN1_sFilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_sFilterConfig.FilterIndex = 0;
    FDCAN1_sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN1_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_sFilterConfig.FilterID1 = CANID_ARM1;
    FDCAN1_sFilterConfig.FilterID2 = 0b11111111100;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        Error_Handler();
    }

    FDCAN3_TxHeader.Identifier = 0x000;
    FDCAN3_TxHeader.IdType = FDCAN_STANDARD_ID;
    FDCAN3_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN3_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    FDCAN3_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN3_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN3_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    FDCAN3_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN3_TxHeader.MessageMarker = 0;

    FDCAN3_sFilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN3_sFilterConfig.FilterIndex = 0;
    FDCAN3_sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    FDCAN3_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN3_sFilterConfig.FilterID1 = 0x101;
    FDCAN3_sFilterConfig.FilterID2 = 0x300;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN1_sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        Error_Handler();
    }

//    HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	int val;
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		HAL_ADC_Stop(&hadc1);
		val = HAL_ADC_GetValue(&hadc1);
		printf("%d\r\n", val);
		//ADC変換終了を待機
	} else {
		printf("error\n");
	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 79;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
