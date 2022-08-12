/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "pid_controller.h"
#include "cJSON.h"

/* Private macro -------------------------------------------------------------*/
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t tick = 0;
float PWM_MaxVal_Out = 7199;
float PWM_MinVal_Out = -7199;
int32_t PID_cnt_pulse = 0;
double PID_Read_Vel = 0;
int16_t PID_Set_Vel = 0;
int16_t PID_Dir = 0;
uint16_t i= 0;

/* PID pamameter */

double PID_Spd_Kp = 150;
double PID_Spd_Ki = 200;
double PID_Spd_Kd = 0;

float sampleTimeSeconds = 0.005;
float pidSpeedOutput = 0;

/* Uart Reciver */
char rx_buffer[200];
unsigned int rx_index = 0;
uint8_t rx_data;

cJSON *str_json, *str_DC3, *str_DIR3;
long last;
uint32_t DC3, DIR3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
            #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else 
            #define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
    return ch;
}

void PWM_Send_Out(int32_t setpoint_in);
void PWM_OFF(void);
int16_t getTIMx_DetaCnt(TIM_TypeDef * TIMx);
void Get_Motor_Speed(void);
void Read_Motor_Speed(void);
void PID_Motor_Vel(void);

void UART_Received_Data(void);
void UART_Clr_Buf_Rev_Data(void);
void UART_Handle_Json(char *DataJson);
PIDControl mySpeedPID;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    tick = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    PIDInit(&mySpeedPID,PID_Spd_Kp,PID_Spd_Ki,PID_Spd_Kd,sampleTimeSeconds,PWM_MinVal_Out,PWM_MaxVal_Out,AUTOMATIC,DIRECT);
    PIDControllerDirectionSet(&mySpeedPID, PID_Dir);
    HAL_UART_Receive_IT(&huart1,&rx_data,1);
    while (1)
    {
        if ( DC3 == 0 ) PWM_OFF();
        PID_Motor_Vel();
    }
}

/**
    * @brief System Clock Configuration
    * @retval None
    */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 7199;
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
    sConfigOC.Pulse = 1750;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
    * @brief TIM2 Initialization Function
    * @param None
    * @retval None
    */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

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
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void PID_Motor_Vel(void)
{
    if ((HAL_GetTick()-tick)>5)
    {
        Get_Motor_Speed();
        PIDSetpointSet(&mySpeedPID,PID_Set_Vel);
        PIDInputSet(&mySpeedPID, PID_Read_Vel);
        PIDCompute(&mySpeedPID);
        pidSpeedOutput = PIDOutputGet(&mySpeedPID);
        PWM_Send_Out(pidSpeedOutput);
        tick = HAL_GetTick();
    }
}

void PWM_Send_Out(int32_t setpoint_in)
{
    if (setpoint_in > 0) {
        /* Motor Rotate Forward Direction */
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,setpoint_in);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

     }
    else {
         /* Motor Rotate Reverse Direction */
         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);
         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, abs(setpoint_in));
     }
}

void PWM_OFF(void)
{
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 0);
}

int16_t getTIMx_DetaCnt(TIM_TypeDef * TIMx)
{
    int16_t cnt;
    /*This is the default belief that the encoder will not have a 
    pulse change of 0x7fff per unit time, which is actually the case */
    cnt = TIMx->CNT-0x7fff;
    TIMx->CNT = 0x7fff;
    return cnt;
}

void Get_Motor_Speed(void)
{
    static int PID_cnt_pulse_now = 0;
    static int PID_cnt_pulse_last = 0;
    /* Record this left and right encoder data */
    PID_cnt_pulse_now += getTIMx_DetaCnt(TIM2);
    PID_cnt_pulse = abs(PID_cnt_pulse_now - PID_cnt_pulse_last);
    /* 5ms speed measurement */
    PID_Read_Vel = (PID_cnt_pulse*12000)/1320;
    /* Record the last encoder data */
    PID_cnt_pulse_last = PID_cnt_pulse_now;
}

void Read_Motor_Speed(void)
{
    if ((HAL_GetTick()-tick)>5)
    {
        Get_Motor_Speed();
        PWM_Send_Out(PWM_MaxVal_Out);
        tick = HAL_GetTick();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart -> Instance == USART1)
    {
        //Chuong trinh doc du lieu => mang data
        UART_Received_Data();
        HAL_UART_Receive_IT(&huart1,&rx_data,1);
    }
}

void UART_Received_Data(void)
{
    /* rx_buffer */
    if(rx_data != '\n')
    {
        rx_buffer[rx_index++] = rx_data;
    }
    else 
    {
        UART_Handle_Json(rx_buffer);
        UART_Clr_Buf_Rev_Data();
    }
}

void UART_Clr_Buf_Rev_Data(void)
{
    rx_index = 0;
    for(int i = 0; i < 200; i++)
    {
        rx_buffer[i] = 0;
    }
    last = HAL_GetTick();
}

void UART_Handle_Json (char *DataJson)
{
    str_json = cJSON_Parse(DataJson);
    if(!str_json)
    {
        printf("JSON ERROR! \r\n");
        return;
    }
    else
    {
//      printf("JSON OK!\r\n");
        if(cJSON_GetObjectItem(str_json, "DC3"))
        {
            DC3 = atoi(cJSON_GetObjectItem(str_json, "DC3") ->valuestring);
						PID_Set_Vel = DC3;
        }
				if(cJSON_GetObjectItem(str_json, "DIR3"))
        {
            DIR3 = atoi(cJSON_GetObjectItem(str_json, "DIR3") ->valuestring);
						PIDControllerDirectionSet(&mySpeedPID, DIR3);
        }
        cJSON_Delete(str_json);
    }
}
/* USER CODE END 4 */

/**
    * @brief    This function is executed in case of error occurrence.
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

#ifdef    USE_FULL_ASSERT
/**
    * @brief    Reports the name of the source file and the source line number
    *                 where the assert_param error has occurred.
    * @param    file: pointer to the source file name
    * @param    line: assert_param error line source number
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
