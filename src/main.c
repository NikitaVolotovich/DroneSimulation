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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct USART_prop{
    uint8_t usart_buf[60];
    uint8_t usart_cnt;
    uint8_t is_tcp_connect;
    uint8_t is_text;
} USART_prop_ptr;

MPU_ConfigTypeDef mpuConfig;
USART_prop_ptr usartprop;
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NORMAL_SPEED 10000
#define PLUS_COEFFICIENT 6.6
#define MINUS_COEFFICIENT 0.5
int FLUENT_INCREASE_SPEED = 0;
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int range = -1;
char str[60] = {0};
int mode = 0;
uint8_t i = 0;
uint8_t newline[] = "\r\n";
char *str2[] = {
    "String1\r\n",
    "String2\r\n",
    "String3\r\n",
    "String4\r\n",
    "String5\r\n"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
    MX_I2C2_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    accelSensorInitialization();
    enginesControlInitialization();
    rangeSensorInitialization();

    HAL_UART_Receive_IT(&huart2, (uint8_t*) str, 8);
    //char str_uart[9] = {0};
    
    HAL_TIM_Base_Start_IT(&htim2);
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1){ 
        // PWM max 65535
        // Normal 10000

        if(mode != 2){
            refreshAccelValues();
            if(mode == 1){
                sendAccelValues();
            }
            enginesPowerComputation(myAccelRaw);
        } else {
            uint8_t batteryLowMsg[] = "\n\rBattery is too low";
            sendStringByUART(batteryLowMsg, 21);
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
    RCC_OscInitStruct.PLL.PLLN = 72;
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
void enginesControl(int engine1PWM, int engine2PWM, int engine3PWM, int engine4PWM){
    TIM4->CCR1 = engine1PWM;
    TIM4->CCR2 = engine2PWM;
    TIM4->CCR3 = engine3PWM;
    TIM4->CCR4 = engine4PWM;
}

void enginesPowerComputation(RawData_Def AccelRaw){
    int engine1Speed = 0, engine2Speed = 0, engine3Speed = 0, engine4Speed = 0;
    int x = AccelRaw.x, y = AccelRaw.y;
    switch(mode){
        case 0: {
            engine3Speed = FLUENT_INCREASE_SPEED;
            engine1Speed = FLUENT_INCREASE_SPEED;
            engine2Speed = FLUENT_INCREASE_SPEED;
            engine4Speed = FLUENT_INCREASE_SPEED;
            FLUENT_INCREASE_SPEED+=100;
            sendValueByUART(FLUENT_INCREASE_SPEED);
            sendStringByUART(newline, sizeof(newline));
            if(FLUENT_INCREASE_SPEED >= NORMAL_SPEED){
                mode = 1;
            }
            break;
        }
        case 1: {
            if (AccelRaw.x > 0){
                engine3Speed = NORMAL_SPEED + (x * PLUS_COEFFICIENT);
                engine1Speed = NORMAL_SPEED - (x * MINUS_COEFFICIENT);
            } else {
                engine1Speed = NORMAL_SPEED + (abs(x)*PLUS_COEFFICIENT);
                engine3Speed = NORMAL_SPEED - (abs(x)*MINUS_COEFFICIENT);
            }
            if (AccelRaw.y > 0){
                engine4Speed = NORMAL_SPEED + (y * PLUS_COEFFICIENT);
                engine2Speed = NORMAL_SPEED - (y * MINUS_COEFFICIENT);
            } else {
                engine2Speed = NORMAL_SPEED + (abs(y)*PLUS_COEFFICIENT);
                engine4Speed = NORMAL_SPEED - (abs(y)*MINUS_COEFFICIENT);
            }
            break;
        }
    }
    enginesControl(engine1Speed, engine2Speed, engine3Speed, engine4Speed);
}

void sendValueByUART(int value){
    char buffer[16];  
    HAL_UART_Transmit(&huart2, (uint8_t*) buffer, sprintf(buffer, "%d", value), 100);
}

void sendStringByUART(uint8_t *string, int length){
    HAL_UART_Transmit(&huart2, string, length, 0xFFFF);
}

void refreshAccelValues(){
    MPU6050_Get_Accel_RawData(&myAccelRaw);
    MPU6050_Get_Gyro_RawData(&myGyroRaw);
}

void refreshRangeValues(){ 

}

void rangeSensorInitialization(){
    // setup_VL6180X();
    bool isInitialized = init();
    configureDefault();
    uint8_t string[] = "\r\nVL6180X status: ";
    sendStringByUART(string, sizeof(string));
    sendValueByUART(isInitialized);
}

void accelSensorInitialization(){
    MPU6050_Init(&hi2c2);
    mpuConfig.Accel_Full_Scale = AFS_SEL_4g;
    mpuConfig.ClockSource = Internal_8MHz;
    mpuConfig.CONFIG_DLPF =  DLPF_184A_188G_Hz;
    mpuConfig.Gyro_Full_Scale = FS_SEL_500;
    mpuConfig.Sleep_Mode_Bit = 0;
    MPU6050_Config(&mpuConfig);
}

void enginesControlInitialization(){
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void sendAccelValues() {
    uint8_t stringX[] = "\r\nAccelerometer x: ";
    sendStringByUART(stringX, sizeof(stringX));
    sendValueByUART(myAccelRaw.x);

    uint8_t stringY[] = "\r\nAccelerometer y: ";
    sendStringByUART(stringY, sizeof(stringY));
    sendValueByUART(myAccelRaw.y);

    uint8_t stringZ[] = "\r\nAccelerometer z: ";
    sendStringByUART(stringZ, sizeof(stringZ));
    sendValueByUART(myAccelRaw.z);

    sendStringByUART(newline, sizeof(newline));
}

void string_parse(char* buf_str){
    HAL_UART_Transmit(&huart2, (uint8_t*)buf_str, strlen(buf_str), 0x1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart==&huart2){
    UART2_RxCpltCallback();

  }
}

void UART2_RxCpltCallback(){
    uint8_t b;
    b = str[0];

    if (usartprop.usart_cnt>59){
        usartprop.usart_cnt=0;
        HAL_UART_Receive_IT(&huart2,(uint8_t*)str,1);
        return;
    }
    
    usartprop.usart_buf[usartprop.usart_cnt] = b;
 
    if(b==0x0A){
        usartprop.usart_buf[usartprop.usart_cnt+1]=0;
        string_parse((char*)usartprop.usart_buf);
        usartprop.usart_cnt=0;
        HAL_UART_Receive_IT(&huart2,(uint8_t*)str,1);
        return;
    }

    usartprop.usart_cnt++;
    HAL_UART_Receive_IT(&huart2,(uint8_t*)str,1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    checkBatteryVoltageLevel();
    if(htim == &htim2) {
        // HAL_UART_Transmit(&huart2,(uint8_t*)str2[i],strlen(str2[i]),0x1000);
        // i++;
        // if(i>4) {
        //     i=0;
        // }
    }
}

void checkBatteryVoltageLevel(){
    int adc_value = 0;
    HAL_ADC_Start(&hadc1); 
    if(HAL_ADC_PollForConversion(&hadc1, 7) == HAL_OK){ 
        adc_value  = HAL_ADC_GetValue(&hadc1); 
    }
    HAL_ADC_Stop(&hadc1);
    if(adc_value < 256){ // 256/4095
        mode = 2;
    } else if (mode == 2 && adc_value > 256){
        mode = 0;
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
