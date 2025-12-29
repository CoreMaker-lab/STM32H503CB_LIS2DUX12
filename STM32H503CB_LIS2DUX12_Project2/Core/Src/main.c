/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "icache.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lis2dux12_reg.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SENSOR_BUS hi2c1


/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static lis2dux12_xl_data_t data_xl;
static lis2dux12_outt_data_t data_temp;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);


float data_accx_min=0,data_accx_max=0;//加速度计x轴极值
float data_accy_min=0,data_accy_max=0;//加速度计y轴极值
float data_accz_min=0,data_accz_max=0;//加速度计z轴极值

float Offset_x=0.0f;//x偏移
float Scale_factor_x=0.0f;//x灵敏度
float Offset_y=0.0f;//y偏移
float Scale_factor_y=0.0f;//y灵敏度
float Offset_z=0.0f;//z偏移
float Scale_factor_z=0.0f;//z灵敏度

float calibrated_x=0.0f;//校准后加速度计x轴值
float calibrated_y=0.0f;//校准后加速度计y轴值
float calibrated_z=0.0f;//校准后加速度计z轴值

int acc_i=0;

uint8_t RxBuff[1];      //进入中断接收数据的数组
int Rx_flag=0;					//接受到数据标志


void calculate_calibration_params(void) {
	
	Offset_x=(data_accx_max+data_accx_min)/2;
	Offset_y=(data_accy_max+data_accy_min)/2;			
	Offset_z=(data_accz_max+data_accz_min)/2;		
	
	
	Scale_factor_x=(data_accx_max-data_accx_min)/2;
	Scale_factor_y=(data_accy_max-data_accy_min)/2;			
	Scale_factor_z=(data_accz_max-data_accz_min)/2;		
	
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
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("HELLO\n");
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);			
	HAL_GPIO_WritePin(SA0_GPIO_Port, SA0_Pin, GPIO_PIN_RESET);	
	
	
  lis2dux12_status_t status;
  stmdev_ctx_t dev_ctx;
  uint8_t id;
  lis2dux12_md_t md;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
//  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  lis2dux12_exit_deep_power_down(&dev_ctx);

  /* Check device ID */
  lis2dux12_device_id_get(&dev_ctx, &id);
	printf("LIS2DUX12_ID=0x%x,id=0x%x\n",LIS2DUX12_ID,id);
  if (id != LIS2DUX12_ID)
    while(1);

  /* Restore default configuration */
  lis2dux12_sw_reset(&dev_ctx);

  /* init bdu and add_inc */
  lis2dux12_init_set(&dev_ctx);

  /* Set Output Data Rate */
  md.fs =  LIS2DUX12_4g;
  md.bw = LIS2DUX12_ODR_div_4;
  md.odr = LIS2DUX12_3Hz_ULP;
  lis2dux12_mode_set(&dev_ctx, &md);
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, 1); //打开串口中断接收
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* Read output only if new values are available */
   /* Read output only if new values are available */
    lis2dux12_status_get(&dev_ctx, &status);
    if (status.drdy) {
      lis2dux12_xl_data_get(&dev_ctx, &md, &data_xl);
		if(Rx_flag==1)//X轴min
		{
			data_accx_min=data_xl.mg[0];
			Rx_flag=0;
			calculate_calibration_params();
		}
		else if(Rx_flag==2)//X轴max
		{
			data_accx_max=data_xl.mg[0];
			Rx_flag=0;
			calculate_calibration_params();
		}
		else if(Rx_flag==3)//Y轴min
		{
			data_accy_min=data_xl.mg[1];
			Rx_flag=0;
			calculate_calibration_params();
		}		
		else if(Rx_flag==4)//Y轴max
		{
			data_accy_max=data_xl.mg[1];
			Rx_flag=0;
			calculate_calibration_params();
		}		
		else if(Rx_flag==5)//Y轴min
		{
			data_accz_min=data_xl.mg[2];
			Rx_flag=0;
			calculate_calibration_params();
		}		
		else if(Rx_flag==6)//Y轴max
		{
			data_accz_max=data_xl.mg[2];
			Rx_flag=0;
			calculate_calibration_params();
		}			
	

			calibrated_x=1000*(data_xl.mg[0]-Offset_x)/Scale_factor_x;
			calibrated_y=1000*(data_xl.mg[1]-Offset_y)/Scale_factor_y;			
			calibrated_z=1000*(data_xl.mg[2]-Offset_z)/Scale_factor_z;			
			
			
			printf("min_x=%4.2f,max_x=%4.2lf,min_y=%4.2f,max_y=%4.2f,min_z=%4.2f,max_z=%4.2f\r\n",
			data_accx_min,data_accx_max,data_accy_min,data_accy_max,data_accz_min,data_accz_max);				
			printf("校准前acc[mg]:%4.2f\t%4.2f\t%4.2f\r\n",
			data_xl.mg[0], data_xl.mg[1], data_xl.mg[2]);			
			printf("校准后acc[mg]:%4.2f\t%4.2f\t%4.2f\r\n",
			calibrated_x, calibrated_y, calibrated_z);			
		}	
		HAL_Delay(10);
		
		
		
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */

// 捕获中断回调函数，每次捕获到信号就会进入这个回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle)
{

    Rx_flag=RxBuff[0];   
    RxBuff[0]=0;
//	printf("flag=%d",Rx_flag);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, 1); //每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收

}



/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}



/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);

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
#ifdef USE_FULL_ASSERT
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
