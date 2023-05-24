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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
}subKeyBoard;

typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}Color;
typedef struct
{
	uint16_t H;
	uint16_t S;
	uint16_t V;
}HSV_Color;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY1                    HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)
#define KEY2                    HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)
#define KEY3                    HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)
#define KEY4                    HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)
#define FUNCTION_KEY1           HAL_GPIO_ReadPin(FUNCTION_KEY1_GPIO_Port,FUNCTION_KEY1_Pin)
#define FUNCTION_KEY2           HAL_GPIO_ReadPin(FUNCTION_KEY2_GPIO_Port,FUNCTION_KEY2_Pin)
#define FUNCTION_KEY3           HAL_GPIO_ReadPin(FUNCTION_KEY3_GPIO_Port,FUNCTION_KEY3_Pin)
#define FUNCTION_KEY4           HAL_GPIO_ReadPin(FUNCTION_KEY4_GPIO_Port,FUNCTION_KEY4_Pin)
#define KNOB                    HAL_GPIO_ReadPin(KNOB_GPIO_Port,KNOB_Pin)
#define EC11_A                  HAL_GPIO_ReadPin(EC11_A_GPIO_Port,EC11_A_Pin)
#define EC11_B                  HAL_GPIO_ReadPin(EC11_B_GPIO_Port,EC11_B_Pin)
#define CharacterID(x)          (x-'a'+4)
#define LED_NUM                 (4)
#define ONE_PULSE               (61)
#define ZERO_PULSE              (29)
#define NONE_PULSE              (90)
#define RGB_BUFFER_LENGTH       (424)
#define KEY1_BINDING            CharacterID('z')
#define KEY2_BINDING            CharacterID('x')
#define KEY3_BINDING            CharacterID('c')
#define KEY4_BINDING            CharacterID('v')
#define KONB_BINDING            0x29                //Escape
#define FUNCTION_KEY1_BINDING   0
#define FUNCTION_KEY2_BINDING   0
#define FUNCTION_KEY3_BINDING   0
#define FUNCTION_KEY4_BINDING   0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buf=0xff;
uint8_t count=0;
uint8_t keys[10]={1};
uint8_t ec11flag=0;
//uint8_t spi_sw_i=0;
//uint8_t spi_sw_j=0;
//uint8_t spi_sw_f=0;
//uint8_t spi_sw_c=0x80;
subKeyBoard keyBoardHIDsub = {0,0,0,0,0,0,0,0};
Color colors[4]={{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
const uint8_t code[]={0x80,0xFE};
uint32_t RGB_buffer[RGB_BUFFER_LENGTH] = {
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  NONE_PULSE,NONE_PULSE,NONE_PULSE,NONE_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,
  ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE,ZERO_PULSE
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_RGB(uint8_t R,uint8_t G,uint8_t B,uint16_t index)
{
  for (uint8_t i = 0;i < 8;i++)
  {
    RGB_buffer[304+index*24+i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
    RGB_buffer[304+index*24+i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
    RGB_buffer[304+index*24+i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start_DMA(&htim8,TIM_CHANNEL_3,RGB_buffer,RGB_BUFFER_LENGTH);
  for (uint8_t i=1;i<127;i++)
  {
    Set_RGB(i,i,i,0);
    Set_RGB(i,i,i,1);
    Set_RGB(i,i,i,2);
    Set_RGB(i,i,i,3);
    HAL_Delay(1);
  }
  for (uint8_t i=128;i>0;i--)
  {
    Set_RGB(i,i,i,0);
    Set_RGB(i,i,i,1);
    Set_RGB(i,i,i,2);
    Set_RGB(i,i,i,3);
    HAL_Delay(1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    colors[0].R=keys[0]?colors[0].R?colors[0].R-=2:0:128;
    colors[1].B=keys[1]?colors[1].B?colors[1].B-=2:0:128;
    colors[2].G=keys[2]?colors[2].G?colors[2].G-=2:0:128;
    colors[3].R=keys[3]?colors[3].R?colors[3].R-=2:0:128;
    colors[3].G=keys[3]?colors[3].G?colors[3].G-=2:0:128;
    Set_RGB(colors[0].R,colors[0].G,colors[0].B,0);
    Set_RGB(colors[1].R,colors[1].G,colors[1].B,1);
    Set_RGB(colors[2].R,colors[2].G,colors[2].B,2);
    Set_RGB(colors[3].R,colors[3].G,colors[3].B,3);
    HAL_Delay(1);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM7)
  {
    keys[0]=KEY1;
    keys[1]=KEY2;
    keys[2]=KEY3;
    keys[3]=KEY4;
    keys[4]=FUNCTION_KEY1;
    keys[5]=FUNCTION_KEY2;
    keys[6]=FUNCTION_KEY3;
    keys[7]=FUNCTION_KEY4;
    keys[8]=KNOB;
    keyBoardHIDsub.KEYCODE1=keys[0]?0:KEY1_BINDING;
    keyBoardHIDsub.KEYCODE2=keys[1]?0:KEY2_BINDING;
    keyBoardHIDsub.KEYCODE3=keys[2]?0:KEY3_BINDING;
    keyBoardHIDsub.KEYCODE4=keys[3]?0:KEY4_BINDING;
    keyBoardHIDsub.KEYCODE1=keys[0]?keys[4]?0:0x35:keyBoardHIDsub.KEYCODE1;//`
    //keyBoardHIDsub.MODIFIER=keys[0]?keys[4]?0:0x80:keyBoardHIDsub.MODIFIER;
    keyBoardHIDsub.KEYCODE2=keys[1]?keys[5]?0:0x2b:keyBoardHIDsub.KEYCODE2;//Tab
    keyBoardHIDsub.KEYCODE3=keys[2]?keys[6]?0:0x3b:keyBoardHIDsub.KEYCODE3;//F2
    keyBoardHIDsub.KEYCODE4=keys[3]?keys[7]?0:0x3b:keyBoardHIDsub.KEYCODE4;//Shift+F2
    keyBoardHIDsub.MODIFIER=keys[3]?keys[7]?0:0x02:keyBoardHIDsub.MODIFIER;
    keyBoardHIDsub.KEYCODE5=keys[8]?0:0x29;
    if(ec11flag)
    {
      keyBoardHIDsub.KEYCODE6=keys[9]?0x51:0x52;
      ec11flag--;
    }
    else
    {
      keyBoardHIDsub.KEYCODE6=0;
    }
    USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&keyBoardHIDsub,sizeof(keyBoardHIDsub));
    count++;
    if(count>=10)
    {
      count=0;
      buf=keys[0]<<3|keys[1]<<2|keys[2]<<1|keys[3]|
          keys[4]<<7|keys[5]<<6|keys[6]<<5|keys[7]<<4;
      HAL_UART_Transmit(&huart1,&buf,1,10);
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
        if(htim->Instance->CR1==0x01)
        {
          ec11flag=5;
          keys[9]=0;
        }
        if(htim->Instance->CR1==0x11)
        {
          ec11flag=5;
          keys[9]=1;
        }
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
