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
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint64_t former;
  uint64_t latter;
} Line;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MD_OLED_RST_Clr()     HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_RESET) //oled 复位端口操作
#define MD_OLED_RST_Set()     HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_SET)
#define FONT                  u8g2_font_6x13B_tf
#define WIDTH                 128
#define HEIGHT                64
#define MARGIN_LEFT           64
#define PADDING_UP            0
#define MARGIN_UP             12
#define MARGIN_DOWN           12
#define HALF_WIDTH            64
#define BEGINBIT              0x8000000000000000
#define ENDBIT                (0x0000000000000001<<(MARGIN_LEFT/2))
#define TILE_WIDTH            10
#define NUMBER_STRING_LENGTH  16
#define REFRESH_RATE          48
#define KPS_HISTORY_LENGTH    65
//#define TILE_LENGTH         56
#define CHART_HEIGHT          (HEIGHT-MARGIN_DOWN-MARGIN_UP)
#define TILE1                 0
#define TILE2                 10
#define TILE3                 20
#define TILE4                 30
#define SCREEN_REST_TIME      60
#define _SCREEN_REST_ON
#define roll()                rand()%2;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
u8g2_t u8g2;
uint16_t n = 0;
uint16_t i = 0;
int8_t j = 0;
uint16_t t = 0;
uint16_t fps = 0;
uint16_t count = 0;
uint8_t str[5] = {0};
uint8_t kpsstr[5] = {0};
uint8_t str1[10] = {0};
uint8_t keys[4] = {0};
uint8_t keys1[4] = {0};
uint8_t kpsqueue[REFRESH_RATE] = {0};
uint8_t kpshistory[KPS_HISTORY_LENGTH] = {0};
uint8_t kpshistoryi = 0;
uint8_t kpsi = 0;
uint8_t kpsmax = 1;
uint8_t kpsmaxps = 0;
uint8_t kpsmax1 = 1;
uint8_t displayon = SCREEN_REST_TIME;
uint32_t counts[4] = {0,0,0,0};
uint8_t num1[NUMBER_STRING_LENGTH] = {0};
uint8_t num2[NUMBER_STRING_LENGTH] = {0};
uint8_t num3[NUMBER_STRING_LENGTH] = {0};
uint8_t num4[NUMBER_STRING_LENGTH] = {0};
Line lines[4] = {0};
uint8_t rec_buf[2]={0xff};
float floatlist[8] = {0.0};
uint8_t kps = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,void *arg_ptr)
{
  switch (msg)
  {
    case U8X8_MSG_BYTE_SEND:
      //while(__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_BSY));
      HAL_SPI_Transmit(&hspi1,(uint8_t *)arg_ptr,arg_int,10);
      break;
    case U8X8_MSG_BYTE_INIT:
      break;
    case U8X8_MSG_BYTE_SET_DC:
      HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,arg_int);
      break;
    case U8X8_MSG_BYTE_START_TRANSFER: 
      break;
    case U8X8_MSG_BYTE_END_TRANSFER: 
      break;
    default:
      return 0;
  }
  return 1;
}

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr) 
{
  switch (msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      break;
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay(arg_int);
      break;
    case U8X8_MSG_GPIO_CS:
      break;
    case U8X8_MSG_GPIO_DC:
      HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,arg_int);
      break;
    case U8X8_MSG_GPIO_RESET:
      break;
  }
  return 1;
}


void update()
{

  //keys[0]=roll();
  //keys[1]=roll();
  //keys[2]=roll();
  //keys[3]=roll();
  keys[0]=!(rec_buf[0]&0x08);
  keys[1]=!(rec_buf[0]&0x04);
  keys[2]=!(rec_buf[0]&0x02);
  keys[3]=!(rec_buf[0]&0x01);
  kpsqueue[kpsi]=0;
  if(keys[0]&&!keys1[0])
  {
    counts[0]++;
    keys1[0]=1;
    kpsqueue[kpsi]++;
  }
  if(!keys[0]&&keys1[0])
  {
    keys1[0]=0;
  }
  if(keys[1]&&!keys1[1])
  {
    counts[1]++;
    keys1[1]=1;
    kpsqueue[kpsi]++;
  }
  if(!keys[1]&&keys1[1])
  {
    keys1[1]=0;
  }
  if(keys[2]&&!keys1[2])
  {
    counts[2]++;
    keys1[2]=1;
    kpsqueue[kpsi]++;
  }
  if(!keys[2]&&keys1[2])
  {
    keys1[2]=0;
  }
  if(keys[3]&&!keys1[3])
  {
    counts[3]++;
    keys1[3]=1;
    kpsqueue[kpsi]++;
  }
  if(!keys[3]&&keys1[3])
  {
    keys1[3]=0;
  }
  lines[0].latter >>= 1;
  lines[1].latter >>= 1;
  lines[2].latter >>= 1;
  lines[3].latter >>= 1;
  if(lines[0].former & ENDBIT)
    lines[0].latter |= BEGINBIT;
  if(lines[1].former & ENDBIT)
    lines[1].latter |= BEGINBIT;
  if(lines[2].former & ENDBIT)
    lines[2].latter |= BEGINBIT;
  if(lines[3].former & ENDBIT)
    lines[3].latter |= BEGINBIT;
  lines[0].former >>= 1;
  lines[1].former >>= 1;
  lines[2].former >>= 1;
  lines[3].former >>= 1;
  if(keys[0])
  {
    //counts[0]++;
    lines[0].former |= BEGINBIT;
  }
  if(keys[1])
  {
    //counts[1]++;
    lines[1].former |= BEGINBIT;
  }
  if(keys[2])
  {
    //counts[2]++;
    lines[2].former |= BEGINBIT;
  }
  if(keys[3])
  {
    //counts[3]++;
    lines[3].former |= BEGINBIT;
  }
  kps=0;
  for(i=0;i<REFRESH_RATE;i++)
    kps+=kpsqueue[i];
  if (kps>kpsmaxps)
    kpsmaxps=kps;
  kpsmax1=0;
  
  
  sprintf(num1,"%5d",counts[0]);
  sprintf(num2,"%5d",counts[1]);
  sprintf(num3,"%5d",counts[2]);
  sprintf(num4,"%5d",counts[3]);
  
}

void refresh()
{
  u8g2_ClearBuffer(&u8g2);
#ifdef _SCREEN_REST_ON
  if(!displayon)
  {
    u8g2_SendBuffer(&u8g2);
    return;
  }
#endif
  for (i=0;i<HALF_WIDTH;i++)
  {
    if ((lines[0].former<<i)&BEGINBIT)
      u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT,TILE1+MARGIN_UP,TILE_WIDTH);
    //if ((lines[0].latter<<i)&BEGINBIT)
    //  u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT+HALF_WIDTH,TILE1+PADDING_UP,TILE_WIDTH);
    if ((lines[1].former<<i)&BEGINBIT)
      u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT,TILE2+MARGIN_UP,TILE_WIDTH);
    //if ((lines[1].latter<<i)&BEGINBIT)
    //  u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT+HALF_WIDTH,TILE2+PADDING_UP,TILE_WIDTH);
    if ((lines[2].former<<i)&BEGINBIT)
      u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT,TILE3+MARGIN_UP,TILE_WIDTH);
    //if ((lines[2].latter<<i)&BEGINBIT)
    //  u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT+HALF_WIDTH,TILE3+PADDING_UP,TILE_WIDTH);
    if ((lines[3].former<<i)&BEGINBIT)
      u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT,TILE4+MARGIN_UP,TILE_WIDTH);
    //if ((lines[3].latter<<i)&BEGINBIT)
    //  u8g2_DrawVLine(&u8g2,i+MARGIN_LEFT+HALF_WIDTH,TILE4+PADDING_UP,TILE_WIDTH);
  }
  
  floatlist[2]=kpsmax;
  for (i=0;i<KPS_HISTORY_LENGTH-1;i++)
  {
    if(kpsmax)
    {
      j = kpshistoryi-i-1;
        
      if(j>0)
      {        
        floatlist[0]=kpshistory[j];
        floatlist[1]=kpshistory[j-1];
      }
      if(j==0)
      {        
        floatlist[0]=kpshistory[0];
        floatlist[1]=kpshistory[KPS_HISTORY_LENGTH-1];
      }
      if(j<0)
      {
        floatlist[0]=kpshistory[KPS_HISTORY_LENGTH + j];
        floatlist[1]=kpshistory[KPS_HISTORY_LENGTH + j-1];
      }
      //floatlist[0]=kpshistory[j];
      //floatlist[1]=kpshistory[j-1];
      //u8g2_DrawLine(&u8g2,64-i*2,35,64-(1+i)*2,35);
      if(floatlist[0]>floatlist[2] || floatlist[1]>floatlist[2])
        u8g2_DrawLine(&u8g2,KPS_HISTORY_LENGTH-i,MARGIN_UP,KPS_HISTORY_LENGTH-i-1,MARGIN_UP);
      else
        u8g2_DrawLine(&u8g2,
          KPS_HISTORY_LENGTH-i-1,CHART_HEIGHT+MARGIN_UP-(uint8_t)(floatlist[3]*(floatlist[0]/floatlist[2])),
          KPS_HISTORY_LENGTH-i-2,CHART_HEIGHT+MARGIN_UP-(uint8_t)(floatlist[3]*(floatlist[1]/floatlist[2])));
    }

    if (kpsmax1<kpshistory[i])
      kpsmax1=kpshistory[i];
  }
  kpsmax = kpsmax1;
  
  
  u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
  u8g2_DrawStr(&u8g2,0,MARGIN_UP-1,"KPS:");
  u8g2_DrawStr(&u8g2,32,MARGIN_UP-1,"MAX:");
  u8g2_DrawStr(&u8g2,96,MARGIN_UP-1,"FPS:");
  u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
  u8g2_DrawStr(&u8g2,2,63,num1);
  u8g2_DrawStr(&u8g2,34,63,num2);
  u8g2_DrawStr(&u8g2,66,63,num3);
  u8g2_DrawStr(&u8g2,98,63,num4);
  u8g2_DrawStr(&u8g2,96+15,MARGIN_UP-1,str);
  sprintf(kpsstr,"%2d",kps);
  u8g2_DrawStr(&u8g2,15,MARGIN_UP-1,kpsstr);
  sprintf(kpsstr,"%2d",kpsmax);
  u8g2_DrawStr(&u8g2,15+32,MARGIN_UP-1,kpsstr);
  
  u8g2_DrawHLine(&u8g2,0,MARGIN_UP,128);
  u8g2_DrawHLine(&u8g2,0,CHART_HEIGHT+MARGIN_UP,128);
  u8g2_DrawVLine(&u8g2,64,MARGIN_UP,CHART_HEIGHT+MARGIN_DOWN);
  u8g2_DrawVLine(&u8g2,32,HEIGHT-MARGIN_DOWN,MARGIN_DOWN);
  u8g2_DrawVLine(&u8g2,96,HEIGHT-MARGIN_DOWN,MARGIN_DOWN);
  
  u8g2_SendBuffer(&u8g2);
  
  fps++;
  kpsi++;
  if(kpsi>=REFRESH_RATE)
  {
    kpsi=0;
  }
  /*
  count++;
  if(count>=2)
  {
    count=0;
    kpshistory[kpshistoryi]=kps;
    kpshistoryi++;
    if(kpshistoryi>=KPS_HISTORY_LENGTH)
      kpshistoryi=0;
  }
  */
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  MD_OLED_RST_Set();
  floatlist[3]=(float)(CHART_HEIGHT-1);
  
  u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi , u8x8_stm32_gpio_and_delay); // 初始化 u8g2 结构体
  u8g2_InitDisplay(&u8g2);                                                                       // 根据所选的芯片进行初始化工作，初始化完成后，显示器处于关闭状态
  u8g2_SetPowerSave(&u8g2, 0);                                                                   // 打开显示器
  u8g2_ClearBuffer(&u8g2);

  u8g2_SetFontDirection(&u8g2,0);
  sprintf(str,"%d",sizeof(uint64_t));
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim6);
  //LL_USART_EnableIT_RXNE(USART1);
  HAL_UART_Receive_IT(&huart1,rec_buf,1);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM3)
  {
    update();
    refresh();
    HAL_IWDG_Refresh(&hiwdg);
    if(kpsmax||(uint8_t)(~rec_buf[0]))
      displayon=SCREEN_REST_TIME;
  }
  if (htim->Instance==TIM6)
  {
    sprintf(str,"%3d",fps);
    if(displayon)
      displayon--;
    kpshistory[kpshistoryi]=kpsmaxps;
    kpshistoryi++;
    if(kpshistoryi>=KPS_HISTORY_LENGTH)
      kpshistoryi=0;
    
    fps=0;
    kpsmaxps=0;
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
