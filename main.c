/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "HzLib.h"
#include "AsciiLib.h"
#include <stdlib.h>
#include "stm32f1xx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  SSD1289    5

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
static uint8_t LCD_Code;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#define DISP_ORIENTATION  0  /* angle 0 90 */

#if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )

#define  MAX_X  320
#define  MAX_Y  240

#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )

#define  MAX_X  240
#define  MAX_Y  320

#endif

/* LCD Registers */
#define R34            0x22

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

#define RGB565CONVERT(red, green, blue)\
(uint16_t)( (( red   >> 3 ) << 11 ) | \
(( green >> 2 ) << 5  ) | \
( blue  >> 3 ))

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
  SystemInit();
  //LCD_Clear(Red);

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
  MX_USART1_UART_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	LCD_Clear(Red);
	GUI_Text(68,144,"HY-MiniSTM32V",White,Red);
	if(HAL_GPIO_ReadPin(GPIOC, KEY_A_Pin) == 0){
		printf("Hola Mundo!\r\n");
		HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
		HAL_Delay(500);
	}
	if(HAL_GPIO_ReadPin(GPIOB, KEY_B_Pin) == 1){
		printf("Hola Mundo2!\r\n");
		HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
		HAL_Delay(500);
	}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL13;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_A_Pin */
  GPIO_InitStruct.Pin = KEY_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_B_Pin */
  GPIO_InitStruct.Pin = KEY_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_B_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 10;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 10;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 3;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 3;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */
  printf("Inicio GLCD\r\n");

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

static void delay_ms(uint16_t ms)
{
	uint16_t i,j;
	for( i = 0; i < ms; i++ )
	{
		for( j = 0; j < 1141; j++ );
	}
}

static __attribute__((always_inline)) uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD_REG = LCD_Reg;
  /* Read 16-bit Reg */
  return (LCD_RAM);
}

static __attribute__((always_inline)) void LCD_WriteReg(uint8_t LCD_Reg,uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  LCD_REG = LCD_Reg;
  /* Write 16-bit Reg */
  LCD_RAM = LCD_RegValue;
}

static __attribute__((always_inline)) void LCD_WriteRAM_Prepare(void)
{
  LCD_REG = R34;
}

static __attribute__((always_inline)) void LCD_WriteRAM(uint16_t RGB_Code)
{
  /* Write 16-bit GRAM Reg */
  LCD_RAM = RGB_Code;
}

static __attribute__((always_inline)) uint16_t LCD_ReadRAM(void)
{
  volatile uint16_t dummy;
  /* Write 16-bit Index (then Read Reg) */
  LCD_REG = R34; /* Select GRAM Reg */
  /* Read 16-bit Reg */
  dummy = LCD_RAM;

  return LCD_RAM;
}

static __attribute__((always_inline)) void LCD_SetCursor( uint16_t Xpos, uint16_t Ypos )
{
    #if  ( DISP_ORIENTATION == 90 ) || ( DISP_ORIENTATION == 270 )

 	uint16_t temp = Xpos;

			 Xpos = Ypos;
			 Ypos = ( MAX_X - 1 ) - temp;

	#elif  ( DISP_ORIENTATION == 0 ) || ( DISP_ORIENTATION == 180 )

	#endif

  switch( LCD_Code )
  {
     default:		 /* 0x9320 0x9325 0x9328 0x9331 0x5408 0x1505 0x0505 0x7783 0x4531 0x4535 */
          LCD_WriteReg(0x0020, Xpos );
          LCD_WriteReg(0x0021, Ypos );
	      break;

     case SSD1289:   /* 0x8989 */
	      LCD_WriteReg(0x004e, Xpos );
          LCD_WriteReg(0x004f, Ypos );
	      break;
  }
}

static __attribute__((always_inline)) uint16_t LCD_BGR2RGB(uint16_t color)
{
	uint16_t  r, g, b, rgb;

	b = ( color>>0 )  & 0x1f;
	g = ( color>>5 )  & 0x3f;
	r = ( color>>11 ) & 0x1f;

	rgb =  (b<<11) + (g<<5) + (r<<0);

	return( rgb );
}

uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos)
{
	uint16_t dummy;

	LCD_SetCursor(Xpos,Ypos);

	switch( LCD_Code )
	{
		case SSD1289:
 		     return  LCD_ReadRAM();
             {
		        uint8_t red,green,blue;
				/* Write 16-bit Index (then Read Reg) */
                LCD_REG = R34;  /* Select GRAM Reg */

				dummy = LCD_RAM;

		        red = LCD_RAM >> 3;
                green = LCD_RAM >> 2;
                blue = LCD_RAM >> 3;
                dummy = (uint16_t) ( ( red<<11 ) | ( green << 5 ) | blue );
			 }
 	         return  dummy;
        default:	/* 0x9320 0x9325 0x9328 0x9331 0x5408 0x1505 0x0505 0x9919 */
 		     return  LCD_BGR2RGB( LCD_ReadRAM() );
	}
}

void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)
{
	if( Xpos >= MAX_X || Ypos >= MAX_Y )
	{
		return;
	}
	LCD_SetCursor(Xpos,Ypos);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(point);
}

void LCD_DrawLine( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 , uint16_t color )
{
    short dx,dy;      /* ����X Y�������ӵı���ֵ */
    short temp;       /* ��� �յ��С�Ƚ� ��������ʱ���м���� */

    if( x0 > x1 )     /* X�����������յ� �������� */
    {
	    temp = x1;
		x1 = x0;
		x0 = temp;
    }
    if( y0 > y1 )     /* Y�����������յ� �������� */
    {
		temp = y1;
		y1 = y0;
		y0 = temp;
    }

	dx = x1-x0;       /* X�᷽���ϵ����� */
	dy = y1-y0;       /* Y�᷽���ϵ����� */

    if( dx == 0 )     /* X����û������ ����ֱ�� */
    {
        do
        {
            LCD_SetPoint(x0, y0, color);   /* �����ʾ �费ֱ�� */
            y0++;
        }
        while( y1 >= y0 );
		return;
    }
    if( dy == 0 )     /* Y����û������ ��ˮƽֱ�� */
    {
        do
        {
            LCD_SetPoint(x0, y0, color);   /* �����ʾ ��ˮƽ�� */
            x0++;
        }
        while( x1 >= x0 );
		return;
    }
	/* ����ɭ��ķ(Bresenham)�㷨���� */
    if( dx > dy )                         /* ����X�� */
    {
	    temp = 2 * dy - dx;               /* �����¸����λ�� */
        while( x0 != x1 )
        {
	        LCD_SetPoint(x0,y0,color);    /* ����� */
	        x0++;                         /* X���ϼ�1 */
	        if( temp > 0 )                /* �ж����¸����λ�� */
	        {
	            y0++;                     /* Ϊ�������ڵ㣬����x0+1,y0+1�� */
	            temp += 2 * dy - 2 * dx;
	 	    }
            else
            {
			    temp += 2 * dy;           /* �ж����¸����λ�� */
			}
        }
        LCD_SetPoint(x0,y0,color);
    }
    else
    {
	    temp = 2 * dx - dy;                      /* ����Y�� */
        while( y0 != y1 )
        {
	 	    LCD_SetPoint(x0,y0,color);
            y0++;
            if( temp > 0 )
            {
                x0++;
                temp+=2*dy-2*dx;
            }
            else
			{
                temp += 2 * dy;
			}
        }
        LCD_SetPoint(x0,y0,color);
	}
}

void PutChar( uint16_t Xpos, uint16_t Ypos, uint8_t ASCI, uint16_t charColor, uint16_t bkColor )
{
	uint16_t i, j;
    uint8_t buffer[16], tmp_char;
    GetASCIICode(buffer,ASCI);  /* ȡ��ģ���� */
    for( i=0; i<16; i++ )
    {
        tmp_char = buffer[i];
        for( j=0; j<8; j++ )
        {
            if( (tmp_char >> 7 - j) & 0x01 == 0x01 )
            {
                LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* �ַ���ɫ */
            }
            else
            {
                LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* ������ɫ */
            }
        }
    }
}

void GUI_Text(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
    uint8_t TempChar;
    do
    {
        TempChar = *str++;
        PutChar( Xpos, Ypos, TempChar, Color, bkColor );
        if( Xpos < MAX_X - 8 )
        {
            Xpos += 8;
        }
        else if ( Ypos < MAX_Y - 16 )
        {
            Xpos = 0;
            Ypos += 16;
        }
        else
        {
            Xpos = 0;
            Ypos = 0;
        }
    }
    while ( *str != 0 );
}

void PutChinese(uint16_t Xpos,uint16_t Ypos,uint8_t *str,uint16_t Color,uint16_t bkColor)
{
	uint8_t i,j;
	uint8_t buffer[32];
	uint16_t tmp_char=0;

	GetGBKCode(buffer,str);  /* ȡ��ģ���� */

	for ( i = 0; i < 16; i++ )
	{
		tmp_char = buffer[i*2];
		tmp_char = ( tmp_char << 8 );
		tmp_char |= buffer[2*i+1];
		for (j = 0; j < 16; j++ )
		{
		    if ( (tmp_char >> 15-j ) & 0x01 == 0x01 )
	        {
		        LCD_SetPoint(Xpos+j,Ypos+i,Color);  /* �ַ���ɫ */
	        }
	        else
	        {
	            LCD_SetPoint(Xpos+j,Ypos+i,bkColor);  /* ������ɫ */
	        }
	    }
	}
}

void GUI_Chinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor)
{
	do
	{
		PutChinese(Xpos,Ypos,str++,Color,bkColor);
		str++;
		if ( Xpos < MAX_X - 16 )
		{
			Xpos += 16;
		}
		else if ( Ypos < MAX_Y - 16 )
		{
			Xpos = 0;
			Ypos += 16;
		}
        else
        {
            Xpos = 0;
            Ypos = 0;
        }
    }
    while(*str!=0);
}

void LCD_Initializtion(void){
	uint16_t DeviceCode;
	delay_ms(100);
	DeviceCode = LCD_ReadReg(0x0000);

	if( DeviceCode == 0x8989 ){
		LCD_Code = SSD1289;
		LCD_WriteReg(0x0000,0x0001);    delay_ms(50);   /* �򿪾��� */
		LCD_WriteReg(0x0003,0xA8A4);    delay_ms(50);
		LCD_WriteReg(0x000C,0x0000);    delay_ms(50);
		LCD_WriteReg(0x000D,0x080C);    delay_ms(50);
		LCD_WriteReg(0x000E,0x2B00);    delay_ms(50);
		LCD_WriteReg(0x001E,0x00B0);    delay_ms(50);
		LCD_WriteReg(0x0001,0x2B3F);    delay_ms(50);   /* �����������320*240 0x2B3F */
		LCD_WriteReg(0x0002,0x0600);    delay_ms(50);
		LCD_WriteReg(0x0010,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0011,0x6070);    delay_ms(50);   /* �������ݸ�ʽ 16λɫ ���� 0x6070 */
		LCD_WriteReg(0x0005,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0006,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0016,0xEF1C);    delay_ms(50);
		LCD_WriteReg(0x0017,0x0003);    delay_ms(50);
		LCD_WriteReg(0x0007,0x0133);    delay_ms(50);
		LCD_WriteReg(0x000B,0x0000);    delay_ms(50);
		LCD_WriteReg(0x000F,0x0000);    delay_ms(50);   /* ɨ�迪ʼ��ַ */
		LCD_WriteReg(0x0041,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0042,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0048,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0049,0x013F);    delay_ms(50);
		LCD_WriteReg(0x004A,0x0000);    delay_ms(50);
		LCD_WriteReg(0x004B,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0044,0xEF00);    delay_ms(50);
		LCD_WriteReg(0x0045,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0046,0x013F);    delay_ms(50);
		LCD_WriteReg(0x0030,0x0707);    delay_ms(50);
		LCD_WriteReg(0x0031,0x0204);    delay_ms(50);
		LCD_WriteReg(0x0032,0x0204);    delay_ms(50);
		LCD_WriteReg(0x0033,0x0502);    delay_ms(50);
		LCD_WriteReg(0x0034,0x0507);    delay_ms(50);
		LCD_WriteReg(0x0035,0x0204);    delay_ms(50);
		LCD_WriteReg(0x0036,0x0204);    delay_ms(50);
		LCD_WriteReg(0x0037,0x0502);    delay_ms(50);
		LCD_WriteReg(0x003A,0x0302);    delay_ms(50);
		LCD_WriteReg(0x003B,0x0302);    delay_ms(50);
		LCD_WriteReg(0x0023,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0024,0x0000);    delay_ms(50);
		LCD_WriteReg(0x0025,0x8000);    delay_ms(50);
		LCD_WriteReg(0x004f,0);        /* ����ַ0 */
		LCD_WriteReg(0x004e,0);
	}

}

void LCD_Clear(uint16_t Color)
{
	uint32_t index=0;
	LCD_SetCursor(0,0);
	LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

	for( index = 0; index < MAX_X * MAX_Y; index++ )
	{
		LCD_RAM = Color;
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
