/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if defined (DEBUG)
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small PRINTF (option LD Linker->Libraries->Small PRINTF
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef 	RCC_ClockFreq;

/*Assign Address variable*/

//Three variable allocate description method in KeilC as below
// use STLINKV2 to monitor below address data 
/*
	0x8001006 : 0x600D
	0x8001016 : 0x601E
	0x8001026 : 0x602F		//for test
*/

#define KEY_PageStart				(uint32_t)0x08001000
#define KEY1_Address				(uint32_t)(KEY_PageStart|0x04)			//0x8001006

#define KEY1_Default					(uint32_t)0x87654321
#define KEY1						(uint32_t)0x600D4321					//to check application exist or not

/*Flash copy variable*/
#define FLASH_PAGE	(0x400/4)
__IO uint32_t TempBuffer[FLASH_PAGE]={0};

/*JUMP address variable*/
#define APPLICATION_ADDRESS     	(uint32_t)0x08003000
#define APP_DEFAULT_BOOT			(uint32_t)0x08000000 	

#if   (defined ( __CC_ARM ))
  __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
#endif

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

/*UART DATA variable*/
#define USART_RX_DATA_SIZE   10
uint8_t USART_Rx_ptr_in = 0;
uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE] ={0}; 

#define ForceBreak(void)		\
{	\
	PRINTF("\r\n");	\
	if ('x' == USART_ReceiveData(USART1))	\
		break;	\
	if ('X' == USART_ReceiveData(USART1))	\
		break;	\
}	\

/* Private functions ---------------------------------------------------------*/
void Flash_DataUpdate(uint32_t InsertAddress,uint32_t Data)
{
	uint16_t i = 0 ;
	uint16_t j = 0 ;
	uint32_t StartAddress = KEY_PageStart;
	uint16_t InsertIndex = 0;
	uint16_t Data_MSB = 0,Data_LSB = 0;;

	for (i=0;i<(FLASH_PAGE);i++)
	{
		TempBuffer[i] = *(__IO uint32_t*) (StartAddress + i*4);
	}

	#if 0	//terminal debug
	PRINTF("\r\n");		
	for (j=0;j<(FLASH_PAGE/4);j++)
	{
		PRINTF("LINE%2d :",j);
		PRINTF("Address 0x%8X : 0x%8X,0x%8X,0x%8X,0x%8X ",(StartAddress + j*0x10),
															TempBuffer[j*4+0],
															TempBuffer[j*4+1],
															TempBuffer[j*4+2],
															TempBuffer[j*4+3]);
		PRINTF("\r\n");		
	}
	PRINTF("\r\n");		
	#endif

	#if 1	//insert data in buffer , which index ? 
	InsertIndex = (InsertAddress - StartAddress)%4 + 1;	//need to find better check inded method
	PRINTF("A)InsertIndex:%2d\r\n" , InsertIndex);
	PRINTF("A)BEFORE :TempBuffer:0x%8X\r\n" , TempBuffer[InsertIndex]);	

	Data_MSB= (uint16_t)(TempBuffer[InsertIndex]>>16);
	Data_LSB= (uint16_t)(TempBuffer[InsertIndex]&0x0000FFFF);	
	PRINTF("A)Data_MSB:0x%8X\r\nData_LSB:0x%8X\r\n" , Data_MSB,Data_LSB);

	Data_MSB = Data >>16;
	TempBuffer[InsertIndex] = Data_MSB<<16 |Data_LSB;
	PRINTF("A)AFTER :TempBuffer:0x%8X\r\n" , TempBuffer[InsertIndex]);	
	#endif

}

void Flash_SignReverse(void)
{
	uint8_t writeprotect = 0;
	uint32_t flashdestination = 0;

	/* Initialize flashdestination variable */
	flashdestination = KEY_PageStart;

	//copy data & update Key1 to buffer
	Flash_DataUpdate(KEY1_Address,KEY1_Default);
	
	//erase page , 0x8001000~0x80013FF
	FLASH_ErasePage(KEY_PageStart);
	
	if (FLASH_If_GetWriteProtectionStatus() != 0)  
	{
		/* Disable the write protection */
		writeprotect = FLASH_If_DisableWriteProtection();

		if ( writeprotect== 0)
		{
			/* Launch loading new option bytes */
			FLASH_OB_Launch();
		}
	}	
	FLASH_If_Write(&flashdestination,(uint32_t*) TempBuffer, FLASH_PAGE);  
	PRINTF("\r\nA)Flash_SignReverse finish\r\n");		
}

void UARTReceiveStringCheck(void)
{
	/*
		if UART receive string BOOT
		1. update KEY1 to (0x8765)
		2. Jump to Boot loader
	*/

    USART_Rx_Buffer[USART_Rx_ptr_in++] = USART_ReceiveData(USART1);

	/* To avoid buffer overflow */
	if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
	{
		USART_Rx_ptr_in = 0;
	}

	if (USART_Rx_Buffer[0]== 'B' &&
		USART_Rx_Buffer[1]== 'O' &&
		USART_Rx_Buffer[2]== 'O' &&
		USART_Rx_Buffer[3]== 'T'	&&
		USART_Rx_Buffer[4]== '\0')
	{
		PRINTF("A)String check OK , JUMP to BOOT\r\n");

		FLASH_If_Init();	
		Flash_SignReverse();

//		JumpToBootFunction();		
		NVIC_SystemReset();				
	}

	#if 0//if wrong string , clear index since string incorrect
	PRINTF("String INPUT NG , reset INDEX start \r\n");
	memset( USART_Rx_Buffer,0,sizeof( USART_Rx_Buffer));			
	USART_Rx_ptr_in = 0;
	PRINTF("String INPUT NG , reset INDEX finish\r\n");

	#endif
}

void JumpToBootFunction(void)
{
	if (((*(__IO uint32_t*)APP_DEFAULT_BOOT) & 0x2FFE0000 ) == 0x20000000)
	{ 
	  /* Jump to user application */
	  JumpAddress = *(__IO uint32_t*) (APP_DEFAULT_BOOT + 4);
	  Jump_To_Application = (pFunction) JumpAddress;
	  
	  /* Initialize user application's Stack Pointer */
	  __set_MSP(*(__IO uint32_t*) APP_DEFAULT_BOOT);
	  
	  /* Jump to application */
	  Jump_To_Application();
	}	
}

void RelocateAPSRAM(void)
{
	uint8_t i = 0 ;
	
	for(i = 0; i < 48; i++)
	{
		VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}

	/* Enable the SYSCFG peripheral clock*/
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	/* Remap SRAM at 0x00000000 */
	SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
}
	
void LED_Config(void)	//for test purpose
{
  	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);		

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void LED_Test(void)	//for test purpose
{
	GPIOC->ODR ^= GPIO_Pin_8;
	Delay(1000);
//	Delay(500);
	PRINTF("A)APPLICATION program running\r\n");	
	
//	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
//	Delay(100);

//	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
//	Delay(100);	
}

/*
	TIMxCLK = PCLK1 = HCLK = SystemCoreClock
	TIMx counter clock = TIMxCLK /((Prescaler + 1)*(Period + 1))
	                = 48 MHz / ((7+1)*(11999+1))
	                = 500 Hz 
     ==> TIMx counter period = 2 ms
*/
void TIM6_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIMx clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* TIMx base configuration */
	TIM_TimeBaseStructure.TIM_Period = 11999;
	TIM_TimeBaseStructure.TIM_Prescaler = 7;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM6, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART1_Test(void)
{
	__IO uint8_t temp;
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	{
			temp = USART_ReceiveData(USART1);
			PRINTF("Press KEY : %c \n\r",temp);

			switch (temp)
			{

				case '1' :
					JumpToBootFunction();
					break;
					
				default : 
					PRINTF("INPUT CMD not support !\r\n");
					break;
			}

	}
}

void USART1_Config(void)	//TX : PA9 , RX : PA10
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Clock configuration ---------------------------------------------------*/
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* GPIO configuration ----------------------------------------------------*/
	GPIO_DeInit(GPIOA);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	/* Configure USARTx_Tx,USARTx_Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration ---------------------------------------------------*/
	USART_DeInit(USART1);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	/* The software must wait until TC=1. The TC flag remains cleared during all data
	transfers and it is set by hardware at the last frame’s end of transmission*/	
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	/* Enable the USARRx */
	USART_Cmd(USART1, ENABLE);
}

void SysTickTimer_Config(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 1 //debug
	PRINTF("A)===========================\r\n");
	PRINTF("A)SYSCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.SYSCLK_Frequency);
	PRINTF("A)HCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.HCLK_Frequency);
	PRINTF("A)PCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.PCLK_Frequency);
	PRINTF("A)ADCCLK_Frequency= %d \r\n" , 	RCC_ClockFreq.ADCCLK_Frequency);
	PRINTF("A)CECCLK_Frequency = %d \r\n" , 	RCC_ClockFreq.CECCLK_Frequency);
	PRINTF("A)I2C1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.I2C1CLK_Frequency);
	PRINTF("A)USART1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.USART1CLK_Frequency); 
	#endif /*debug*/
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t uTime)
{ 
	uwTimingDelay = uTime;
	while(uwTimingDelay != 0);
}

//void Delay_s(__IO uint32_t mTime)
//{ 
//	uint32_t i;
//	for(i=0;i<mTime;i++)
//		Delay_ms(1000);
//}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/

void UART_SendByte(uint8_t Data)
{
	USART_SendData(USART1 , (unsigned char)Data);
	while (USART_GetFlagStatus(USART1 , USART_FLAG_TXE)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


