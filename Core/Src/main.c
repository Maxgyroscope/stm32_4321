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
#define  initPacket 0x1BEEFFF9
#define  loaderModePacket 0x13F8FFF9

#define  loaderModePC 0x13FEFE01

#define  sizeone  8
#define  sizetwo  5
#define  sizetry  4
#define  sizefour 3
#define  FLASH_USER_START_ADDR 0x8004000
#define  FLASH_USER_END_ADDR 0x8021350
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
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
uint8_t RxData[8] = {0,0,0,0,0,0,0,0};
uint8_t TxData[8] = {0,0,0,0,0,0,0,0};
uint32_t address = 0x08004000;
CAN_RxHeaderTypeDef RxHeader;
int cnt = 3;
uint32_t start = 1;
volatile uint32_t now  = 10;
volatile uint32_t end  = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CRC_Init(void);
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
  MX_CAN1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  
  if( HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }
  
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    
  while (start < end) {
        HAL_Delay(500);
            CAN1_Tx(initPacket, TxData, sizeone);
            start++;
          }
   start_application();
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
CAN_FilterTypeDef canfilterconfig;

canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
canfilterconfig.FilterBank = 0;
canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
canfilterconfig.FilterIdHigh = 0x0000;// старша€ часть первого "регистра фильтра" эта основна€ 0x0100<<5;
canfilterconfig.FilterIdLow = 0x0000;// младша€ ч 1 "р ф"
canfilterconfig.FilterMaskIdHigh = 0x0000;// ст ч 2 "р ф" и эта основна€ 0x07F8<<5;
canfilterconfig.FilterMaskIdLow = 0x0000;// мл ч 2 "р ф"
canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
canfilterconfig.SlaveStartFilterBank = 0;

HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t crc32_calc(uint8_t *buffer, size_t size){
  static uint32_t crc32_value = 0;
  
  //__HAL_RCC_CRC_CLK_ENABLE();
  crc32_value = HAL_CRC_Calculate(&hcrc, (uint32_t *)buffer, size);
  //__HAL_RCC_CRC_CLK_DISABLE();
  
  return crc32_value;
}

void CAN1_Rx(void){
  
  //CAN_RxHeaderTypeDef RxHeader;
  
  uint8_t RxData[8];
  
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

void CAN1_Tx(int32_t dataExtId, uint8_t * TxData, int32_t size){

    uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.DLC = size;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.ExtId = dataExtId;        
        TxHeader.StdId= 0x00;
        TxHeader.RTR = CAN_RTR_DATA;

 if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox) != HAL_OK)
  {
    start_application();//если кака€-то херотень - мы валим в приложуху
    		Error_Handler();
  }
 //HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox);
 //sprintf(msg,"Message Transmitted\r\n");
 //HAL_UART_Transmit(&huart4,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
  if(RxHeader.ExtId == loaderModePC)//фильтр FID
  {
      canCheckRxData(RxData); 
  }
}

void canCheckRxData(uint8_t * aData)
{ static uint8_t L = 0;
  static uint8_t rxbuf[8];
  switch(aData[2])
                            {case 0x00:
                                   while(1)
                                            {
                                                  CAN1_Tx(initPacket, aData, sizeone);
                                            }
                                    return;
                                case 0xAA:
                                              aData[3] = 0xC20;
                                                aData[4] = 0x04;
                                                  CAN1_Tx(loaderModePacket, aData, sizetwo);
                                    return;
                                case 0x01:
                                    flash_erase((uint32_t)1);                             //должна происходить отчистка FLASH
                                             
                                    CAN1_Tx(loaderModePacket, aData, sizetry);
                                    return;
                                case 0x02:
                                      CAN1_Tx(loaderModePacket, aData, sizefour);
                                      __disable_irq();                                  //отключаем все прерывани€, переводим STM32 в посто€нныей приЄм данных
                                      
                                       uint8_t cnt_flash = 4;
                                       uint32_t crc32_value = 0;
                                       
                                      while(1)
                                      {
                                        uint16_t PackageCounter = 0;
                                       if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)==HAL_OK && RxHeader.ExtId == loaderModePC)
                                       {
                                               crc32_value = crc32_calc( (uint8_t*)address, address - 0x08004000);
                                               
                                               
                                              if (RxData[0] == 0x00 && RxData[1] == 0x00 && RxData[2] != 0x00 && RxData[3] != 0x00  && (PackageCounter == ((RxData[3]<<8) | RxData[2] )))
                                                {
                                                  CAN1_Tx(loaderModePacket, aData, sizefour);
                                                  start_application();
                                                  break;
                                                };
                                        
                                        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
                                        
                                        HAL_FLASH_Unlock();
                                      
                                         assert_param(IS_FLASH_ADDRESS(address));
                                          
                                            CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
                                            FLASH->CR |= FLASH_PSIZE_BYTE;// FLASH_PSIZE_WORD
                                            FLASH->CR |= FLASH_CR_PG;
                                            
                                            while (cnt_flash <8)
                                            {
                                             *(__IO uint8_t*)address = RxData[cnt_flash];//
                                              address+=0x01;
                                              cnt_flash++;
                                            }
                                            cnt_flash = 4;
                                            
                                           
                                            
                                        read_flash (0x08004000, rxbuf, 8);
                                        HAL_FLASH_Lock();
                                        
                                        PackageCounter++;
                                       }
                                      }
                                      
                                    return;
                            default:
                                      return;
                            }

}

void start_application(void)
{
    uint32_t appJumpAddress;
	uint32_t temp;
	void (*GoToApp)(void);

    __disable_irq();
   // __disable_interrupt();
	// Disable all interrupts
	NVIC->ICER[0] = 0xFFFFFFFF;
	NVIC->ICER[1] = 0xFFFFFFFF;
	NVIC->ICER[2] = 0xFFFFFFFF;

	// Clear pendings
	NVIC->ICPR[0] = 0xFFFFFFFF;
	NVIC->ICPR[1] = 0xFFFFFFFF;
	NVIC->ICPR[2] = 0xFFFFFFFF;

	// Stop sys tick
	SysTick->CTRL = 0;
        
        //Peripheral  risit disable
        //Peripheral  disable clock
        LL_RCC_SetSysClkSource(0x00000000U);
          while(LL_RCC_GetSysClkSource() != 0x00000000U)
            {
            };
        LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
        LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
        LL_RCC_PLL_Disable();
        LL_RCC_HSE_Disable();
        
        SCB->VTOR = FLASH_BASE + 0x4000; 
	appJumpAddress = *((volatile uint32_t*)(SCB->VTOR + 4));
	GoToApp = (void (*)(void))appJumpAddress;
        
        //NVIC_SetVectorTable( MAIN_APP_ADDR, 0 );      //перекидываем вектора
	__set_MSP(*((volatile uint32_t*) SCB->VTOR)); //stack pointer (to RAM) for USER app in this address
	GoToApp();
}

void flash_write(uint32_t address, uint8_t *data){
   
   uint8_t write_data[4] = {0};
   uint8_t copy_data[8] = {0};
   uint8_t cnt_flash = 0;
   
   //¬ыдел€ем последние [4...7] байта из data
   memcpy(copy_data,data,8);
   for (int i = 0; i < sizeof(write_data); i++) 
     memcpy(write_data[i],copy_data[i+4],1);

    HAL_FLASH_Unlock();
    
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP| FLASH_FLAG_OPERR| FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |FLASH_FLAG_PGPERR |FLASH_FLAG_PGSERR );
    
   while(cnt_flash < sizeof(write_data)) //записываем 4 байтовое слово 
   {
          assert_param(IS_FLASH_ADDRESS(address));
          /* If the previous operation is completed, proceed to program the new data */
          CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
          FLASH->CR |= FLASH_PSIZE_BYTE;// FLASH_PSIZE_WORD
          FLASH->CR |= FLASH_CR_PG;
          *(__IO uint32_t*)address = write_data[cnt_flash];// data[0]
          cnt_flash++; //перебор передаваемых байт, по 1 байту. но не пон€тно надо ли?
   }
      /*    
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,(uint64_t)write_data) == HAL_OK) //запись байтов
        {   
          HAL_FLASH_Lock();
          return;
        }
      else
      {
        HAL_FLASH_GetError ();
      }
      */
 HAL_FLASH_Lock();
}

void read_flash (uint32_t address, uint8_t* rxbuf, uint16_t numofwords)
{
  while(1)
  {
    *rxbuf = *(__IO uint32_t*)address;
    address += 0x04;
    rxbuf++;
    if(!(numofwords--)) break;
  }
};

void flash_erase(uint32_t index){
    
    /*
      HAL_FLASH_Unlock();
      FLASH->ACR|= FLASH_ACR_LATENCY_5WS;
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP| FLASH_FLAG_OPERR| FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |FLASH_FLAG_PGPERR |FLASH_FLAG_PGSERR);
    
      FLASH->CR|= FLASH_CR_SER;
      FLASH->CR|= FLASH_CR_PSIZE_1;
    
      FLASH->CR|= (index << FLASH_CR_SNB_Pos); //FLASH->CR|=((uint32_t)2 << FLASH_CR_SNB_Pos);
      FLASH->CR|= FLASH_CR_STRT;
      while((FLASH->SR&FLASH_SR_BSY) == 1) {};
      
      HAL_FLASH_Lock();// FLASH->CR = 0x80000000;
    */
  
     uint32_t ulBadBlocks = 0;
     FLASH_EraseInitTypeDef eraseInfo;
     
      eraseInfo.Banks = FLASH_BANK_1;
      eraseInfo.Sector = FLASH_SECTOR_1;
      eraseInfo.NbSectors = 11; // 0xFF - стереть все сектора, в противном случае стираетс€ указанное число
      eraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
      eraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
      
      HAL_FLASH_Unlock(); // –азблокировка Flash-пам€ти
      HAL_FLASHEx_Erase(&eraseInfo, &ulBadBlocks); // —тирание заданных секторов */
      HAL_FLASH_Lock(); // —нова блокировка Flash-пам€ти
      
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
