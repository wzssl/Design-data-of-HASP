/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"
#include "usbd_audio.h"
#include "usb_device.h"

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
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
volatile uint8_t speaker_en;
volatile uint8_t out_mute;
volatile uint16_t out_volumn;
volatile uint32_t out_freq;
volatile uint16_t out_rd_ptr;  
volatile uint16_t out_wr_ptr; 
volatile uint8_t out_play_en;
volatile uint16_t out_remain_len;
uint32_t DataOutbuffer[AUDIO_OUT_BUF_WORD];   //16384 WORD@96K
uint32_t DataOutbuffer0[AUDIO_OUT_PACKET / 4];//120 WORD@96K 
volatile uint8_t outbuf_xfrc; 
volatile uint8_t feedbk_xfrc;
uint16_t xin_remain[200];
uint16_t xout_remain[200];

volatile uint8_t record_en;
volatile uint8_t in_mute;
volatile uint16_t in_volumn;
volatile uint32_t in_freq; 
volatile uint16_t in_rd_ptr;  
volatile uint16_t in_wr_ptr; 
volatile uint8_t in_play_en;
volatile uint8_t in_choose;
volatile uint16_t in_remain_len;
uint32_t DataInbuffer[AUDIO_IN_BUF_WORD];//8192 WORD@96K
#ifdef NOT512
uint32_t DataInbuffer0[PDM_IN_PACKET / 2 / 4];//128 WORD@96K
#endif
#ifdef ADAPTION_ENABLE
uint32_t DataInbuffer1[AUDIO_IN_PACKET / 4 + AUDIO_IN_PACKET / 16];//96+24=120 WORD@96K  
#else
uint32_t DataInbuffer1[AUDIO_IN_PACKET / 4];
#endif
uint32_t PDMDatabuffer0[PDM_IN_PACKET / 4];//256 WORD@96K   
uint32_t PDMDatabuffer1[PDM_IN_PACKET / 4];//256 WORD@96K
volatile uint8_t collect_xfrc;
volatile uint8_t firstin_xfrc;


#ifdef FEEDBACK_ENABLE 
uint8_t FeedBackbuffer[AUDIO_FB_PACKET];
#endif

uint32_t cur_freq;
uint16_t xout_remain_len;
uint16_t xin_remain_len;

uint8_t state_out_full;
uint8_t state_out_empty;
uint8_t state_out_freq;
uint8_t state_out_dmae;
uint8_t state_out_precv;
uint8_t state_out_flhfd;
uint8_t state_out_flhout;
uint8_t state_out_trans;
uint16_t state_out_dnadj;
uint16_t state_out_upadj;
uint32_t state_out_nak;

uint8_t state_in_full;
uint8_t state_in_empty;
uint8_t state_in_freq;
uint8_t state_in_dmae;
uint8_t state_in_flhin;
uint8_t state_in_trans;
uint16_t state_in_upadj;
uint16_t state_in_dnadj;

uint16_t in_count = 0;
uint16_t out_count = 0;
uint16_t sof_count= 0;
uint16_t feed_count = 0;
uint32_t play_count = 0;
uint16_t sai_count = 0;
uint32_t data_count = 0;

uint16_t xin_count;
uint16_t xout_count;
uint16_t xsof_count;
uint16_t xfeed_count;
uint32_t xplay_count;
uint16_t xsai_count;
uint32_t xdata_count;

#ifdef MYDEBUG		
uint16_t xcount;
uint32_t freq[64];
uint32_t time[64];
uint8_t  type[64];
uint32_t out_max_calc;
uint32_t out_min_calc;
uint32_t out_max_real;
uint32_t out_min_real;
uint32_t in_max_real;
uint32_t in_min_real;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S2_Init(void);
static void MX_SAI1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if(hi2s->Instance == SPI2)
	{
		if(out_play_en == 1)
		{
			out_rd_ptr += (uint16_t)(SPEAKER_OUT_WORD);
			
			//Check Empty
			if((uint16_t)(out_wr_ptr - out_rd_ptr) <= (uint16_t)(AUDIO_OUT_PACKET / 4))//EMPTY
			{
				init_out_parameter();
#ifdef MYDEBUG	
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x02;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				state_out_empty++;
			}
			else
			{
				//DMA transfer
				if(HAL_OK != HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)(&DataOutbuffer[out_rd_ptr & (uint16_t)(AUDIO_OUT_BUF_WORD - 1)]), (uint16_t)(SPEAKER_OUT_WORD * 2)))//DMAE
				{
#ifdef MYDEBUG	
					time[xcount] = HAL_GetTick();
					type[xcount] = 0x03;
					xcount++;
					if(xcount == 64)
						xcount = 63;
#endif
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
					state_out_dmae++;
				}
				else
					play_count++;
			}
		}
	}
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	if(hsai->Instance == SAI1_Block_A)
	{
		if(in_play_en == 0)
			return;
		
#ifdef IN_STEREO
		if(in_choose == 0x00)
		{
			in_choose = 0x01;
			
			//DMA transfer
			if(HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)PDMDatabuffer1, (uint16_t)(PDM_IN_PACKET / 2)))//DMAE
			{
#ifdef MYDEBUG	
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x13;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				state_in_dmae++;
			}
			else
				sai_count++;
		}
		else
		{
			in_choose = 0x00;
			
			//DMA transfer
			if(HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)PDMDatabuffer0, (uint16_t)(PDM_IN_PACKET / 2)))//DMAE
			{
#ifdef MYDEBUG	
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x13;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				state_in_dmae++;
			}
			else
				sai_count++;
		}
#else
#endif
		collect_xfrc = 1;
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
  MX_CRC_Init();
  MX_I2S2_Init();
  MX_SAI1_Init();
  MX_TIM8_Init();
  MX_PDM2PCM_Init();
	
	in_mute = 0;
	in_volumn = 0;
	record_en = 0x00;
	
	out_mute = 0;
	out_volumn = 0;
	speaker_en = 0;
	
	outbuf_xfrc = 0;
	feedbk_xfrc = 0;
	collect_xfrc = 0;
	firstin_xfrc = 0;

  /* USER CODE BEGIN 2 */
	state_out_full = 0;
	state_out_empty = 0;
	state_out_freq = 0;
	state_out_dmae = 0;
	state_out_precv = 0;
	state_out_flhfd = 0;
	state_out_flhout = 0;
	state_out_trans = 0;
	state_out_nak = 0;
	state_out_dnadj = 0;
	state_out_upadj = 0;
	
	state_in_full = 0;
	state_in_empty = 0;
	state_in_freq = 0;
	state_in_dmae = 0;
	state_in_flhin = 0;
	state_in_trans = 0;
	state_in_upadj = 0;
	state_in_dnadj = 0;
	
#ifdef MYDEBUG
	out_max_calc = USBD_AUDIO_SPEAKER_FREQ;
	out_min_calc = USBD_AUDIO_SPEAKER_FREQ;
	out_max_real = USBD_AUDIO_SPEAKER_FREQ;
	out_min_real = USBD_AUDIO_SPEAKER_FREQ;
	
	in_max_real = (uint16_t)(AUDIO_IN_BUF_WORD / 2);
	in_min_real = (uint16_t)(AUDIO_IN_BUF_WORD / 2);
#endif

  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
#if 0
		if(record_en == 0)
		{
			in_count = 0;
			out_count = 0;
			sof_count = 0;
			feed_count = 0;
			play_count = 0;
			sai_count = 0;
			data_count = 0;
			HAL_Delay(1000);
			xin_count = in_count;
			xout_count = out_count;
			xsof_count = sof_count;
			xfeed_count = feed_count;
			xplay_count = play_count;
			xsai_count = sai_count;
			xdata_count = data_count;
		}
#endif
    /* USER CODE BEGIN 3 */

		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		}
		
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
		{
		}
		
		if(collect_xfrc == 1)//SAI COLLECT
		{	
			collect_xfrc = 0;
			
			if(in_play_en == 1)
			{
				uint16_t i;
				//PDM to PCM
#ifdef NOT512
				if(in_choose == 0x01)
					PDM_To_PCM((uint16_t *)PDMDatabuffer0, (uint16_t *)(DataInbuffer0));
				else
					PDM_To_PCM((uint16_t *)PDMDatabuffer1, (uint16_t *)(DataInbuffer0));
				
				//Copay Data 
				for(i = 0; i < (uint16_t)(PDM_IN_PACKET / 2 / 4); i++)//128 WORD
				{
					DataInbuffer[in_wr_ptr & (uint16_t)(AUDIO_IN_BUF_WORD - 1)] = DataInbuffer0[i];
					in_wr_ptr++;
				}
#else
				if(in_choose == 0x01)
					PDM_To_PCM((uint16_t *)PDMDatabuffer0, (uint16_t *)(&DataInbuffer[in_wr_ptr & (uint16_t)(AUDIO_IN_BUF_WORD - 1)]));
				else
					PDM_To_PCM((uint16_t *)PDMDatabuffer1, (uint16_t *)(&DataInbuffer[in_wr_ptr & (uint16_t)(AUDIO_IN_BUF_WORD - 1)]));
				
				in_wr_ptr += (uint16_t)(PDM_IN_PACKET / 2 / 4);
#endif
				
				//Check Full
				if((uint16_t)(in_wr_ptr - in_rd_ptr) >= (uint16_t)(AUDIO_IN_BUF_WORD - AUDIO_IN_PACKET / 4))//FULL
				{
					init_in_parameter();
#ifdef MYDEBUG	
					time[xcount] = HAL_GetTick();
					type[xcount] = 0x11;
					xcount++;
					if(xcount == 64)
						xcount = 63; 
#endif
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
					state_in_full++;
				}
			}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 350;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 24;
#if (USBD_AUDIO_SPEAKER_FREQ == 96000U)
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
#elif (USBD_AUDIO_SPEAKER_FREQ == 48000U)
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
#elif (USBD_AUDIO_SPEAKER_FREQ == 32000U)
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
#elif (USBD_AUDIO_SPEAKER_FREQ == 24000U)
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
#elif (USBD_AUDIO_SPEAKER_FREQ == 16000U)
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
#endif
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 8;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 16;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 125;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLI2SDivQ = (uint32_t)(4 * (96000 / USBD_AUDIO_MICROPHONE_FREQ));//[6.144M:96K/4] [3.072M:48K/8] [2.048M:32K/12]  [1.536M:24K/16] [1.024M:16K/24]
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = USBD_AUDIO_SPEAKER_FREQ;//I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.ClockSource = SAI_CLKSOURCE_NA;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.FrameInit.FrameLength = 32;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 16;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pins : GPIO_PIN_0 GPIO_PIN_1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins : PB1 PB2 PB3 PB4 
                           PB5 PB6 PB7 PB8 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
