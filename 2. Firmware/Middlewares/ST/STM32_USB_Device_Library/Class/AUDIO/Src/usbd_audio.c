/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

  /* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  - "stm32xxxxx_{eval}{discovery}_audio.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"
#include "pdm2pcm.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
extern I2S_HandleTypeDef hi2s2;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;

extern volatile uint8_t speaker_en;
extern volatile uint8_t out_mute;
extern volatile uint16_t out_volumn;
extern volatile uint32_t out_freq;
extern volatile uint16_t out_rd_ptr;  
extern volatile uint16_t out_wr_ptr; 
extern volatile uint8_t out_play_en;
extern volatile uint16_t out_remain_len;
extern uint32_t DataOutbuffer[]; 
extern uint32_t DataOutbuffer0[]; 
extern volatile uint8_t outbuf_xfrc;
extern volatile uint8_t feedbk_xfrc;
 
extern volatile uint8_t record_en;
extern volatile uint8_t collect_xfrc;
extern volatile uint8_t in_mute;
extern volatile uint16_t in_volumn;
extern volatile uint32_t in_freq;
extern volatile uint16_t in_rd_ptr;  
extern volatile uint16_t in_wr_ptr; 
extern volatile uint8_t in_play_en;
extern volatile uint8_t in_choose;
extern volatile uint16_t in_remain_len;
extern uint32_t DataInbuffer[];
extern uint32_t DataInbuffer1[];
extern uint32_t PDMDatabuffer0[];
extern volatile uint8_t collect_xfrc;
extern volatile uint8_t firstin_xfrc;

#ifdef FEEDBACK_ENABLE
extern uint8_t FeedBackbuffer[];
#endif

extern uint32_t cur_freq;
extern uint16_t xout_remain_len;
extern uint16_t xin_remain_len;
extern uint16_t xin_remain[200];
extern uint16_t xout_remain[200];
uint32_t k=0;
uint32_t r=0;

extern uint8_t state_out_full;
extern uint8_t state_out_freq;
extern uint8_t state_out_precv;
extern uint8_t state_out_flhfd;
extern uint8_t state_out_flhout;
extern uint8_t state_out_trans;
#ifndef FEEDBACK_ENABLE 
extern uint16_t state_out_dnadj;
extern uint16_t state_out_upadj;
#endif

extern uint8_t state_in_empty;
extern uint8_t state_in_freq;
extern uint8_t state_in_flhin;
extern uint8_t state_in_trans;
extern uint16_t state_in_upadj;
extern uint16_t state_in_dnadj;

extern uint16_t in_count;
extern uint16_t out_count;
extern uint16_t sof_count;
extern uint16_t feed_count;
extern uint32_t data_count;

#ifdef MYDEBUG		
extern uint16_t xcount;
extern uint32_t freq[];
extern uint32_t time[];
extern uint8_t  type[];
extern uint32_t out_max_calc;
extern uint32_t out_min_calc;
extern uint32_t out_max_real;
extern uint32_t out_min_real;
extern uint32_t in_max_real;
extern uint32_t in_min_real;
#endif
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */
#define AUDIO_SAMPLE_FREQ(frq)      		(uint8_t)(frq), \
																				(uint8_t)((frq >> 8)), \
																				(uint8_t)((frq >> 16))

#define SPEAKER_PACKET_SZE(siz)         (uint8_t)(siz & 0xFF), \
																				(uint8_t)((siz >> 8) & 0xFF)

#define MICPHONE_PACKET_SZE(siz)        (uint8_t)(siz & 0xFF), \
																				(uint8_t)((siz >> 8) & 0xFF)

#define FEEDBACK_PACKET_SZE(siz)				(uint8_t)(siz & 0xFF), \
																				(uint8_t)((siz >> 8) & 0xFF)

/**
  * @}
  */




/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetResource(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef  USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_TOTAL_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* USB Audio Configuration */
  USB_CONFIG_DESC_SIZE,                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_TOTAL_CONFIG_DESC_SIZE),   /* wTotalLength */
  HIBYTE(USB_TOTAL_CONFIG_DESC_SIZE),      
  0x03,                                 /* bNumInterfaces */                              //共3个接口
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0xC0,                                 /* bmAttributes  BUS Powred*/
  0x64,                                 /* bMaxPower = 200 mA*/
  /* 09 byte*/
  
  /* Standard USB Audio Control interface descriptor */                                   //输出接口
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_INTERFACE_CONTROL,             	/* bInterfaceNumber */                            //接口序号0                         
  0x00,                                 /* bAlternateSetting */                           //备用编号0
  0x00,                                 /* bNumEndpoints */                               //使用端点数目为0  
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                             //音频接口类0x01
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */                          //音频控制接口子类0x01
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */                          //不使用协议
  0x00,                                 /* iInterface */                                  //接口字符串引索
  /* 09 byte*/
  
  /* Class-specific USB Audio Control Interface Descriptor */      
  AUDIO_INTERFACE_DESC_SIZE + 1,   			/* bLength */                                     //2个流接口:输入和输出
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */                          //描述符子类:控制头
  0x00,                                 /* bcdADC */                                      //版本 1.00
  0x01,
  AUDIO_HEADER_SIZE,                    /* wTotalLength = 72*/
  0x00,
  0x02,                                 /* bInCollection */                               //流接口数量
  AUDIO_INTERFACE_IN,                 	/* baInterfaceNr */                               //属于本接口的流接口编号
  AUDIO_INTERFACE_OUT,                  /* baInterfaceNr */                               //属于本接口的流接口编号
  /* 10 byte*/

	/* USB Microphone Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */                          //控制输入终端
  AUDIO_IN_INTERM_ID,                   /* bTerminalID */                                 //终端ID 1
  0x01,                                 /* wTerminalType */                               //输入终端类型: 0x0201
  0x02,
  0x00,                                 /* bAssocTerminal */                              //与其结合在一起的输出终端:无
  IN_CHANNELS_NUM,                      /* bNrChannels */                                 //音频簇的通道数目:1
  IN_CHANNELS_CONFIG,                   /* wChannelConfig */                              //双声道 0:Left Front 1:Right Front 2:Center Front 3:Low Freq 4:Left Surround 5:Right Surround 6:Center Left 7:Center Right
	0x00,
  0x00,                                 /* iChannelNames */                               //逻辑通道的字符串索引号:无
  0x00,                                 /* iTerminal */                                   //输入终端的字符串索引号:无
  /* 12 byte*/
	
	/* USB Microphone Feature Unit Descriptor */                                         		//控制单元
#ifdef IN_STEREO
  AUDIO_FEATURE_UNIT_DESC_SIZE,     		/* bLength */
#else
  AUDIO_FEATURE_UNIT_DESC_SIZE - 1,     /* bLength */
#endif
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_IN_FU_ID,              					/* bUnitID */                                     //单元ID 2
  AUDIO_IN_INTERM_ID,                   /* bSourceID */                                   //连接的ID:3
  0x01,                                 /* bControlSize */                                //控制大小1个字节
  AUDIO_CONTROL_MUTE, 									/* bmaControls(Mute) */
#ifdef IN_STEREO
  0x00,                 								/* bmaControls(0)(Volume) */
  0x00,                 								/* bmaControls(1)(Volume) */
#else
  0x00,                 								/* bmaControls(0)(Volume) */
#endif
  0x00,                                 /* iTerminal */                                   //特征单元的字符串索引号:无
  /* 10 byte*/
  
	/* USB Microphone Output Terminal Descriptor */
  AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  AUDIO_IN_OUTTERM_ID,                  /* bTerminalID */                                 //终端ID 3
  0x01,                                 /* wTerminalType */                        			  //输出终端类型 0x0101 USB Streaming
  0x01,
  0x00,                                 /* bAssocTerminal */                              //与其结合在一起的输入终端:无
  AUDIO_IN_FU_ID,                       /* bSourceID */                                   //连接的ID:2
  0x00,                                 /* iTerminal */                                   //输出终端的字符串索引号:无
  /* 09 byte*/
	
  /* USB Speaker Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */                          //控制输入终端
  AUDIO_OUT_INTERM_ID,                  /* bTerminalID */                                 //终端ID 4
  0x01,                                 /* wTerminalType */                               //输入终端类型: 0x0101 USB Streaming
  0x01,
  0x00,                                 /* bAssocTerminal */                              //与其结合在一起的输出终端:无
  OUT_CHANNELS_NUM,                     /* bNrChannels */                                 //音频簇的通道数目:2
  OUT_CHANNELS_CONFIG,                  /* wChannelConfig */                              //左右声道
  0x00,
  0x00,                                 /* iChannelNames */                               //逻辑通道的字符串索引号:无
  0x00,                                 /* iTerminal */                                   //输入终端的字符串索引号:无
  /* 12 byte*/
  
  /* USB Speaker Feature Unit Descriptor */                                               //控制单元
  AUDIO_FEATURE_UNIT_DESC_SIZE,     		/* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_FU_ID,             					/* bUnitID */                                     //单元ID 5
  AUDIO_OUT_INTERM_ID,                  /* bSourceID */                                   //连接的ID:4
  0x01,                                 /* bControlSize */                                //控制大小1个字节
  AUDIO_CONTROL_MUTE, 									/* bmaControls(Mute) */
  AUDIO_CONTROL_VOLUME,                 /* bmaControls(0)(Volume) */
  AUDIO_CONTROL_VOLUME,                 /* bmaControls(1)(Volume) */
  0x00,                                 /* iTerminal */                                   //特征单元的字符串索引号:无
  /* 10 byte*/
  
  /*USB Speaker Output Terminal Descriptor */
  AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  AUDIO_OUT_OUTTERM_ID,                 /* bTerminalID */                                 //终端ID 6
  0x01,                                 /* wTerminalType  0x0301*/                        //输出终端类型 0x0301 speaker
  0x03,
  0x00,                                 /* bAssocTerminal */                              //与其结合在一起的输入终端:无
  AUDIO_OUT_FU_ID,                      /* bSourceID */                                   //连接的ID:5
  0x00,                                 /* iTerminal */                                   //输出终端的字符串索引号:无
  /* 09 byte*/
  
	//-------------------Microphone  interface---------------------//
	
	/* USB Microphone Standard AS Interface Descriptor Interface 1, Alternate Setting 0 */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */	                                    //输出接口
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_INTERFACE_IN,                  	/* bInterfaceNumber */                            //接口号:1
  0x00,                                 /* bAlternateSetting */                           //备用编号0
  0x00,                                 /* bNumEndpoints */                               //端点数量 0  
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                             //            
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */                      
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */                                  //接口字符串索引号:无
  /* 09 byte*/
  
  /* USB Microphone Standard AS Interface Descriptor Interface 1, Alternate Setting 1 */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_INTERFACE_IN,                  	/* bInterfaceNumber */                            //接口号:2              
  0x01,                                 /* bAlternateSetting */                           //备用编号1
  0x01,                                 /* bNumEndpoints */                               //端点数量 1 
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                             //音频接口类0x01
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */                          //音频控制接口子类0x02
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */                          //不使用协议
  0x00,                                 /* iInterface */                                  //接口字符串索引号:无
  /* 09 byte*/
  
  /* USB Microphone Class-specific Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_DESCRIPTOR,           /* bDescriptorSubtype */
  AUDIO_IN_OUTTERM_ID,                  /* bTerminalLink */                               //连接的ID:6
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/          //PCM
  0x00,
  /* 07 byte*/
  
  /* USB Microphone Class-specific AS Type I Format Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE - 1 + 3,    /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */                                 //TYPE I
  IN_CHANNELS_NUM,                     	/* bNrChannels */                                 //通道
  IN_SUBFRAMESIZE,                      /* bSubFrameSize :  2 Bytes per frame (16bits) */ //字节
  IN_BITRESOLUTION,                    	/* bBitResolution (16-bits per sample) */         //16bit
  0x01,                                 /* bSamFreqType only one frequency supported */   //支持的采样种类
  AUDIO_SAMPLE_FREQ(USBD_AUDIO_MICROPHONE_FREQ),   /* Audio sampling frequency coded on 3 bytes */   //具体采样说明
  /* 11 byte*/
  
  /* USB Microphone Standard AS ISOC Endpoint Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_IN_EP,                          /* bEndpointAddress 3 IN endpoint*/              //端点3输入
#ifdef ADAPTION_ENABLE
	USBD_EP_TYPE_ISOC | 0x08,        			/* bmAttributes */                                //同步端点 D3:2 01:异步;10:适应;11:同步;
	MICPHONE_PACKET_SZE(AUDIO_IN_PACKET + (AUDIO_IN_PACKET / 4)), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*SubSize(HalfWord)) */ //每个包的长度
#else
	USBD_EP_TYPE_ISOC,        						/* bmAttributes */                                //同步端点 D3:2 01:异步;10:适应;11:同步;
	MICPHONE_PACKET_SZE(AUDIO_IN_PACKET), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*SubSize(HalfWord)) */ //每个包的长度
#endif
	0x01,                                 /* bInterval */                                   //1ms
  0x00,                                 /* bRefresh */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/
  
  /* USB Microphone Class-specific AS ISOC Endpoint Descriptor*/
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_DESCRIPTOR,            /* bDescriptor */           
  0x00,                                 /* bmAttributes */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/
	
	//-------------------Speaker  interface---------------------//
	
  /* USB Speaker Standard AS Interface Descriptor Interface 1, Alternate Setting 0 */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */	                                    //输出接口
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_INTERFACE_OUT,                 	/* bInterfaceNumber */                            //接口号:2
  0x00,                                 /* bAlternateSetting */                           //备用编号0
  0x00,                                 /* bNumEndpoints */                               //端点数量 0  
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                             //            
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */                      
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */                                  //接口字符串索引号:无
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor Interface 1, Alternate Setting 1 */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_INTERFACE_OUT,                 /* bInterfaceNumber */                            //接口号:1              
  0x01,                                 /* bAlternateSetting */                           //备用编号1
#ifdef FEEDBACK_ENABLE
	0x02,                                 /* bNumEndpoints */                               //端点数量 2 
#else
  0x01,                                 /* bNumEndpoints */                               //端点数量 1 
#endif
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                             //音频接口类0x01
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */                          //音频控制接口子类0x02
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */                          //不使用协议
  0x00,                                 /* iInterface */                                  //接口字符串索引号:无
  /* 09 byte*/
  
  /* USB Speaker Class-specific Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_DESCRIPTOR,           /* bDescriptorSubtype */
  AUDIO_OUT_INTERM_ID,                  /* bTerminalLink */                               //连接的ID:1
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/          //PCM
  0x00,
  /* 07 byte*/
  
  /* USB Speaker Class-specific AS Type I Format Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE - 1 + 3,    /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */                                 //TYPE I
  OUT_CHANNELS_NUM,                     /* bNrChannels */                                 //双通道
  OUT_SUBFRAMESIZE,                     /* bSubFrameSize :  2 Bytes per frame (16bits) */ //2个字节
  OUT_BITRESOLUTION,                    /* bBitResolution (16-bits per sample) */         //16bit
  0x01,                                 /* bSamFreqType only one frequency supported */   //支持的采样种类
  AUDIO_SAMPLE_FREQ(USBD_AUDIO_SPEAKER_FREQ),   /* Audio sampling frequency coded on 3 bytes */   //具体采样说明
  /* 11 byte*/
  
  /* USB Speaker Standard AS ISOC Endpoint Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/              //端点1输出
#ifdef FEEDBACK_ENABLE
  USBD_EP_TYPE_ISOC | 0x04,             /* bmAttributes */                                //同步端点 D3:2 01:异步;10:适应;11:同步;
#else
	USBD_EP_TYPE_ISOC,             				/* bmAttributes */                                //同步端点 D3:2 01:异步;10:适应;11:同步;
#endif
  SPEAKER_PACKET_SZE(AUDIO_OUT_PACKET), /* wMaxPacketSize in Bytes (Freq(Samples)*OUT_CHANNELS(Stereo)*SubSize(HalfWord)) */ //每个包的长度
  0x01,                                 /* bInterval */                                   //1ms
  0x00,                                 /* bRefresh */
#ifdef FEEDBACK_ENABLE
  AUDIO_FB_EP,                          /* bSynchAddress */
#else
  0x00,                                 /* bSynchAddress */
#endif
  /* 09 byte*/
  
  /* USB Speaker Class-specific AS ISOC Endpoint Descriptor*/
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_DESCRIPTOR,            /* bDescriptor */
  0x00,                                 /* bmAttributes */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/

#ifdef FEEDBACK_ENABLE
	/* USB Speaker Standard AS ISOC Endpoint Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_FB_EP,                         	/* bEndpointAddress 2 IN endpoint*/              	//端点2输入
  USBD_EP_TYPE_ISOC | 0x10,							/* bmAttributes */                                //同步端点 D3:2 01:异步;10:适应;11:同步;
  FEEDBACK_PACKET_SZE(AUDIO_FB_PACKET), /* wMaxPacketSize in Bytes */											//每个包的长度
  0x01,                                 /* bInterval */                                   //1ms
  FEEDBACK_RATE,                        /* bRefresh */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/
#endif
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  USB_MAX_EP0_SIZE,
  USBD_MAX_NUM_INTERFACES,
  0x00,
};

#ifdef FEEDBACK_ENABLE	
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
void feedbakc_calc(uint32_t real_freq)
{
	// example
	// freq:24_100
	// 10.14fb = (24) << 14 + (100) << 4
/*
	96000 : 0x180000  95952 : 0x17FB80  95940 : 0x17FAC0  95614 : 0x17E660  
*/
	uint16_t freq_int = (uint16_t)(real_freq / 1000);
	uint32_t freq_data = real_freq * 16 + freq_int * 384;
	uint8_t *pfreq = (uint8_t *)(&freq_data);
	FeedBackbuffer[0] = pfreq[0];
	FeedBackbuffer[1] = pfreq[1];
	FeedBackbuffer[2] = pfreq[2];
}
#endif


void init_out_parameter(void)
{
	out_play_en = 0;
	out_rd_ptr = 0;
	out_wr_ptr = 0;
	out_remain_len = 0;
	outbuf_xfrc = 0;
}

void init_in_parameter(void)
{
	in_choose = 0;
	in_play_en = 0;
	in_rd_ptr = 0;
	in_wr_ptr = (uint16_t)(AUDIO_IN_BUF_WORD / 2);
	in_remain_len = 0;
	collect_xfrc = 0;
}
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
volatile uint8_t Audio_ClassData[sizeof (USBD_AUDIO_HandleTypeDef)];//llhhxy
static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef   *haudio;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
                 AUDIO_OUT_EP,
                 USBD_EP_TYPE_ISOC,
                 AUDIO_OUT_PACKET);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

#ifdef FEEDBACK_ENABLE	
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 AUDIO_FB_EP,
                 USBD_EP_TYPE_ISOC,
                 AUDIO_FB_PACKET);
  pdev->ep_in[AUDIO_FB_EP & 0xFU].is_used = 1U;
#endif
	
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 AUDIO_IN_EP,
                 USBD_EP_TYPE_ISOC,
#ifdef ADAPTION_ENABLE
                 (AUDIO_IN_PACKET + AUDIO_IN_PACKET / 4));
#else
                 AUDIO_IN_PACKET);
#endif
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = (void *)(Audio_ClassData);//USBD_malloc(sizeof (USBD_AUDIO_HandleTypeDef));//llhhxy

  if(pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }
  else
  {
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
    haudio->out_alt_setting = 0; 
		haudio->in_alt_setting = 0; 

    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Init(USBD_AUDIO_SPEAKER_FREQ, USBD_AUDIO_MICROPHONE_FREQ, 0, 0) != USBD_OK)//AUDIO_Init_HS
    {
      return USBD_FAIL;
    }

    /* Prepare Out endpoint to receive 1st packet */
		
    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, (uint8_t *)DataOutbuffer0, AUDIO_OUT_PACKET); 
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{

  /* Open EP OUT */
  USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;

#ifdef FEEDBACK_ENABLE		
	USBD_LL_CloseEP(pdev, AUDIO_FB_EP);
  pdev->ep_in[AUDIO_FB_EP & 0xFU].is_used = 0U;
#endif
	
	USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
   ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->DeInit(0U);//AUDIO_DeInit_HS
    //USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS : //类请求 
    switch (req->bRequest)
    {
			case AUDIO_REQ_GET_CUR:
				AUDIO_REQ_GetCurrent(pdev, req);//获取当前音量
				break;
			
			case AUDIO_REQ_GET_MIN:
				AUDIO_REQ_GetMinimum(pdev, req);//获取最小的音量
				break;
			
			case AUDIO_REQ_GET_MAX:
				AUDIO_REQ_GetMaximum(pdev, req);//获取最大的音量
				break;
			
			case AUDIO_REQ_GET_RES:
				AUDIO_REQ_GetResource(pdev, req);//获取分辨率
				break;
				
			case AUDIO_REQ_SET_CUR:
				AUDIO_REQ_SetCurrent(pdev, req);//设置当前状态
				break;
				
			default:
				USBD_CtlError (pdev, req);
				ret = USBD_FAIL; 
    }
    break;

  case USB_REQ_TYPE_STANDARD: //标准请求
    switch (req->bRequest)
    {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_CtlSendData (pdev, (uint8_t *)(void *)&status_info, 2U);
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_GET_DESCRIPTOR:
      if(HIBYTE(req->wValue) == AUDIO_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_AUDIO_CfgDesc + 18;
        len = MIN(USB_AUDIO_DESC_SIZE , req->wLength);

        USBD_CtlSendData (pdev, pbuf, len);
      }
      break;

    case USB_REQ_GET_INTERFACE :
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        if(req->wIndex == AUDIO_INTERFACE_OUT)
				{
					USBD_CtlSendData (pdev, (uint8_t *)&(haudio->out_alt_setting), 1);
				}
				else if(req->wIndex == AUDIO_INTERFACE_IN)
				{
					USBD_CtlSendData (pdev, (uint8_t *)&(haudio->in_alt_setting), 1);
				}
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE :
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
         if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
         {
						if(req->wIndex == AUDIO_INTERFACE_OUT)
						{
							haudio->out_alt_setting = (uint8_t)(req->wValue);
							if(haudio->out_alt_setting == 1)//启用工作设置
							{
								speaker_en = 1;
								feedbk_xfrc = 1;
							}
							else
							{
								speaker_en = 0;
								feedbk_xfrc = 0;
							}
							init_out_parameter();
						}
						else if(req->wIndex == AUDIO_INTERFACE_IN)
						{
							haudio->in_alt_setting = (uint8_t)(req->wValue);
							if(haudio->in_alt_setting == 1)//启用工作设置
							{
								record_en = 0x01;
								firstin_xfrc = 1;
							}
							else
							{
								record_en = 0x00;
								firstin_xfrc = 0;
							}
							init_in_parameter();
						}
         }
         else
         {
           /* Call the error management function (command will be nacked */
           USBD_CtlError (pdev, req);
           ret = USBD_FAIL;
         }
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;
  default:
    USBD_CtlError (pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return ret;
}

/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  /* In this driver, to simplify code, only SET_CUR request is managed */
  if (haudio->control.cmd == AUDIO_REQ_SET_FREQ)//SET_FREQ命令
  {
		//设置定时器频率
		if (haudio->control.unit == AUDIO_OUT_EP)//Speaker Uint
    {
//			haudio->control.data[3] = 0;
//			out_freq = *((uint32_t *)(haudio->control.data));
			out_freq = ((uint32_t)(haudio->control.data[2]) << 16);
			out_freq += ((uint16_t)(haudio->control.data[1]) << 8);
			out_freq += haudio->control.data[0];
		}
		else if (haudio->control.unit == AUDIO_IN_EP)//Microphone Uint
		{
//			haudio->control.data[3] = 0;
//			in_freq = *((uint32_t *)(haudio->control.data));
			in_freq = ((uint32_t)(haudio->control.data[2]) << 16);
			in_freq += ((uint16_t)(haudio->control.data[1]) << 8);
			in_freq += haudio->control.data[0];
		}
	}
	else if (haudio->control.cmd == AUDIO_REQ_SET_VOLU)//SET_VOLU命令
	{
    if (haudio->control.unit == AUDIO_OUT_FU_ID)//Speaker Uint
    {
			if(HIBYTE(haudio->control.type) == AUDIO_CS_MUTE)
			{
				out_mute = haudio->control.data[0];
				((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->MuteCtl(out_mute); //AUDIO_MuteCtl_FS 01:静音 00:非静音   
			}
			else if(HIBYTE(haudio->control.type) == AUDIO_CS_VOLUME)
			{
				out_volumn = ((uint16_t)(haudio->control.data[1]) << 8) + haudio->control.data[0];
				((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->VolumeCtl(out_volumn);
			}
    }
    else if (haudio->control.unit == AUDIO_IN_FU_ID)//Microphone Uint
    {
			if(HIBYTE(haudio->control.type) == AUDIO_CS_MUTE)
			{
				in_mute = haudio->control.data[0];
				((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->MuteCtl(in_mute); //AUDIO_MuteCtl_FS 01:静音 00:非静音   
			}
			else if(HIBYTE(haudio->control.type) == AUDIO_CS_VOLUME)
			{
				in_volumn = ((uint16_t)(haudio->control.data[1]) << 8) + haudio->control.data[0];
				((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->VolumeCtl(in_volumn);
			}
    }
		haudio->control.cmd = 0;
		haudio->control.len = 0;
  } 

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}

/**
  * @brief  AUDIO_REQ_GetMinimum
  *         Handles the GET_MIN Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  memset(haudio->control.data, 0, req->wLength);
	
	if(HIBYTE(req->wValue) == AUDIO_CS_VOLUME)//Volumn
	{
		if(HIBYTE(req->wIndex) == AUDIO_OUT_FU_ID)//Speaker Uint
		{
			haudio->control.data[0] = 0x00;
			haudio->control.data[1] = 0x80;
		}
		else if(HIBYTE(req->wIndex) == AUDIO_IN_FU_ID)//Microphone Uint
		{
			haudio->control.data[0] = 0x00;
			haudio->control.data[1] = 0x80;
		}
	}
	
	/* Send the current mute state */
	USBD_CtlSendData (pdev, 
										haudio->control.data,
										req->wLength);
}

/**
  * @brief  AUDIO_REQ_GetMaximum
  *         Handles the GET_MAX Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  memset(haudio->control.data, 0, req->wLength);
	
	if(HIBYTE(req->wValue) == AUDIO_CS_VOLUME)//Volumn
	{
		if(HIBYTE(req->wIndex) == AUDIO_OUT_FU_ID)//Speaker Uint
		{
			haudio->control.data[0] = 0xFF;
			haudio->control.data[1] = 0x7F;
		}
		else if(HIBYTE(req->wIndex) == AUDIO_IN_FU_ID)//Microphone Uint
		{
			haudio->control.data[0] = 0xFF;
			haudio->control.data[1] = 0x7F;
		}
	}
	
	/* Send the current mute state */
	USBD_CtlSendData (pdev, 
										haudio->control.data,
										req->wLength);
}

/**
  * @brief  AUDIO_REQ_GetResource
  *         Handles the GET_RES Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetResource(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  memset(haudio->control.data, 0, req->wLength);
	
	if(HIBYTE(req->wValue) == AUDIO_CS_VOLUME)//Volumn
	{
		if(HIBYTE(req->wIndex) == AUDIO_OUT_FU_ID)//Speaker Uint
		{
			haudio->control.data[0] = 0x01;
			haudio->control.data[1] = 0x00;
		}
		else if(HIBYTE(req->wIndex) == AUDIO_IN_FU_ID)//Microphone Uint
		{
			haudio->control.data[0] = 0x01;
			haudio->control.data[1] = 0x00;
		}
	}
	
	/* Send the current mute state */
	USBD_CtlSendData (pdev, 
										haudio->control.data,
										req->wLength);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{ 
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, haudio->control.data, req->wLength);    
    
    haudio->control.type = req->wValue;  /* Set the request target type */ 
		/********* HIBYTE(req->wValue) 0x01:mute;        0x02:volume        *********/
		/********* LOBYTE(req->wValue) 0x01:left channel;0x02:right channel *********/
    switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
    {
			case USB_REQ_RECIPIENT_ENDPOINT://2
				haudio->control.cmd = AUDIO_REQ_SET_FREQ;     /* Set the request value */
				haudio->control.unit = LOBYTE(req->wIndex);  /* Set the request target unit */
				break;
					
			case USB_REQ_RECIPIENT_INTERFACE://1
				haudio->control.cmd = AUDIO_REQ_SET_VOLU;     /* Set the request value */
				haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
				break;
			
			case USB_REQ_RECIPIENT_DEVICE://0
				break;

			default:
				break;
    }
    haudio->control.len = req->wLength;          /* Set the request data length */
  }
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  memset(haudio->control.data, 0, req->wLength);
	
	switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
	{
		case USB_REQ_RECIPIENT_ENDPOINT://2
			if(HIBYTE(req->wIndex) == AUDIO_OUT_EP)//Speaker EP
			{
				uint8_t freq[3] = {AUDIO_SAMPLE_FREQ(USBD_AUDIO_SPEAKER_FREQ)};
				haudio->control.data[0] = freq[0];
				haudio->control.data[1] = freq[1];
				haudio->control.data[2] = freq[2];
			}
			else if(HIBYTE(req->wIndex) == AUDIO_IN_EP)//Microphone EP
			{
				uint8_t freq[3] = {AUDIO_SAMPLE_FREQ(USBD_AUDIO_MICROPHONE_FREQ)};
				haudio->control.data[0] = freq[0];
				haudio->control.data[1] = freq[1];
				haudio->control.data[2] = freq[2];
			}
			break;
				
		case USB_REQ_RECIPIENT_INTERFACE://1
			if(HIBYTE(req->wValue) == AUDIO_CS_MUTE)//Mute
			{
				if(HIBYTE(req->wIndex) == AUDIO_OUT_FU_ID)//Speaker Uint
				{
					haudio->control.data[0] = out_mute;
				}
				else if(HIBYTE(req->wIndex) == AUDIO_IN_FU_ID)//Microphone Uint
				{
					haudio->control.data[0] = in_mute;
				}
			}
			else if(HIBYTE(req->wValue) == AUDIO_CS_VOLUME)//Volumn
			{
				if(HIBYTE(req->wIndex) == AUDIO_OUT_FU_ID)//Speaker Uint
				{
					haudio->control.data[0] = (uint8_t)(out_volumn);
					haudio->control.data[1] = (uint8_t)(out_volumn >> 8);
				}
				else if(HIBYTE(req->wIndex) == AUDIO_IN_FU_ID)//Microphone Uint
				{
					haudio->control.data[0] = (uint8_t)(in_volumn);
					haudio->control.data[1] = (uint8_t)(in_volumn >> 8);
				}
			}
			break;
			
		case USB_REQ_RECIPIENT_DEVICE://0
			break;

		default:
			break;
	}
		
	/* Send the current mute state */
	USBD_CtlSendData (pdev, 
										haudio->control.data,
										req->wLength);
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return 0;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev)
{
	sof_count++;
#ifdef FEEDBACK_ENABLE
	if(feedbk_xfrc == 1)
	{
		if(USBD_OK != USBD_LL_FlushEP(pdev, AUDIO_OUT_EP))
		{
			state_out_flhout++;
		}
		if(USBD_OK != USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, (uint8_t *)DataOutbuffer0, (uint16_t)(AUDIO_OUT_PACKET)))
		{
			state_out_precv++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		
		feedbakc_calc(USBD_AUDIO_SPEAKER_FREQ);
		if(USBD_OK != USBD_LL_FlushEP(pdev, AUDIO_FB_EP))
		{
			state_out_flhfd++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		if(USBD_OK != USBD_LL_Transmit(pdev, AUDIO_FB_EP, FeedBackbuffer, (uint16_t)(AUDIO_FB_PACKET)))
		{
			state_out_trans++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}	
		feedbk_xfrc = 0;
	}
	else if(feedbk_xfrc == 2)
	{
		feedbakc_calc(cur_freq);
		if(USBD_OK != USBD_LL_FlushEP(pdev, AUDIO_FB_EP))
		{
			state_out_flhfd++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		if(USBD_OK != USBD_LL_Transmit(pdev, AUDIO_FB_EP, FeedBackbuffer, (uint16_t)(AUDIO_FB_PACKET)))
		{
			state_out_trans++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		feedbk_xfrc = 0;
	}
			
#endif

	if(firstin_xfrc == 1)
	{
		if(USBD_OK != USBD_LL_FlushEP(pdev, AUDIO_IN_EP))
		{
			state_in_flhin++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if(USBD_OK != USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t *)DataInbuffer1, (uint16_t)(AUDIO_IN_PACKET)))
		{
			state_in_trans++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		firstin_xfrc = 0;
	}

  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index //端点索引
  * @retval status
  */
uint16_t cur_slen;
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == (AUDIO_IN_EP & 0x7F))//0x03
	{
		if(record_en == 0)
			return USBD_OK;
		
		uint16_t sendlen = (uint16_t)(AUDIO_IN_PACKET / 4); //96
		in_count++;
		//Calc Length
		HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		in_remain_len = (uint16_t)(in_wr_ptr - in_rd_ptr);
		if(in_play_en == 1)
			in_remain_len += (uint16_t)((PDM_IN_PACKET / 2 - (uint16_t)(__HAL_DMA_GET_COUNTER(&hdma_sai1_a))) / 4);//得到当前还剩余多少个数据
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		
		//Save Length
		xin_remain_len = in_remain_len;
		
		//Check Record
		if(in_play_en == 0)
		{
			if(xin_remain_len <= (uint16_t)(AUDIO_IN_PACKET / 4 * 2)) //192
			{
				in_rd_ptr = 0;
				xin_remain_len = (uint16_t)(AUDIO_IN_BUF_WORD / 2);
				HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)PDMDatabuffer0, (uint16_t)(PDM_IN_PACKET / 2)); //512bytes
				in_play_en = 1;
			}
		}
		
		if ((600199 >= r)&&(r >= 600000))
		{
			xin_remain[r-600000]=xin_remain_len;
		}
		r++;
		
		//Check Empty
		if((uint16_t)(in_wr_ptr - in_rd_ptr) <= (uint16_t)(AUDIO_IN_PACKET / 4))//EMPTY  //96
		{
			init_in_parameter();
#ifdef MYDEBUG	
			time[xcount] = HAL_GetTick();
			type[xcount] = 0x12;
			xcount++;
			if(xcount == 64)
				xcount = 63;
#endif
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			state_in_empty++;
		}
		
		//Manage Data Length 
		if(in_play_en == 1)
		{
#ifdef ADAPTION_ENABLE
			if(xin_remain_len >= (uint16_t)(RECORD_MAX_COUNT)) //≥5120
			{
				init_in_parameter();
#ifdef MYDEBUG	
				freq[xcount] = xin_remain_len;
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x14;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif	
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				state_in_freq++;
			}
			
			if(xin_remain_len <= (uint16_t)(RECORD_MIN_COUNT)) //≤3072
			{
				init_in_parameter();
#ifdef MYDEBUG	
				freq[xcount] = xin_remain_len;
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x14;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif	
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				state_in_freq++;
			}
			
			uint16_t diff;
			if(xin_remain_len > (uint16_t)(AUDIO_IN_BUF_WORD / 2))  //数据量多于半满？
			{
				diff = (xin_remain_len - (uint16_t)(AUDIO_IN_BUF_WORD / 2)) / 8;  //多于半满的数据量缩小8倍
				if(diff > 24)   //多出的数据量>192words
					diff = 24;  
				sendlen +=  diff; //96<sendlen=<120
			}
			else
			{
				diff = ((uint16_t)(AUDIO_IN_BUF_WORD / 2) - xin_remain_len) / 8;
				if(diff > 24)
					diff = 24;
				sendlen -= diff;  //72<=sendlen=<96
			}
#else
			if(xin_remain_len > (uint16_t)(RECORD_UP_COUNT))
			{
				if(xin_remain_len >= (uint16_t)(RECORD_MAX_COUNT))
				{
					init_in_parameter();
#ifdef MYDEBUG	
					freq[xcount] = xin_remain_len;
					time[xcount] = HAL_GetTick();
					type[xcount] = 0x14;
					xcount++;
					if(xcount == 64)
						xcount = 63;
#endif	
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
					state_in_freq++;
				}
				else
				{
					in_rd_ptr++;
					state_in_upadj++;
				}
			}
			
			if(xin_remain_len < (uint16_t)(RECORD_DN_COUNT))
			{
				if(xin_remain_len <= (uint16_t)(RECORD_MIN_COUNT))
				{
					init_in_parameter();
#ifdef MYDEBUG	
					freq[xcount] = xin_remain_len;
					time[xcount] = HAL_GetTick();
					type[xcount] = 0x14;
					xcount++;
					if(xcount == 64)
						xcount = 63;
#endif	
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
					state_in_freq++;
				}
				else
				{
					in_rd_ptr--;
					state_in_dnadj++;
				}
			}
#endif
		
#ifdef MYDEBUG			
			//Save Infromation
			if(xin_remain_len < in_min_real)
				in_min_real = xin_remain_len;
			if(xin_remain_len > in_max_real)
				in_max_real = xin_remain_len;
#endif
		}	
		
		//Copy Data
		cur_slen = sendlen;
		
		for(uint16_t i = 0; i < sendlen; i++)
		{
			DataInbuffer1[i] = DataInbuffer[in_rd_ptr & (uint16_t)(AUDIO_IN_BUF_WORD - 1)];
			in_rd_ptr++;
		}
		
		//Transfer Record Packet
		if(USBD_OK != USBD_LL_FlushEP(pdev, AUDIO_IN_EP))     //刷新低级驱动程序的端点,AUDIO_IN_EP:0x83
		{
			state_in_flhin++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		if(USBD_OK != USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t *)DataInbuffer1, (uint16_t)(sendlen * 4)))  //通过端点传输数据
		{
			state_in_trans++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	}
#ifdef FEEDBACK_ENABLE
	if (epnum == (AUDIO_FB_EP & 0x7F))//0x02
	{
		feed_count++;
//		feedbk_xfrc = 2;
	}
#endif
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
uint32_t epstatus;
static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == AUDIO_OUT_EP)//0x01
  {
		if(speaker_en == 0)
			return USBD_OK;
		
		out_count++;
		uint16_t recv_len = (uint16_t)(USBD_LL_GetRxDataSize(pdev, epnum)) / 4;
		if(recv_len != 0)
		{
			data_count += recv_len;
			//Copy Data
			for(uint16_t i = 0; i < recv_len; i++)
			{
				DataOutbuffer[out_wr_ptr & (uint16_t)(AUDIO_OUT_BUF_WORD - 1)] = DataOutbuffer0[i];
				out_wr_ptr++;
			}
			
			//Check Full
			if((uint16_t)(out_wr_ptr - out_rd_ptr) >= (uint16_t)(AUDIO_OUT_BUF_WORD - AUDIO_OUT_PACKET / 4))//FULL 2^14-120
			{
				init_out_parameter();
#ifdef MYDEBUG	
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x01;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
				state_out_full++;
			}

			//Calc Length
			HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
			out_remain_len = (uint16_t)(out_wr_ptr - out_rd_ptr);
			if(out_play_en == 1)
				out_remain_len -= (uint16_t)(SPEAKER_OUT_WORD - (uint16_t)(__HAL_DMA_GET_COUNTER(&hdma_spi2_tx)) / 2);
			HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
			
			xout_remain_len = out_remain_len;
			
			//Check Play
			if(out_play_en == 0)
			{
				if(xout_remain_len >= (uint16_t)(AUDIO_OUT_BUF_WORD / 2))
				{
					out_wr_ptr = (uint16_t)(AUDIO_OUT_BUF_WORD / 2);
					xout_remain_len = (uint16_t)(AUDIO_OUT_BUF_WORD / 2);
					HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)DataOutbuffer, (uint16_t)(SPEAKER_OUT_WORD * 2));//传输的数据大小为16bit,SPEAKER_OUT_WORD表示类型为word
					out_play_en = 1;
				}
			}
		
		 if ((600199 >= k)&&(k >= 600000))
		 {
			xout_remain[k-600000]=xout_remain_len;
		 }
		 k++;
		}	
		
		//for test
		epstatus = ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)(((PCD_HandleTypeDef *)(pdev->pData))->Instance) + USB_OTG_OUT_ENDPOINT_BASE + ((epnum) * USB_OTG_EP_REG_SIZE)))->DOEPCTL;
		
		//Prepare Recv	
		if(USBD_OK != USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, (uint8_t *)DataOutbuffer0, (uint16_t)(AUDIO_OUT_PACKET)))
		{
			state_out_precv++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		
#ifdef FEEDBACK_ENABLE
#ifdef FEEDBACK_MAINLOOP
		outbuf_xfrc = 1;
#else
		//Manage Data Length 
		cur_freq = USBD_AUDIO_SPEAKER_FREQ;
		if(out_play_en == 1)
		{
			cur_freq += AUDIO_OUT_BUF_WORD;
			cur_freq -= xout_remain_len ;//* 2;//use xout_remain_len bacause out_remain_len is always change

#ifdef MYDEBUG	
			//Save Infromation
			if(cur_freq > out_max_real)
				out_max_real = cur_freq;
			if(cur_freq < out_min_real)
				out_min_real = cur_freq;
#endif
			//Eliminating jitter
//			if((cur_freq >= (USBD_AUDIO_SPEAKER_FREQ - 12)) && (cur_freq <= (USBD_AUDIO_SPEAKER_FREQ + 12)))
//			{
//				cur_freq = USBD_AUDIO_SPEAKER_FREQ;
//			}
			
			if(cur_freq >= FEED_AUDIO_MAX_FREQ) //>=120k
			{
//				init_out_parameter();
#ifdef MYDEBUG	
				freq[xcount] = cur_freq;
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x04;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif	
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				cur_freq = FEED_AUDIO_MAX_FREQ;
				state_out_freq++;
			}
			
			if(cur_freq <= FEED_AUDIO_MIN_FREQ) //<=84k
			{
//				init_out_parameter();
#ifdef MYDEBUG	
				freq[xcount] = cur_freq;
				time[xcount] = HAL_GetTick();
				type[xcount] = 0x04;
				xcount++;
				if(xcount == 64)
					xcount = 63;
#endif	
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				cur_freq = FEED_AUDIO_MIN_FREQ;
				state_out_freq++;
			}
	
#ifdef MYDEBUG	
			//Save Infromation
			if(cur_freq > out_max_calc)
				out_max_calc = cur_freq;
			if(cur_freq < out_min_calc)
				out_min_calc = cur_freq;
#endif
		}	

		//Transfer FeedBack Packet
		feedbakc_calc(cur_freq);
		if(USBD_OK != USBD_LL_FlushEP(pdev, AUDIO_FB_EP))
		{
			state_out_flhfd++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		if(USBD_OK != USBD_LL_Transmit(pdev, AUDIO_FB_EP, FeedBackbuffer, (uint16_t)(AUDIO_FB_PACKET)))
		{
			state_out_trans++;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}	
#endif	
#else
		if(out_play_en == 1)
		{
			if(xout_remain_len > (uint16_t)(SPEAK_UP_COUNT))
			{
				if(xout_remain_len >= (uint16_t)(SPEAK_MAX_COUNT))
				{
//					init_out_parameter();
#ifdef MYDEBUG	
					freq[xcount] = xout_remain_len;
					time[xcount] = HAL_GetTick();
					type[xcount] = 0x04;
					xcount++;
					if(xcount == 64)
						xcount = 63;
#endif	
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				}
				
				out_wr_ptr --;
				state_out_upadj++;
			}
			
			if(xout_remain_len < (uint16_t)(SPEAK_DN_COUNT))
			{
				if(xout_remain_len <= (uint16_t)(SPEAK_MIN_COUNT))
				{
//					init_out_parameter();
#ifdef MYDEBUG	
					freq[xcount] = xout_remain_len;
					time[xcount] = HAL_GetTick();
					type[xcount] = 0x04;
					xcount++;
					if(xcount == 64)
						xcount = 63;
#endif	
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				}
				
				out_wr_ptr ++;
				state_out_dnadj++;
			}
		}
#endif		
  }
  
  return USBD_OK;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
