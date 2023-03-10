/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_audio.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */


/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */
#define MYDEBUG           

#define IN_STEREO																			1
#define FEEDBACK_ENABLE																1

#ifdef FEEDBACK_ENABLE
//#define FEEDBACK_MAINLOOP
#define ADAPTION_ENABLE																1
#endif


#define USBD_AUDIO_SPEAKER_FREQ     (96000U)
#ifdef IN_STEREO
#define USBD_AUDIO_MICROPHONE_FREQ  (96000U)
#else
#define USBD_AUDIO_MICROPHONE_FREQ  (96000U)
#endif

#if (USBD_AUDIO_SPEAKER_FREQ == 96000U)
#define SPEAKER_OUT_WORD															256U
#define AUDIO_OUT_BUF_WORD														16384U
#define AUDIO_IN_BUF_WORD															8192U
#elif (USBD_AUDIO_SPEAKER_FREQ == 48000U)
#define SPEAKER_OUT_WORD															128U
#define AUDIO_OUT_BUF_WORD														8192U
#define AUDIO_IN_BUF_WORD															4096U
#elif (USBD_AUDIO_SPEAKER_FREQ == 32000U)
#define SPEAKER_OUT_WORD															64U
#define AUDIO_OUT_BUF_WORD														8192U
#define AUDIO_IN_BUF_WORD															4096U
#elif (USBD_AUDIO_SPEAKER_FREQ == 24000U)
#define SPEAKER_OUT_WORD															32U
#define AUDIO_OUT_BUF_WORD														4096U
#define AUDIO_IN_BUF_WORD															2048U
#elif (USBD_AUDIO_SPEAKER_FREQ == 16000U)
#define SPEAKER_OUT_WORD															8U
#define AUDIO_OUT_BUF_WORD														4096U
#define AUDIO_IN_BUF_WORD															2048U
#endif


#define CTRL_OUT_EP                                   0x00
#define CTRL_IN_EP                                    0x80
#define AUDIO_OUT_EP                                  0x01
#define AUDIO_FB_EP                                   0x82
#define AUDIO_IN_EP                                   0x83

#ifdef IN_STEREO
#ifdef FEEDBACK_ENABLE
#define USB_TOTAL_CONFIG_DESC_SIZE                    0xCB
#else
#define USB_TOTAL_CONFIG_DESC_SIZE                    0xC2
#endif
#define AUDIO_HEADER_SIZE															0x48
#else
#ifdef FEEDBACK_ENABLE
#define USB_TOTAL_CONFIG_DESC_SIZE                    0xCA
#else
#define USB_TOTAL_CONFIG_DESC_SIZE                    0xC1
#endif
#define AUDIO_HEADER_SIZE															0x47
#endif

#define USB_AUDIO_DESC_SIZE                           0x09

#define USB_CONFIG_DESC_SIZE													0x09
#define AUDIO_INTERFACE_DESC_SIZE                     0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0C	
#define AUDIO_FEATURE_UNIT_DESC_SIZE									0x0A
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09

#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07

#define AUDIO_DESCRIPTOR_TYPE                         0x21
#define USB_DEVICE_CLASS_AUDIO                        0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02
#define AUDIO_PROTOCOL_UNDEFINED                      0x00
#define AUDIO_STREAMING_DESCRIPTOR                    0x01
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02
#define AUDIO_ENDPOINT_DESCRIPTOR                     0x01

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06

#define AUDIO_INTERFACE_CONTROL												0x00
#define AUDIO_INTERFACE_IN														0x01
#define AUDIO_INTERFACE_OUT														0x02

#define AUDIO_REQ_GET_CUR                             0x81
#define AUDIO_REQ_GET_MIN                             0x82
#define AUDIO_REQ_GET_MAX                             0x83
#define AUDIO_REQ_GET_RES                             0x84
#define AUDIO_REQ_GET_MEM                             0x85

#define AUDIO_REQ_SET_CUR                             0x01
#define AUDIO_REQ_SET_MIN                             0x02
#define AUDIO_REQ_SET_MAX                             0x03
#define AUDIO_REQ_SET_RES                             0x04
#define AUDIO_REQ_SET_MEM                             0x05

#define AUDIO_FORMAT_TYPE_I                           0x01
#define AUDIO_FORMAT_TYPE_III                         0x03

#define AUDIO_REQ_SET_VOLU                            0x11
#define AUDIO_REQ_SET_FREQ                            0x12

#define AUDIO_IN_INTERM_ID														0x01
#define AUDIO_IN_FU_ID                       					0x02
#define AUDIO_IN_OUTTERM_ID														0x03

#define AUDIO_OUT_INTERM_ID														0x04
#define AUDIO_OUT_FU_ID                      					0x05
#define AUDIO_OUT_OUTTERM_ID													0x06

#define AUDIO_CONTROL_MUTE														0x01
#define AUDIO_CONTROL_VOLUME													0x02

#define AUDIO_CS_UNDEF																0x00
#define AUDIO_CS_MUTE																	0x01
#define AUDIO_CS_VOLUME																0x02
#define AUDIO_CS_BASS																	0x03
#define AUDIO_CS_MID																	0x04
#define AUDIO_CS_TREBLE																0x05
#define AUDIO_CS_EQUA																	0x06
#define AUDIO_CS_AUTOGAIN															0x07
#define AUDIO_CS_DELAY																0x08
#define AUDIO_CS_BBOOST																0x09
#define AUDIO_CS_LOUDNESS															0x0A

#define OUT_SUBFRAMESIZE															0x02
#define OUT_BITRESOLUTION															0x10

#define OUT_CHANNELS_NUM															0x02
#define OUT_CHANNELS_CONFIG														0x03

#define IN_SUBFRAMESIZE																0x02
#define IN_BITRESOLUTION															0x10

#ifdef IN_STEREO
#define IN_CHANNELS_NUM																0x02
#define IN_CHANNELS_CONFIG														0x03
#else
#define IN_CHANNELS_NUM																0x01
#define IN_CHANNELS_CONFIG														0x00
#endif


#ifdef FEEDBACK_ENABLE
#define AUDIO_FB_PACKET 														  3
#define FEEDBACK_RATE 																4

#define FEED_AUDIO_MIN_FREQ 		(USBD_AUDIO_SPEAKER_FREQ - USBD_AUDIO_SPEAKER_FREQ / 8) //84k
#define FEED_AUDIO_MAX_FREQ 		(USBD_AUDIO_SPEAKER_FREQ + USBD_AUDIO_SPEAKER_FREQ / 4) //120k

#else

#define SPEAK_DN_COUNT 		((AUDIO_OUT_BUF_WORD / 2) - (AUDIO_OUT_BUF_WORD / 256))
#define SPEAK_UP_COUNT 		((AUDIO_OUT_BUF_WORD / 2) + (AUDIO_OUT_BUF_WORD / 256))

#define SPEAK_MIN_COUNT 	((AUDIO_OUT_BUF_WORD / 2) - (AUDIO_OUT_BUF_WORD / 64))
#define SPEAK_MAX_COUNT 	((AUDIO_OUT_BUF_WORD / 2) + (AUDIO_OUT_BUF_WORD / 64))

#endif

#define RECORD_DN_COUNT 	((AUDIO_IN_BUF_WORD / 2) - (AUDIO_IN_BUF_WORD / 64))  //3968
#define RECORD_UP_COUNT 	((AUDIO_IN_BUF_WORD / 2) + (AUDIO_IN_BUF_WORD / 64))  //4224

#define RECORD_MIN_COUNT 	((AUDIO_IN_BUF_WORD / 2) - (AUDIO_IN_BUF_WORD / 8))   //3072
#define RECORD_MAX_COUNT 	((AUDIO_IN_BUF_WORD / 2) + (AUDIO_IN_BUF_WORD / 8))   //5120


#ifdef FEEDBACK_ENABLE
#if (((FEED_AUDIO_MAX_FREQ * OUT_CHANNELS_NUM * OUT_SUBFRAMESIZE) % 1000) == 0)
#define AUDIO_OUT_PACKET                              (uint32_t)(((FEED_AUDIO_MAX_FREQ * OUT_CHANNELS_NUM * OUT_SUBFRAMESIZE) /1000))  //480
#else
#define AUDIO_OUT_PACKET                              (uint32_t)(((FEED_AUDIO_MAX_FREQ * OUT_CHANNELS_NUM * OUT_SUBFRAMESIZE) /1000) + 1)
#endif
#else
#if (((USBD_AUDIO_SPEAKER_FREQ * OUT_CHANNELS_NUM * OUT_SUBFRAMESIZE) % 1000) == 0)
#define AUDIO_OUT_PACKET                              (uint32_t)(((USBD_AUDIO_SPEAKER_FREQ * OUT_CHANNELS_NUM * OUT_SUBFRAMESIZE) /1000))
#else
#define AUDIO_OUT_PACKET                              (uint32_t)(((USBD_AUDIO_SPEAKER_FREQ * OUT_CHANNELS_NUM * OUT_SUBFRAMESIZE) /1000) + 1)
#endif
#endif

#if (((USBD_AUDIO_MICROPHONE_FREQ * IN_CHANNELS_NUM * IN_SUBFRAMESIZE) % 1000) == 0)
#define AUDIO_IN_PACKET                              (uint32_t)(((USBD_AUDIO_MICROPHONE_FREQ * IN_CHANNELS_NUM * IN_SUBFRAMESIZE) /1000)) //384
#else
#define AUDIO_IN_PACKET                              (uint32_t)(((USBD_AUDIO_MICROPHONE_FREQ * IN_CHANNELS_NUM * IN_SUBFRAMESIZE) /1000) + 1)
#endif

/* Audio Commands enumeration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
}AUDIO_CMD_TypeDef;

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct
{
   uint8_t data[USB_MAX_EP0_SIZE]; 
   uint8_t cmd;    
   uint8_t unit;  
   uint16_t type;  
   uint16_t len;    
}
USBD_AUDIO_ControlTypeDef; 

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  AUDIO_IN_IDLE = 0,
  AUDIO_IN_BUSY,
}
AUDIO_IN_StateTypeDef; 

typedef struct
{ 
  __IO uint8_t               out_alt_setting;
  __IO uint8_t               in_alt_setting;
	
  USBD_AUDIO_ControlTypeDef  control;    
}
USBD_AUDIO_HandleTypeDef; 


typedef struct
{
    int8_t  (*Init)         (uint32_t  AudioSPKFreq, uint32_t  AudioMICFreq, uint32_t Volume, uint32_t options);
    int8_t  (*DeInit)       (uint32_t options);
    int8_t  (*AudioCmd)     (uint8_t* pbuf, uint32_t size, uint8_t cmd);
    int8_t  (*VolumeCtl)    (uint8_t vol);
    int8_t  (*MuteCtl)      (uint8_t cmd);
    int8_t  (*PeriodicTC)   (uint8_t cmd);
    int8_t  (*GetState)     (void);
}USBD_AUDIO_ItfTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_AUDIO;
#define USBD_AUDIO_CLASS  &USBD_AUDIO
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, USBD_AUDIO_ItfTypeDef *fops);
void init_out_parameter(void);
void init_in_parameter(void);
#ifdef FEEDBACK_ENABLE	
void feedbakc_calc(uint32_t cur_freq);
#endif
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_AUDIO_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
