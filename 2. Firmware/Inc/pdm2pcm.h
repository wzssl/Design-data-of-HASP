/**
  ******************************************************************************
  * @file    pdm2pcm.h
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    28-February-2017
  * @brief   Global header for PDM2PCM conversion code
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Image SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_image_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PDM2PCM_H
#define __PDM2PCM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "pdm2pcm_glo.h"
/* Exported constants --------------------------------------------------------*/
	 
/* Exported types ------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

#define PDM_IN_PACKET  																((uint16_t)((32 / 8) * 512 / 2))   //1024

#define CHANNEL_DEMUX_MASK                  					((uint8_t)0x55)
/* Exported functions ------------------------------------------------------- */


/* PDM2PCM init function */
void MX_PDM2PCM_Init(void);

void PDMDecoder_Init(uint32_t real_freq, uint8_t in_channels_num, uint8_t out_channels_num);
HAL_StatusTypeDef PDM_To_PCM(uint16_t* PDMBuf, uint16_t* PCMBuf);

#ifdef __cplusplus
}
#endif

#endif /* __PDM2PCM_FILTER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

