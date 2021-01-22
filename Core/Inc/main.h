/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <arm_math.h>
#define FILTER_BLOCK_SIZE 205
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union{
	uint8_t ui8[4];
	uint16_t ui16[2];
	float_t f;
} Floating;

typedef struct
{
    volatile uint8_t flag;     /* Timeout event flag */
    uint16_t timer;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;

typedef struct{
    uint8_t pinDigital;
    bool status;
    uint16_t sensorData;
    Floating Power;
    bool aktif;
    uint8_t data_send[8];
    arm_biquad_casd_df1_inst_f32 filter_inst;
    float pState[4];
    Floating Amp;
    float totalAmp;
    uint32_t samples;
    Floating rms;
    GPIO_TypeDef *GPIOPort;
    uint16_t GPIOPin;
    bool midPointSet;
    float midPoint;
//#A112S
} module_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_2
#define A3_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_3
#define A4_GPIO_Port GPIOA
#define A5_Pin GPIO_PIN_4
#define A5_GPIO_Port GPIOA
#define A6_Pin GPIO_PIN_5
#define A6_GPIO_Port GPIOA
#define A7_Pin GPIO_PIN_6
#define A7_GPIO_Port GPIOA
#define A8_Pin GPIO_PIN_7
#define A8_GPIO_Port GPIOA
#define A9_Pin GPIO_PIN_0
#define A9_GPIO_Port GPIOB
#define A10_Pin GPIO_PIN_1
#define A10_GPIO_Port GPIOB
#define AnalogV_Pin GPIO_PIN_12
#define AnalogV_GPIO_Port GPIOB
#define D10_Pin GPIO_PIN_11
#define D10_GPIO_Port GPIOA
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOA
#define D8_Pin GPIO_PIN_15
#define D8_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_3
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_4
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_8
#define D2_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define DMA_BUF_SIZE 200
#define DMA_TIMEOUT_MS 10      /* DMA Timeout duration in msec */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
