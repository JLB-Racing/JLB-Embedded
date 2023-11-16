/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

#include "stm32l5xx_ll_ucpd.h"
#include "stm32l5xx_ll_bus.h"
#include "stm32l5xx_ll_cortex.h"
#include "stm32l5xx_ll_rcc.h"
#include "stm32l5xx_ll_system.h"
#include "stm32l5xx_ll_utils.h"
#include "stm32l5xx_ll_pwr.h"
#include "stm32l5xx_ll_gpio.h"
#include "stm32l5xx_ll_dma.h"

#include "stm32l5xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Delay_Ticks(uint32_t delay);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_CH_A_Pin GPIO_PIN_3
#define ENC_CH_A_GPIO_Port GPIOE
#define ENC_CHB_Pin GPIO_PIN_4
#define ENC_CHB_GPIO_Port GPIOE
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define DRIVE_ENABLE_Pin GPIO_PIN_3
#define DRIVE_ENABLE_GPIO_Port GPIOF
#define MCU_FB_Pin GPIO_PIN_5
#define MCU_FB_GPIO_Port GPIOF
#define MOTOR_PWM1_Pin GPIO_PIN_6
#define MOTOR_PWM1_GPIO_Port GPIOF
#define REMOTE_GAS_Pin GPIO_PIN_7
#define REMOTE_GAS_GPIO_Port GPIOF
#define REMOTE_STEER_Pin GPIO_PIN_8
#define REMOTE_STEER_GPIO_Port GPIOF
#define MOTOR_PWM2_Pin GPIO_PIN_9
#define MOTOR_PWM2_GPIO_Port GPIOF
#define BATT_VOLTAGE_Pin GPIO_PIN_0
#define BATT_VOLTAGE_GPIO_Port GPIOC
#define MOT_CURR_Pin GPIO_PIN_1
#define MOT_CURR_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define ENC_CHA_Pin GPIO_PIN_3
#define ENC_CHA_GPIO_Port GPIOC
#define DISTANCE2_ADC_Pin GPIO_PIN_2
#define DISTANCE2_ADC_GPIO_Port GPIOA
#define DISTANCE1_ADC_Pin GPIO_PIN_3
#define DISTANCE1_ADC_GPIO_Port GPIOA
#define DISTANCE3_ADC_Pin GPIO_PIN_4
#define DISTANCE3_ADC_GPIO_Port GPIOA
#define SPI_ADC_CLK_Pin GPIO_PIN_5
#define SPI_ADC_CLK_GPIO_Port GPIOA
#define SPI_ADC_MISO_Pin GPIO_PIN_6
#define SPI_ADC_MISO_GPIO_Port GPIOA
#define LV_BATT_Pin GPIO_PIN_7
#define LV_BATT_GPIO_Port GPIOA
#define ENC_CHBB0_Pin GPIO_PIN_0
#define ENC_CHBB0_GPIO_Port GPIOB
#define DISTANCE4_ADC_Pin GPIO_PIN_1
#define DISTANCE4_ADC_GPIO_Port GPIOB
#define MAX_CS_Pin GPIO_PIN_2
#define MAX_CS_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_13
#define MCU_LED_GPIO_Port GPIOF
#define MOT_BATT_L_Pin GPIO_PIN_14
#define MOT_BATT_L_GPIO_Port GPIOF
#define LV_BATT_L_Pin GPIO_PIN_15
#define LV_BATT_L_GPIO_Port GPIOF
#define BUTTON2_Pin GPIO_PIN_9
#define BUTTON2_GPIO_Port GPIOE
#define ROTARY1_Pin GPIO_PIN_10
#define ROTARY1_GPIO_Port GPIOE
#define ROTARY2_Pin GPIO_PIN_11
#define ROTARY2_GPIO_Port GPIOE
#define ROTARY3_Pin GPIO_PIN_12
#define ROTARY3_GPIO_Port GPIOE
#define ROTARY4_Pin GPIO_PIN_13
#define ROTARY4_GPIO_Port GPIOE
#define RC_PWM_STEERING_IN_Pin GPIO_PIN_14
#define RC_PWM_STEERING_IN_GPIO_Port GPIOE
#define SPI_ADC_MOSI_Pin GPIO_PIN_15
#define SPI_ADC_MOSI_GPIO_Port GPIOE
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define ADCF1_CS_Pin GPIO_PIN_8
#define ADCF1_CS_GPIO_Port GPIOD
#define ADCF2_CS_Pin GPIO_PIN_9
#define ADCF2_CS_GPIO_Port GPIOD
#define ADCF3_CS_Pin GPIO_PIN_10
#define ADCF3_CS_GPIO_Port GPIOD
#define ADCF4_CS_Pin GPIO_PIN_11
#define ADCF4_CS_GPIO_Port GPIOD
#define ADCR1_CS_Pin GPIO_PIN_12
#define ADCR1_CS_GPIO_Port GPIOD
#define ADCR2_CS_Pin GPIO_PIN_13
#define ADCR2_CS_GPIO_Port GPIOD
#define ADCR3_CS_Pin GPIO_PIN_14
#define ADCR3_CS_GPIO_Port GPIOD
#define ADCR4_CS_Pin GPIO_PIN_15
#define ADCR4_CS_GPIO_Port GPIOD
#define RESET_BUTTON_Pin GPIO_PIN_4
#define RESET_BUTTON_GPIO_Port GPIOG
#define SET_BUTTON_Pin GPIO_PIN_5
#define SET_BUTTON_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOG
#define ST_LINK_VCP_TX_Pin GPIO_PIN_7
#define ST_LINK_VCP_TX_GPIO_Port GPIOG
#define ST_LINK_VCP_RX_Pin GPIO_PIN_8
#define ST_LINK_VCP_RX_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define RC_PWM_STEERING_Pin GPIO_PIN_8
#define RC_PWM_STEERING_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOA
#define RC_PWM_THROTTLE_IN_Pin GPIO_PIN_10
#define RC_PWM_THROTTLE_IN_GPIO_Port GPIOA
#define LED_LE_F_Pin GPIO_PIN_0
#define LED_LE_F_GPIO_Port GPIOD
#define INFRA_LE_F_Pin GPIO_PIN_1
#define INFRA_LE_F_GPIO_Port GPIOD
#define LED_OE_F_Pin GPIO_PIN_2
#define LED_OE_F_GPIO_Port GPIOD
#define XBEE_TX_Pin GPIO_PIN_5
#define XBEE_TX_GPIO_Port GPIOD
#define XBEE_RX_Pin GPIO_PIN_6
#define XBEE_RX_GPIO_Port GPIOD
#define INFRA_OE_F_Pin GPIO_PIN_7
#define INFRA_OE_F_GPIO_Port GPIOD
#define LED_LE_R_Pin GPIO_PIN_9
#define LED_LE_R_GPIO_Port GPIOG
#define INFRA_LE_R_Pin GPIO_PIN_10
#define INFRA_LE_R_GPIO_Port GPIOG
#define LED_OE_R_Pin GPIO_PIN_12
#define LED_OE_R_GPIO_Port GPIOG
#define INFRA_OE_R_Pin GPIO_PIN_13
#define INFRA_OE_R_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOG
#define UCPD_DBN_Pin GPIO_PIN_5
#define UCPD_DBN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_0
#define BUTTON1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
