/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "stm32f1xx.h"
#include "compiler.h"
#include "daplink.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32F103XB);

//Connected LED
#define CONNECTED_LED_PORT           GPIOA
#define CONNECTED_LED_PIN            GPIO_PIN_11
#define CONNECTED_LED_PIN_Bit        11

// nRESET OUT Pin
#define nRESET_PIN_PORT              GPIOA
#define nRESET_PIN                   GPIO_PIN_12
#define nRESET_PIN_Bit               12

//SWD
#define SWCLK_TCK_PIN_PORT           GPIOA
#define SWCLK_TCK_PIN                GPIO_PIN_14
#define SWCLK_TCK_PIN_Bit            14

#define SWDIO_OUT_PIN_PORT           GPIOA
#define SWDIO_OUT_PIN                GPIO_PIN_15
#define SWDIO_OUT_PIN_Bit            15

#define SWDIO_IN_PIN_PORT            GPIOA
#define SWDIO_IN_PIN                 GPIO_PIN_13
#define SWDIO_IN_PIN_Bit             13

//LEDs
//USB status LED
#define RUNNING_LED_PORT             GPIOA
#define RUNNING_LED_PIN              GPIO_PIN_10
#define RUNNING_LED_Bit              10

#define PIN_HID_LED_PORT             GPIOA
#define PIN_HID_LED                  GPIO_PIN_10
#define PIN_HID_LED_Bit              10

#define PIN_CDC_LED_PORT             GPIOA
#define PIN_CDC_LED                  GPIO_PIN_10
#define PIN_CDC_LED_Bit              10

#define PIN_MSC_LED_PORT             GPIOA
#define PIN_MSC_LED                  GPIO_PIN_10
#define PIN_MSC_LED_Bit              10


#endif
