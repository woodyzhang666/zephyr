/*
 * Copyright (c) 2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32F1_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32F1_CLOCK_H_

/** Peripheral clock sources */

/** Bus clocks */
#define CH32_CLOCK_BUS_AHB1    0x014
#define CH32_CLOCK_BUS_APB2    0x018
#define CH32_CLOCK_BUS_APB1    0x01c

#define CH32_PERIPH_BUS_MIN	CH32_CLOCK_BUS_AHB1
#define CH32_PERIPH_BUS_MAX	CH32_CLOCK_BUS_APB1

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32F1_CLOCK_H_ */
