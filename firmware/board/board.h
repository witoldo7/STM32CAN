/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32H750XB_DISCOVERY board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32H750XB_DISCOVERY
#define BOARD_NAME                  "DevBox STM32H750XX_M"


/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                25000000U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32H750xx

#define BOARD_OTG_NOVBUSSENS

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#define PA00                           0U
#define PA01_USER_LED                  1U
#define PA02                           2U
#define PA03                           3U
#define PA04                           4U
#define PA05                           5U
#define PA6_SWM1                       6U
#define PA7_SWM0                       7U
#define PA08                           8U
#define PA09                           9U
#define PA10                           10U
#define PA11_USB_DM                    11U
#define PA12_USB_DP                    12U
#define PA13_SWDIO                     13U
#define PA14_SWCLK                     14U
#define PA15                           15U

#define PB00                           0U
#define PB01_VBAT                      1U
#define PB02_QSPI_CLK                  2U
#define PB03                           3U
#define PB04                           4U
#define PB05                           5U
#define PB06_QSPI_BK1_NNCS             6U
#define PB07                           7U
#define PB08_CAN1_RX                   8U
#define PB09_CAN1_TX                   9U
#define PB10                           10U
#define PB11                           11U
#define PB12_CAN2_RX                   12U
#define PB13_CAN2_TX                   13U
#define PB14_SD1_TX                    14U
#define PB15_SD1_RX                    15U

#define PC00                           0U
#define PC01                           1U
#define PC02                           2U
#define PC03                           3U
#define PC04                           4U
#define PC05_BTN_K2                    5U
#define PC06                           6U
#define PC07                           7U
#define PC08_SDIO_D0                   8U
#define PC09_SDIO_D1                   9U
#define PC10_SDIO_D2                   10U
#define PC11_SDIO_D3                   11U
#define PC12_SDIO_SCK                  12U
#define PC13                           13U
#define PC14                           14U
#define PC15                           15U

#define PD00                           0U
#define PD01                           1U
#define PD02_SDIO_CMD                  2U
#define PD03                           3U
#define PD04                           4U
#define PD05_KL_TX                     5U
#define PD06_KL_RX                     6U
#define PD07_SDIO_CD                   7U
#define PD08                           8U
#define PD09                           9U
#define PD10                           10U
#define PD11_QSPI_BK1_IO0              11U
#define PD12_QSPI_BK1_IO1              12U
#define PD13_QSPI_BK1_IO3              13U
#define PD14                           14U
#define PD15                           15U

#define PE00                           0U
#define PE01                           1U
#define PE02_QSPI_BK1_IO0              2U
#define PE3_BTN_K1                     3U
#define PE04                           4U
#define PE05                           5U
#define PE06                           6U
#define PE07                           7U
#define PE08                           8U
#define PE09                           9U
#define PE10                           10U
#define PE11                           11U
#define PE12                           12U
#define PE13                           13U
#define PE14                           14U
#define PE15                           15U

#define PF00                           0U
#define PF01                           1U
#define PF02                           2U
#define PF03                           3U
#define PF04                           4U
#define PF05                           5U
#define PF06                           6U
#define PF07                           7U
#define PF08                           8U
#define PF09                           9U
#define PF10                           10U
#define PF11                           11U
#define PF12                           12U
#define PF13                           13U
#define PF14                           14U
#define PF15                           15U

#define PG00                           0U
#define PG01                           1U
#define PG02                           2U
#define PG03                           3U
#define PG04                           4U
#define PG05                           5U
#define PG06                           6U
#define PG07                           7U
#define PG08                           8U
#define PG09                           9U
#define PG10                           10U
#define PG11                           11U
#define PG12                           12U
#define PG13                           13U
#define PG14                           14U
#define PG15                           15U

#define PH00_OSC_IN                    0U
#define PH01_OSC_OUT                   1U
#define PH02                           2U
#define PH03                           3U
#define PH04                           4U
#define PH05                           5U
#define PH06                           6U
#define PH07                           7U
#define PH08                           8U
#define PH09                           9U
#define PH10                           10U
#define PH11                           11U
#define PH12                           12U
#define PH13                           13U
#define PH14                           14U
#define PH15                           15U

#define PI00                           0U
#define PI01                           1U
#define PI02                           2U
#define PI03                           3U
#define PI04                           4U
#define PI05                           5U
#define PI06                           6U
#define PI07                           7U
#define PI08                           8U
#define PI09                           9U
#define PI10                           10U
#define PI11                           11U
#define PI12                           12U
#define PI13                           13U
#define PI14                           14U
#define PI15                           15U

#define PJ00                           0U
#define PJ01                           1U
#define PJ02                           2U
#define PJ03                           3U
#define PJ04                           4U
#define PJ05                           5U
#define PJ06                           6U
#define PJ07                           7U
#define PJ08                           8U
#define PJ09                           9U
#define PJ10                           10U
#define PJ11                           11U
#define PJ12                           12U
#define PJ13                           13U
#define PJ14                           14U
#define PJ15                           15U

#define PK00                           0U
#define PK01                           1U
#define PK02                           2U
#define PK03                           3U
#define PK04                           4U
#define PK05                           5U
#define PK06                           6U
#define PK07                           7U
#define PK08                           8U
#define PK09                           9U
#define PK10                           10U
#define PK11                           11U
#define PK12                           12U
#define PK13                           13U
#define PK14                           14U
#define PK15                           15U


/*
 * BDM lines assignment.
 */
#define LINE_CONNECTED  PAL_LINE(GPIOE, 7U)     // Vcc input
#define LINE_FREZE      PAL_LINE(GPIOE, 8U)     // input
#define LINE_BKPT       PAL_LINE(GPIOE, 9U)     // output
#define LINE_RESET      PAL_LINE(GPIOE, 10U)    // output
#define LINE_DSI        PAL_LINE(GPIOE, 11U)    // output
#define LINE_DSO        PAL_LINE(GPIOE, 12U)    // input

/*
 * CAN lines assignment.
 */
#define LINE_SWM0      PAL_LINE(GPIOA, 7U)
#define LINE_SWM1      PAL_LINE(GPIOA, 6U)

/*
 * KLine lines assignment.
 */
#define LINE_KL_RX      PAL_LINE(GPIOD, PD06_KL_RX)
#define LINE_KL_TX      PAL_LINE(GPIOD, PD05_KL_TX)

/*
 * Misd lines.
 */
#define LINE_SD_DETECT PAL_LINE(GPIOD, 7U)

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

#define VAL_GPIOA_MODER                 (PIN_MODE_INPUT(PA00) | \
                     PIN_MODE_OUTPUT(PA01_USER_LED) | \
                     PIN_MODE_INPUT(PA02) | \
                     PIN_MODE_INPUT(PA03) | \
                     PIN_MODE_INPUT(PA04) | \
                     PIN_MODE_INPUT(PA05) | \
                     PIN_MODE_OUTPUT(PA6_SWM1) | \
                     PIN_MODE_OUTPUT(PA7_SWM0) | \
                     PIN_MODE_INPUT(PA08) | \
                     PIN_MODE_INPUT(PA09) | \
                     PIN_MODE_INPUT(PA10) | \
                     PIN_MODE_ALTERNATE(PA11_USB_DM) | \
                     PIN_MODE_ALTERNATE(PA12_USB_DP) | \
                     PIN_MODE_ALTERNATE(PA13_SWDIO) | \
                     PIN_MODE_ALTERNATE(PA14_SWCLK) | \
                     PIN_MODE_INPUT(PA15))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00) | \
                     PIN_OTYPE_PUSHPULL(PA01_USER_LED) | \
                     PIN_OTYPE_PUSHPULL(PA02) | \
                     PIN_OTYPE_PUSHPULL(PA03) | \
                     PIN_OTYPE_PUSHPULL(PA04) | \
                     PIN_OTYPE_PUSHPULL(PA05) | \
                     PIN_OTYPE_PUSHPULL(PA6_SWM1) | \
                     PIN_OTYPE_PUSHPULL(PA7_SWM0) | \
                     PIN_OTYPE_PUSHPULL(PA08) | \
                     PIN_OTYPE_PUSHPULL(PA09) | \
                     PIN_OTYPE_PUSHPULL(PA10) | \
                     PIN_OTYPE_PUSHPULL(PA11_USB_DM) | \
                     PIN_OTYPE_PUSHPULL(PA12_USB_DP) | \
                     PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
                     PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
                     PIN_OTYPE_PUSHPULL(PA15))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PA00) | \
                     PIN_OSPEED_SPEED_HIGH(PA01_USER_LED) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA6_SWM1) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA7_SWM0) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA10) | \
                     PIN_OSPEED_SPEED_HIGH(PA11_USB_DM) | \
                     PIN_OSPEED_SPEED_HIGH(PA12_USB_DP) | \
                     PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
                     PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
                     PIN_OSPEED_SPEED_VERYLOW(PA15))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_PULLDOWN(PA00) | \
                     PIN_PUPDR_PULLDOWN(PA01_USER_LED) | \
                     PIN_PUPDR_PULLDOWN(PA02) | \
                     PIN_PUPDR_PULLDOWN(PA03) | \
                     PIN_PUPDR_PULLDOWN(PA04) | \
                     PIN_PUPDR_PULLDOWN(PA05) | \
                     PIN_PUPDR_PULLUP(PA6_SWM1) | \
                     PIN_PUPDR_PULLUP(PA7_SWM0) | \
                     PIN_PUPDR_PULLDOWN(PA08) | \
                     PIN_PUPDR_PULLDOWN(PA09) | \
                     PIN_PUPDR_PULLDOWN(PA10) | \
                     PIN_PUPDR_FLOATING(PA11_USB_DM) | \
                     PIN_PUPDR_FLOATING(PA12_USB_DP) | \
                     PIN_PUPDR_FLOATING(PA13_SWDIO) | \
                     PIN_PUPDR_FLOATING(PA14_SWCLK) | \
                     PIN_PUPDR_PULLDOWN(PA15))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(PA00) | \
                     PIN_ODR_LEVEL_HIGH(PA01_USER_LED) | \
                     PIN_ODR_LEVEL_LOW(PA02) | \
                     PIN_ODR_LEVEL_LOW(PA03) | \
                     PIN_ODR_LEVEL_LOW(PA04) | \
                     PIN_ODR_LEVEL_LOW(PA05) | \
                     PIN_ODR_LEVEL_HIGH(PA6_SWM1) | \
                     PIN_ODR_LEVEL_HIGH(PA7_SWM0) | \
                     PIN_ODR_LEVEL_LOW(PA08) | \
                     PIN_ODR_LEVEL_LOW(PA09) | \
                     PIN_ODR_LEVEL_LOW(PA10) | \
                     PIN_ODR_LEVEL_HIGH(PA11_USB_DM) | \
                     PIN_ODR_LEVEL_HIGH(PA12_USB_DP) | \
                     PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
                     PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
                     PIN_ODR_LEVEL_LOW(PA15))

#define VAL_GPIOA_AFRL          (PIN_AFIO_AF(PA00, 0) | \
                     PIN_AFIO_AF(PA01_USER_LED, 0) | \
                     PIN_AFIO_AF(PA02, 0) | \
                     PIN_AFIO_AF(PA03, 0) | \
                     PIN_AFIO_AF(PA04, 0) | \
                     PIN_AFIO_AF(PA05, 0) | \
                     PIN_AFIO_AF(PA6_SWM1, 0) | \
                     PIN_AFIO_AF(PA7_SWM0, 0))

#define VAL_GPIOA_AFRH          (PIN_AFIO_AF(PA08, 0) | \
                     PIN_AFIO_AF(PA09, 0) | \
                     PIN_AFIO_AF(PA10, 0) | \
                     PIN_AFIO_AF(PA11_USB_DM, 10) | \
                     PIN_AFIO_AF(PA12_USB_DP, 10) | \
                     PIN_AFIO_AF(PA13_SWDIO, 0) | \
                     PIN_AFIO_AF(PA14_SWCLK, 0) | \
                     PIN_AFIO_AF(PA15, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(PB00) | \
                     PIN_MODE_ANALOG(PB01_VBAT) | \
                     PIN_MODE_ALTERNATE(PB02_QSPI_CLK) | \
                     PIN_MODE_INPUT(PB03) | \
                     PIN_MODE_INPUT(PB04) | \
                     PIN_MODE_INPUT(PB05) | \
                     PIN_MODE_ALTERNATE(PB06_QSPI_BK1_NNCS) | \
                     PIN_MODE_INPUT(PB07) | \
                     PIN_MODE_ALTERNATE(PB08_CAN1_RX) | \
                     PIN_MODE_ALTERNATE(PB09_CAN1_TX) | \
                     PIN_MODE_INPUT(PB10) | \
                     PIN_MODE_INPUT(PB11) | \
                     PIN_MODE_ALTERNATE(PB12_CAN2_RX) | \
                     PIN_MODE_ALTERNATE(PB13_CAN2_TX) | \
                     PIN_MODE_ALTERNATE(PB14_SD1_TX) | \
                     PIN_MODE_ALTERNATE(PB15_SD1_RX))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00) | \
                     PIN_OTYPE_PUSHPULL(PB01_VBAT) | \
                     PIN_OTYPE_PUSHPULL(PB02_QSPI_CLK) | \
                     PIN_OTYPE_PUSHPULL(PB03) | \
                     PIN_OTYPE_PUSHPULL(PB04) | \
                     PIN_OTYPE_PUSHPULL(PB05) | \
                     PIN_OTYPE_PUSHPULL(PB06_QSPI_BK1_NNCS) | \
                     PIN_OTYPE_PUSHPULL(PB07) | \
                     PIN_OTYPE_OPENDRAIN(PB08_CAN1_RX) | \
                     PIN_OTYPE_PUSHPULL(PB09_CAN1_TX) | \
                     PIN_OTYPE_PUSHPULL(PB10) | \
                     PIN_OTYPE_PUSHPULL(PB11) | \
                     PIN_OTYPE_OPENDRAIN(PB12_CAN2_RX) | \
                     PIN_OTYPE_PUSHPULL(PB13_CAN2_TX) | \
                     PIN_OTYPE_PUSHPULL(PB14_SD1_TX) | \
                     PIN_OTYPE_OPENDRAIN(PB15_SD1_RX))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PB00) | \
                     PIN_OSPEED_SPEED_HIGH(PB01_VBAT) | \
                     PIN_OSPEED_SPEED_HIGH(PB02_QSPI_CLK) | \
                     PIN_OSPEED_SPEED_VERYLOW(PB03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PB04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PB05) | \
                     PIN_OSPEED_SPEED_HIGH(PB06_QSPI_BK1_NNCS) | \
                     PIN_OSPEED_SPEED_VERYLOW(PB07) | \
                     PIN_OSPEED_SPEED_HIGH(PB08_CAN1_RX) | \
                     PIN_OSPEED_SPEED_HIGH(PB09_CAN1_TX) | \
                     PIN_OSPEED_SPEED_VERYLOW(PB10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PB11) | \
                     PIN_OSPEED_SPEED_HIGH(PB12_CAN2_RX) | \
                     PIN_OSPEED_SPEED_HIGH(PB13_CAN2_TX) | \
                     PIN_OSPEED_SPEED_HIGH(PB14_SD1_TX) | \
                     PIN_OSPEED_SPEED_HIGH(PB15_SD1_RX))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_PULLDOWN(PB00) | \
                     PIN_PUPDR_FLOATING(PB01_VBAT) | \
                     PIN_PUPDR_FLOATING(PB02_QSPI_CLK) | \
                     PIN_PUPDR_PULLDOWN(PB03) | \
                     PIN_PUPDR_PULLDOWN(PB04) | \
                     PIN_PUPDR_PULLDOWN(PB05) | \
                     PIN_PUPDR_FLOATING(PB06_QSPI_BK1_NNCS) | \
                     PIN_PUPDR_PULLDOWN(PB07) | \
                     PIN_PUPDR_FLOATING(PB08_CAN1_RX) | \
                     PIN_PUPDR_FLOATING(PB09_CAN1_TX) | \
                     PIN_PUPDR_PULLDOWN(PB10) | \
                     PIN_PUPDR_PULLDOWN(PB11) | \
                     PIN_PUPDR_FLOATING(PB12_CAN2_RX) | \
                     PIN_PUPDR_FLOATING(PB13_CAN2_TX) | \
                     PIN_PUPDR_FLOATING(PB14_SD1_TX) | \
                     PIN_PUPDR_FLOATING(PB15_SD1_RX))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00) | \
                     PIN_ODR_LEVEL_HIGH(PB01_VBAT) | \
                     PIN_ODR_LEVEL_HIGH(PB02_QSPI_CLK) | \
                     PIN_ODR_LEVEL_LOW(PB03) | \
                     PIN_ODR_LEVEL_LOW(PB04) | \
                     PIN_ODR_LEVEL_LOW(PB05) | \
                     PIN_ODR_LEVEL_HIGH(PB06_QSPI_BK1_NNCS) | \
                     PIN_ODR_LEVEL_LOW(PB07) | \
                     PIN_ODR_LEVEL_HIGH(PB08_CAN1_RX) | \
                     PIN_ODR_LEVEL_HIGH(PB09_CAN1_TX) | \
                     PIN_ODR_LEVEL_LOW(PB10) | \
                     PIN_ODR_LEVEL_LOW(PB11) | \
                     PIN_ODR_LEVEL_HIGH(PB12_CAN2_RX) | \
                     PIN_ODR_LEVEL_HIGH(PB13_CAN2_TX) | \
                     PIN_ODR_LEVEL_HIGH(PB14_SD1_TX) | \
                     PIN_ODR_LEVEL_HIGH(PB15_SD1_RX))

#define VAL_GPIOB_AFRL          (PIN_AFIO_AF(PB00, 0) | \
                     PIN_AFIO_AF(PB01_VBAT, 0) | \
                     PIN_AFIO_AF(PB02_QSPI_CLK, 9) | \
                     PIN_AFIO_AF(PB03, 0) | \
                     PIN_AFIO_AF(PB04, 0) | \
                     PIN_AFIO_AF(PB05, 0) | \
                     PIN_AFIO_AF(PB06_QSPI_BK1_NNCS, 10) | \
                     PIN_AFIO_AF(PB07, 0))

#define VAL_GPIOB_AFRH          (PIN_AFIO_AF(PB08_CAN1_RX, 9) | \
                     PIN_AFIO_AF(PB09_CAN1_TX, 9) | \
                     PIN_AFIO_AF(PB10, 0) | \
                     PIN_AFIO_AF(PB11, 0) | \
                     PIN_AFIO_AF(PB12_CAN2_RX, 9) | \
                     PIN_AFIO_AF(PB13_CAN2_TX, 9) | \
                     PIN_AFIO_AF(PB14_SD1_TX, 4) | \
                     PIN_AFIO_AF(PB15_SD1_RX, 4))

#define VAL_GPIOC_MODER                 (PIN_MODE_INPUT(PC00) | \
                     PIN_MODE_INPUT(PC01) | \
                     PIN_MODE_INPUT(PC02) | \
                     PIN_MODE_INPUT(PC03) | \
                     PIN_MODE_INPUT(PC04) | \
                     PIN_MODE_INPUT(PC05_BTN_K2) | \
                     PIN_MODE_INPUT(PC06) | \
                     PIN_MODE_INPUT(PC07) | \
                     PIN_MODE_ALTERNATE(PC08_SDIO_D0) | \
                     PIN_MODE_ALTERNATE(PC09_SDIO_D1) | \
                     PIN_MODE_ALTERNATE(PC10_SDIO_D2) | \
                     PIN_MODE_ALTERNATE(PC11_SDIO_D3) | \
                     PIN_MODE_ALTERNATE(PC12_SDIO_SCK) | \
                     PIN_MODE_INPUT(PC13) | \
                     PIN_MODE_INPUT(PC14) | \
                     PIN_MODE_INPUT(PC15))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_OPENDRAIN(PC00) | \
                     PIN_OTYPE_PUSHPULL(PC01) | \
                     PIN_OTYPE_PUSHPULL(PC02) | \
                     PIN_OTYPE_PUSHPULL(PC03) | \
                     PIN_OTYPE_PUSHPULL(PC04) | \
                     PIN_OTYPE_PUSHPULL(PC05_BTN_K2) | \
                     PIN_OTYPE_PUSHPULL(PC06) | \
                     PIN_OTYPE_PUSHPULL(PC07) | \
                     PIN_OTYPE_PUSHPULL(PC08_SDIO_D0) | \
                     PIN_OTYPE_PUSHPULL(PC09_SDIO_D1) | \
                     PIN_OTYPE_PUSHPULL(PC10_SDIO_D2) | \
                     PIN_OTYPE_PUSHPULL(PC11_SDIO_D3) | \
                     PIN_OTYPE_PUSHPULL(PC12_SDIO_SCK) | \
                     PIN_OTYPE_PUSHPULL(PC13) | \
                     PIN_OTYPE_PUSHPULL(PC14) | \
                     PIN_OTYPE_PUSHPULL(PC15))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC01) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC04) | \
                     PIN_OSPEED_SPEED_HIGH(PC05_BTN_K2) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC07) | \
                     PIN_OSPEED_SPEED_HIGH(PC08_SDIO_D0) | \
                     PIN_OSPEED_SPEED_HIGH(PC09_SDIO_D1) | \
                     PIN_OSPEED_SPEED_HIGH(PC10_SDIO_D2) | \
                     PIN_OSPEED_SPEED_HIGH(PC11_SDIO_D3) | \
                     PIN_OSPEED_SPEED_HIGH(PC12_SDIO_SCK) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PC15))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_PULLDOWN(PC00) | \
                     PIN_PUPDR_PULLDOWN(PC01) | \
                     PIN_PUPDR_PULLDOWN(PC02) | \
                     PIN_PUPDR_PULLDOWN(PC03) | \
                     PIN_PUPDR_PULLDOWN(PC04) | \
                     PIN_PUPDR_FLOATING(PC05_BTN_K2) | \
                     PIN_PUPDR_PULLDOWN(PC06) | \
                     PIN_PUPDR_PULLDOWN(PC07) | \
                     PIN_PUPDR_PULLUP(PC08_SDIO_D0) | \
                     PIN_PUPDR_PULLUP(PC09_SDIO_D1) | \
                     PIN_PUPDR_PULLUP(PC10_SDIO_D2) | \
                     PIN_PUPDR_PULLUP(PC11_SDIO_D3) | \
                     PIN_PUPDR_FLOATING(PC12_SDIO_SCK) | \
                     PIN_PUPDR_PULLDOWN(PC13) | \
                     PIN_PUPDR_PULLDOWN(PC14) | \
                     PIN_PUPDR_PULLDOWN(PC15))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_LOW(PC00) | \
                     PIN_ODR_LEVEL_LOW(PC01) | \
                     PIN_ODR_LEVEL_LOW(PC02) | \
                     PIN_ODR_LEVEL_LOW(PC03) | \
                     PIN_ODR_LEVEL_LOW(PC04) | \
                     PIN_ODR_LEVEL_HIGH(PC05_BTN_K2) | \
                     PIN_ODR_LEVEL_LOW(PC06) | \
                     PIN_ODR_LEVEL_LOW(PC07) | \
                     PIN_ODR_LEVEL_LOW(PC08_SDIO_D0) | \
                     PIN_ODR_LEVEL_LOW(PC09_SDIO_D1) | \
                     PIN_ODR_LEVEL_LOW(PC10_SDIO_D2) | \
                     PIN_ODR_LEVEL_LOW(PC11_SDIO_D3) | \
                     PIN_ODR_LEVEL_LOW(PC12_SDIO_SCK) | \
                     PIN_ODR_LEVEL_LOW(PC13) | \
                     PIN_ODR_LEVEL_LOW(PC14) | \
                     PIN_ODR_LEVEL_LOW(PC15))

#define VAL_GPIOC_AFRL          (PIN_AFIO_AF(PC00, 0) | \
                     PIN_AFIO_AF(PC01, 0) | \
                     PIN_AFIO_AF(PC02, 0) | \
                     PIN_AFIO_AF(PC03, 0) | \
                     PIN_AFIO_AF(PC04, 0) | \
                     PIN_AFIO_AF(PC05_BTN_K2, 0) | \
                     PIN_AFIO_AF(PC06, 0) | \
                     PIN_AFIO_AF(PC07, 0))

#define VAL_GPIOC_AFRH          (PIN_AFIO_AF(PC08_SDIO_D0, 12) | \
                     PIN_AFIO_AF(PC09_SDIO_D1, 12) | \
                     PIN_AFIO_AF(PC10_SDIO_D2, 12) | \
                     PIN_AFIO_AF(PC11_SDIO_D3, 12) | \
                     PIN_AFIO_AF(PC12_SDIO_SCK, 12) | \
                     PIN_AFIO_AF(PC13, 0) | \
                     PIN_AFIO_AF(PC14, 0) | \
                     PIN_AFIO_AF(PC15, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_INPUT(PD00) | \
                     PIN_MODE_INPUT(PD01) | \
                     PIN_MODE_ALTERNATE(PD02_SDIO_CMD) | \
                     PIN_MODE_INPUT(PD03) | \
                     PIN_MODE_INPUT(PD04) | \
                     PIN_MODE_ALTERNATE(PD05_KL_TX) | \
                     PIN_MODE_ALTERNATE(PD06_KL_RX) | \
                     PIN_MODE_INPUT(PD07_SDIO_CD) | \
                     PIN_MODE_INPUT(PD08) | \
                     PIN_MODE_INPUT(PD09) | \
                     PIN_MODE_INPUT(PD10) | \
                     PIN_MODE_ALTERNATE(PD11_QSPI_BK1_IO0) | \
                     PIN_MODE_ALTERNATE(PD12_QSPI_BK1_IO1) | \
                     PIN_MODE_ALTERNATE(PD13_QSPI_BK1_IO3) | \
                     PIN_MODE_INPUT(PD14) | \
                     PIN_MODE_INPUT(PD15))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00) | \
                     PIN_OTYPE_PUSHPULL(PD01) | \
                     PIN_OTYPE_PUSHPULL(PD02_SDIO_CMD) | \
                     PIN_OTYPE_PUSHPULL(PD03) | \
                     PIN_OTYPE_PUSHPULL(PD04) | \
                     PIN_OTYPE_PUSHPULL(PD05_KL_TX) | \
                     PIN_OTYPE_OPENDRAIN(PD06_KL_RX) | \
                     PIN_OTYPE_PUSHPULL(PD07_SDIO_CD) | \
                     PIN_OTYPE_PUSHPULL(PD08) | \
                     PIN_OTYPE_PUSHPULL(PD09) | \
                     PIN_OTYPE_PUSHPULL(PD10) | \
                     PIN_OTYPE_PUSHPULL(PD11_QSPI_BK1_IO0) | \
                     PIN_OTYPE_PUSHPULL(PD12_QSPI_BK1_IO1) | \
                     PIN_OTYPE_PUSHPULL(PD13_QSPI_BK1_IO3) | \
                     PIN_OTYPE_PUSHPULL(PD14) | \
                     PIN_OTYPE_PUSHPULL(PD15))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PD00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD01) | \
                     PIN_OSPEED_SPEED_HIGH(PD02_SDIO_CMD) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD04) | \
                     PIN_OSPEED_SPEED_HIGH(PD05_KL_TX) | \
                     PIN_OSPEED_SPEED_HIGH(PD06_KL_RX) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD07_SDIO_CD) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD10) | \
                     PIN_OSPEED_SPEED_HIGH(PD11_QSPI_BK1_IO0) | \
                     PIN_OSPEED_SPEED_HIGH(PD12_QSPI_BK1_IO1) | \
                     PIN_OSPEED_SPEED_HIGH(PD13_QSPI_BK1_IO3) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PD15))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_PULLDOWN(PD00) | \
                     PIN_PUPDR_PULLDOWN(PD01) | \
                     PIN_PUPDR_PULLUP(PD02_SDIO_CMD) | \
                     PIN_PUPDR_PULLDOWN(PD03) | \
                     PIN_PUPDR_PULLDOWN(PD04) | \
                     PIN_PUPDR_FLOATING(PD05_KL_TX) | \
                     PIN_PUPDR_FLOATING(PD06_KL_RX) | \
                     PIN_PUPDR_PULLUP(PD07_SDIO_CD) | \
                     PIN_PUPDR_PULLDOWN(PD08) | \
                     PIN_PUPDR_PULLDOWN(PD09) | \
                     PIN_PUPDR_PULLDOWN(PD10) | \
                     PIN_PUPDR_FLOATING(PD11_QSPI_BK1_IO0) | \
                     PIN_PUPDR_FLOATING(PD12_QSPI_BK1_IO1) | \
                     PIN_PUPDR_FLOATING(PD13_QSPI_BK1_IO3) | \
                     PIN_PUPDR_PULLDOWN(PD14) | \
                     PIN_PUPDR_PULLDOWN(PD15))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_LOW(PD00) | \
                     PIN_ODR_LEVEL_LOW(PD01) | \
                     PIN_ODR_LEVEL_LOW(PD02_SDIO_CMD) | \
                     PIN_ODR_LEVEL_LOW(PD03) | \
                     PIN_ODR_LEVEL_LOW(PD04) | \
                     PIN_ODR_LEVEL_HIGH(PD05_KL_TX) | \
                     PIN_ODR_LEVEL_HIGH(PD06_KL_RX) | \
                     PIN_ODR_LEVEL_HIGH(PD07_SDIO_CD) | \
                     PIN_ODR_LEVEL_LOW(PD08) | \
                     PIN_ODR_LEVEL_LOW(PD09) | \
                     PIN_ODR_LEVEL_LOW(PD10) | \
                     PIN_ODR_LEVEL_HIGH(PD11_QSPI_BK1_IO0) | \
                     PIN_ODR_LEVEL_HIGH(PD12_QSPI_BK1_IO1) | \
                     PIN_ODR_LEVEL_HIGH(PD13_QSPI_BK1_IO3) | \
                     PIN_ODR_LEVEL_LOW(PD14) | \
                     PIN_ODR_LEVEL_LOW(PD15))

#define VAL_GPIOD_AFRL          (PIN_AFIO_AF(PD00, 0) | \
                     PIN_AFIO_AF(PD01, 0) | \
                     PIN_AFIO_AF(PD02_SDIO_CMD, 12) | \
                     PIN_AFIO_AF(PD03, 0) | \
                     PIN_AFIO_AF(PD04, 0) | \
                     PIN_AFIO_AF(PD05_KL_TX, 7) | \
                     PIN_AFIO_AF(PD06_KL_RX, 7) | \
                     PIN_AFIO_AF(PD07_SDIO_CD, 0))

#define VAL_GPIOD_AFRH          (PIN_AFIO_AF(PD08, 0) | \
                     PIN_AFIO_AF(PD09, 0) | \
                     PIN_AFIO_AF(PD10, 0) | \
                     PIN_AFIO_AF(PD11_QSPI_BK1_IO0, 9) | \
                     PIN_AFIO_AF(PD12_QSPI_BK1_IO1, 9) | \
                     PIN_AFIO_AF(PD13_QSPI_BK1_IO3, 9) | \
                     PIN_AFIO_AF(PD14, 0) | \
                     PIN_AFIO_AF(PD15, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_INPUT(PE00) | \
                     PIN_MODE_INPUT(PE01) | \
                     PIN_MODE_ALTERNATE(PE02_QSPI_BK1_IO0) | \
                     PIN_MODE_INPUT(PE3_BTN_K1) | \
                     PIN_MODE_INPUT(PE04) | \
                     PIN_MODE_INPUT(PE05) | \
                     PIN_MODE_INPUT(PE06) | \
                     PIN_MODE_INPUT(PE07) | \
                     PIN_MODE_INPUT(PE08) | \
                     PIN_MODE_INPUT(PE09) | \
                     PIN_MODE_INPUT(PE10) | \
                     PIN_MODE_INPUT(PE11) | \
                     PIN_MODE_INPUT(PE12) | \
                     PIN_MODE_INPUT(PE13) | \
                     PIN_MODE_INPUT(PE14) | \
                     PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00) | \
                     PIN_OTYPE_PUSHPULL(PE01) | \
                     PIN_OTYPE_PUSHPULL(PE02_QSPI_BK1_IO0) | \
                     PIN_OTYPE_PUSHPULL(PE3_BTN_K1) | \
                     PIN_OTYPE_PUSHPULL(PE04) | \
                     PIN_OTYPE_PUSHPULL(PE05) | \
                     PIN_OTYPE_PUSHPULL(PE06) | \
                     PIN_OTYPE_PUSHPULL(PE07) | \
                     PIN_OTYPE_PUSHPULL(PE08) | \
                     PIN_OTYPE_PUSHPULL(PE09) | \
                     PIN_OTYPE_PUSHPULL(PE10) | \
                     PIN_OTYPE_PUSHPULL(PE11) | \
                     PIN_OTYPE_PUSHPULL(PE12) | \
                     PIN_OTYPE_PUSHPULL(PE13) | \
                     PIN_OTYPE_PUSHPULL(PE14) | \
                     PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PE00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE01) | \
                     PIN_OSPEED_SPEED_HIGH(PE02_QSPI_BK1_IO0) | \
                     PIN_OSPEED_SPEED_HIGH(PE3_BTN_K1) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_PULLDOWN(PE00) | \
                     PIN_PUPDR_PULLDOWN(PE01) | \
                     PIN_PUPDR_FLOATING(PE02_QSPI_BK1_IO0) | \
                     PIN_PUPDR_FLOATING(PE3_BTN_K1) | \
                     PIN_PUPDR_PULLDOWN(PE04) | \
                     PIN_PUPDR_PULLDOWN(PE05) | \
                     PIN_PUPDR_PULLDOWN(PE06) | \
                     PIN_PUPDR_PULLDOWN(PE07) | \
                     PIN_PUPDR_PULLDOWN(PE08) | \
                     PIN_PUPDR_PULLDOWN(PE09) | \
                     PIN_PUPDR_PULLDOWN(PE10) | \
                     PIN_PUPDR_PULLDOWN(PE11) | \
                     PIN_PUPDR_PULLDOWN(PE12) | \
                     PIN_PUPDR_PULLDOWN(PE13) | \
                     PIN_PUPDR_PULLDOWN(PE14) | \
                     PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_LOW(PE00) | \
                     PIN_ODR_LEVEL_LOW(PE01) | \
                     PIN_ODR_LEVEL_HIGH(PE02_QSPI_BK1_IO0) | \
                     PIN_ODR_LEVEL_HIGH(PE3_BTN_K1) | \
                     PIN_ODR_LEVEL_LOW(PE04) | \
                     PIN_ODR_LEVEL_LOW(PE05) | \
                     PIN_ODR_LEVEL_LOW(PE06) | \
                     PIN_ODR_LEVEL_LOW(PE07) | \
                     PIN_ODR_LEVEL_LOW(PE08) | \
                     PIN_ODR_LEVEL_LOW(PE09) | \
                     PIN_ODR_LEVEL_LOW(PE10) | \
                     PIN_ODR_LEVEL_LOW(PE11) | \
                     PIN_ODR_LEVEL_LOW(PE12) | \
                     PIN_ODR_LEVEL_LOW(PE13) | \
                     PIN_ODR_LEVEL_LOW(PE14) | \
                     PIN_ODR_LEVEL_LOW(PE15))

#define VAL_GPIOE_AFRL          (PIN_AFIO_AF(PE00, 0) | \
                     PIN_AFIO_AF(PE01, 8) | \
                     PIN_AFIO_AF(PE02_QSPI_BK1_IO0, 9) | \
                     PIN_AFIO_AF(PE3_BTN_K1, 0) | \
                     PIN_AFIO_AF(PE04, 0) | \
                     PIN_AFIO_AF(PE05, 0) | \
                     PIN_AFIO_AF(PE06, 0) | \
                     PIN_AFIO_AF(PE07, 0))

#define VAL_GPIOE_AFRH          (PIN_AFIO_AF(PE08, 0) | \
                     PIN_AFIO_AF(PE09, 0) | \
                     PIN_AFIO_AF(PE10, 0) | \
                     PIN_AFIO_AF(PE11, 0) | \
                     PIN_AFIO_AF(PE12, 0) | \
                     PIN_AFIO_AF(PE13, 0) | \
                     PIN_AFIO_AF(PE14, 0) | \
                     PIN_AFIO_AF(PE15, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_INPUT(PF00) | \
                     PIN_MODE_INPUT(PF01) | \
                     PIN_MODE_INPUT(PF02) | \
                     PIN_MODE_INPUT(PF03) | \
                     PIN_MODE_INPUT(PF04) | \
                     PIN_MODE_INPUT(PF05) | \
                     PIN_MODE_INPUT(PF06) | \
                     PIN_MODE_INPUT(PF07) | \
                     PIN_MODE_INPUT(PF08) | \
                     PIN_MODE_INPUT(PF09) | \
                     PIN_MODE_INPUT(PF10) | \
                     PIN_MODE_INPUT(PF11) | \
                     PIN_MODE_INPUT(PF12) | \
                     PIN_MODE_INPUT(PF13) | \
                     PIN_MODE_INPUT(PF14) | \
                     PIN_MODE_INPUT(PF15))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_PUSHPULL(PF00) | \
                     PIN_OTYPE_PUSHPULL(PF01) | \
                     PIN_OTYPE_PUSHPULL(PF02) | \
                     PIN_OTYPE_PUSHPULL(PF03) | \
                     PIN_OTYPE_PUSHPULL(PF04) | \
                     PIN_OTYPE_PUSHPULL(PF05) | \
                     PIN_OTYPE_PUSHPULL(PF06) | \
                     PIN_OTYPE_PUSHPULL(PF07) | \
                     PIN_OTYPE_PUSHPULL(PF08) | \
                     PIN_OTYPE_PUSHPULL(PF09) | \
                     PIN_OTYPE_PUSHPULL(PF10) | \
                     PIN_OTYPE_PUSHPULL(PF11) | \
                     PIN_OTYPE_PUSHPULL(PF12) | \
                     PIN_OTYPE_PUSHPULL(PF13) | \
                     PIN_OTYPE_PUSHPULL(PF14) | \
                     PIN_OTYPE_PUSHPULL(PF15))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PF00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF01) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PF15))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_PULLDOWN(PF00) | \
                     PIN_PUPDR_PULLDOWN(PF01) | \
                     PIN_PUPDR_PULLDOWN(PF02) | \
                     PIN_PUPDR_PULLDOWN(PF03) | \
                     PIN_PUPDR_PULLDOWN(PF04) | \
                     PIN_PUPDR_PULLDOWN(PF05) | \
                     PIN_PUPDR_PULLDOWN(PF06) | \
                     PIN_PUPDR_PULLDOWN(PF07) | \
                     PIN_PUPDR_PULLDOWN(PF08) | \
                     PIN_PUPDR_PULLDOWN(PF09) | \
                     PIN_PUPDR_PULLDOWN(PF10) | \
                     PIN_PUPDR_PULLDOWN(PF11) | \
                     PIN_PUPDR_PULLDOWN(PF12) | \
                     PIN_PUPDR_PULLDOWN(PF13) | \
                     PIN_PUPDR_PULLDOWN(PF14) | \
                     PIN_PUPDR_PULLDOWN(PF15))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_LOW(PF00) | \
                     PIN_ODR_LEVEL_LOW(PF01) | \
                     PIN_ODR_LEVEL_LOW(PF02) | \
                     PIN_ODR_LEVEL_LOW(PF03) | \
                     PIN_ODR_LEVEL_LOW(PF04) | \
                     PIN_ODR_LEVEL_LOW(PF05) | \
                     PIN_ODR_LEVEL_LOW(PF06) | \
                     PIN_ODR_LEVEL_LOW(PF07) | \
                     PIN_ODR_LEVEL_LOW(PF08) | \
                     PIN_ODR_LEVEL_LOW(PF09) | \
                     PIN_ODR_LEVEL_LOW(PF10) | \
                     PIN_ODR_LEVEL_LOW(PF11) | \
                     PIN_ODR_LEVEL_LOW(PF12) | \
                     PIN_ODR_LEVEL_LOW(PF13) | \
                     PIN_ODR_LEVEL_LOW(PF14) | \
                     PIN_ODR_LEVEL_LOW(PF15))

#define VAL_GPIOF_AFRL          (PIN_AFIO_AF(PF00, 0) | \
                     PIN_AFIO_AF(PF01, 0) | \
                     PIN_AFIO_AF(PF02, 0) | \
                     PIN_AFIO_AF(PF03, 0) | \
                     PIN_AFIO_AF(PF04, 0) | \
                     PIN_AFIO_AF(PF05, 0) | \
                     PIN_AFIO_AF(PF06, 0) | \
                     PIN_AFIO_AF(PF07, 0))

#define VAL_GPIOF_AFRH          (PIN_AFIO_AF(PF08, 0) | \
                     PIN_AFIO_AF(PF09, 0) | \
                     PIN_AFIO_AF(PF10, 0) | \
                     PIN_AFIO_AF(PF11, 0) | \
                     PIN_AFIO_AF(PF12, 0) | \
                     PIN_AFIO_AF(PF13, 0) | \
                     PIN_AFIO_AF(PF14, 0) | \
                     PIN_AFIO_AF(PF15, 0))

#define VAL_GPIOG_MODER                 (PIN_MODE_INPUT(PG00) | \
                     PIN_MODE_INPUT(PG01) | \
                     PIN_MODE_INPUT(PG02) | \
                     PIN_MODE_INPUT(PG03) | \
                     PIN_MODE_INPUT(PG04) | \
                     PIN_MODE_INPUT(PG05) | \
                     PIN_MODE_INPUT(PG06) | \
                     PIN_MODE_INPUT(PG07) | \
                     PIN_MODE_INPUT(PG08) | \
                     PIN_MODE_INPUT(PG09) | \
                     PIN_MODE_INPUT(PG10) | \
                     PIN_MODE_INPUT(PG11) | \
                     PIN_MODE_INPUT(PG12) | \
                     PIN_MODE_INPUT(PG13) | \
                     PIN_MODE_INPUT(PG14) | \
                     PIN_MODE_INPUT(PG15))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_PUSHPULL(PG00) | \
                     PIN_OTYPE_PUSHPULL(PG01) | \
                     PIN_OTYPE_PUSHPULL(PG02) | \
                     PIN_OTYPE_PUSHPULL(PG03) | \
                     PIN_OTYPE_PUSHPULL(PG04) | \
                     PIN_OTYPE_PUSHPULL(PG05) | \
                     PIN_OTYPE_PUSHPULL(PG06) | \
                     PIN_OTYPE_PUSHPULL(PG07) | \
                     PIN_OTYPE_PUSHPULL(PG08) | \
                     PIN_OTYPE_PUSHPULL(PG09) | \
                     PIN_OTYPE_PUSHPULL(PG10) | \
                     PIN_OTYPE_PUSHPULL(PG11) | \
                     PIN_OTYPE_PUSHPULL(PG12) | \
                     PIN_OTYPE_PUSHPULL(PG13) | \
                     PIN_OTYPE_PUSHPULL(PG14) | \
                     PIN_OTYPE_PUSHPULL(PG15))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG01) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PG15))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_PULLDOWN(PG00) | \
                     PIN_PUPDR_PULLDOWN(PG01) | \
                     PIN_PUPDR_PULLDOWN(PG02) | \
                     PIN_PUPDR_PULLDOWN(PG03) | \
                     PIN_PUPDR_PULLDOWN(PG04) | \
                     PIN_PUPDR_PULLDOWN(PG05) | \
                     PIN_PUPDR_PULLDOWN(PG06) | \
                     PIN_PUPDR_PULLDOWN(PG07) | \
                     PIN_PUPDR_PULLDOWN(PG08) | \
                     PIN_PUPDR_PULLDOWN(PG09) | \
                     PIN_PUPDR_PULLDOWN(PG10) | \
                     PIN_PUPDR_PULLDOWN(PG11) | \
                     PIN_PUPDR_PULLDOWN(PG12) | \
                     PIN_PUPDR_PULLDOWN(PG13) | \
                     PIN_PUPDR_PULLDOWN(PG14) | \
                     PIN_PUPDR_PULLDOWN(PG15))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_LOW(PG00) | \
                     PIN_ODR_LEVEL_LOW(PG01) | \
                     PIN_ODR_LEVEL_LOW(PG02) | \
                     PIN_ODR_LEVEL_LOW(PG03) | \
                     PIN_ODR_LEVEL_LOW(PG04) | \
                     PIN_ODR_LEVEL_LOW(PG05) | \
                     PIN_ODR_LEVEL_LOW(PG06) | \
                     PIN_ODR_LEVEL_LOW(PG07) | \
                     PIN_ODR_LEVEL_LOW(PG08) | \
                     PIN_ODR_LEVEL_LOW(PG09) | \
                     PIN_ODR_LEVEL_LOW(PG10) | \
                     PIN_ODR_LEVEL_LOW(PG11) | \
                     PIN_ODR_LEVEL_LOW(PG12) | \
                     PIN_ODR_LEVEL_LOW(PG13) | \
                     PIN_ODR_LEVEL_LOW(PG14) | \
                     PIN_ODR_LEVEL_LOW(PG15))

#define VAL_GPIOG_AFRL          (PIN_AFIO_AF(PG00, 0) | \
                     PIN_AFIO_AF(PG01, 0) | \
                     PIN_AFIO_AF(PG02, 0) | \
                     PIN_AFIO_AF(PG03, 0) | \
                     PIN_AFIO_AF(PG04, 0) | \
                     PIN_AFIO_AF(PG05, 0) | \
                     PIN_AFIO_AF(PG06, 0) | \
                     PIN_AFIO_AF(PG07, 0))

#define VAL_GPIOG_AFRH          (PIN_AFIO_AF(PG08, 0) | \
                     PIN_AFIO_AF(PG09, 0) | \
                     PIN_AFIO_AF(PG10, 0) | \
                     PIN_AFIO_AF(PG11, 0) | \
                     PIN_AFIO_AF(PG12, 0) | \
                     PIN_AFIO_AF(PG13, 0) | \
                     PIN_AFIO_AF(PG14, 0) | \
                     PIN_AFIO_AF(PG15, 0))

#define VAL_GPIOH_MODER                 (PIN_MODE_ALTERNATE(PH00_OSC_IN) | \
                     PIN_MODE_ALTERNATE(PH01_OSC_OUT) | \
                     PIN_MODE_INPUT(PH02) | \
                     PIN_MODE_INPUT(PH03) | \
                     PIN_MODE_INPUT(PH04) | \
                     PIN_MODE_INPUT(PH05) | \
                     PIN_MODE_INPUT(PH06) | \
                     PIN_MODE_INPUT(PH07) | \
                     PIN_MODE_INPUT(PH08) | \
                     PIN_MODE_INPUT(PH09) | \
                     PIN_MODE_INPUT(PH10) | \
                     PIN_MODE_INPUT(PH11) | \
                     PIN_MODE_INPUT(PH12) | \
                     PIN_MODE_INPUT(PH13) | \
                     PIN_MODE_INPUT(PH14) | \
                     PIN_MODE_INPUT(PH15))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(PH00_OSC_IN) | \
                     PIN_OTYPE_PUSHPULL(PH01_OSC_OUT) | \
                     PIN_OTYPE_PUSHPULL(PH02) | \
                     PIN_OTYPE_PUSHPULL(PH03) | \
                     PIN_OTYPE_PUSHPULL(PH04) | \
                     PIN_OTYPE_PUSHPULL(PH05) | \
                     PIN_OTYPE_PUSHPULL(PH06) | \
                     PIN_OTYPE_PUSHPULL(PH07) | \
                     PIN_OTYPE_PUSHPULL(PH08) | \
                     PIN_OTYPE_PUSHPULL(PH09) | \
                     PIN_OTYPE_PUSHPULL(PH10) | \
                     PIN_OTYPE_PUSHPULL(PH11) | \
                     PIN_OTYPE_PUSHPULL(PH12) | \
                     PIN_OTYPE_PUSHPULL(PH13) | \
                     PIN_OTYPE_PUSHPULL(PH14) | \
                     PIN_OTYPE_PUSHPULL(PH15))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PH00_OSC_IN) | \
                     PIN_OSPEED_SPEED_HIGH(PH01_OSC_OUT) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PH15))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(PH00_OSC_IN) | \
                     PIN_PUPDR_FLOATING(PH01_OSC_OUT) | \
                     PIN_PUPDR_PULLDOWN(PH02) | \
                     PIN_PUPDR_PULLDOWN(PH03) | \
                     PIN_PUPDR_PULLDOWN(PH04) | \
                     PIN_PUPDR_PULLDOWN(PH05) | \
                     PIN_PUPDR_PULLDOWN(PH06) | \
                     PIN_PUPDR_PULLDOWN(PH07) | \
                     PIN_PUPDR_PULLDOWN(PH08) | \
                     PIN_PUPDR_PULLDOWN(PH09) | \
                     PIN_PUPDR_PULLDOWN(PH10) | \
                     PIN_PUPDR_PULLDOWN(PH11) | \
                     PIN_PUPDR_PULLDOWN(PH12) | \
                     PIN_PUPDR_PULLDOWN(PH13) | \
                     PIN_PUPDR_PULLDOWN(PH14) | \
                     PIN_PUPDR_PULLDOWN(PH15))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(PH00_OSC_IN) | \
                     PIN_ODR_LEVEL_HIGH(PH01_OSC_OUT) | \
                     PIN_ODR_LEVEL_LOW(PH02) | \
                     PIN_ODR_LEVEL_LOW(PH03) | \
                     PIN_ODR_LEVEL_LOW(PH04) | \
                     PIN_ODR_LEVEL_LOW(PH05) | \
                     PIN_ODR_LEVEL_LOW(PH06) | \
                     PIN_ODR_LEVEL_LOW(PH07) | \
                     PIN_ODR_LEVEL_LOW(PH08) | \
                     PIN_ODR_LEVEL_LOW(PH09) | \
                     PIN_ODR_LEVEL_LOW(PH10) | \
                     PIN_ODR_LEVEL_LOW(PH11) | \
                     PIN_ODR_LEVEL_LOW(PH12) | \
                     PIN_ODR_LEVEL_LOW(PH13) | \
                     PIN_ODR_LEVEL_LOW(PH14) | \
                     PIN_ODR_LEVEL_LOW(PH15))

#define VAL_GPIOH_AFRL          (PIN_AFIO_AF(PH00_OSC_IN, 0) | \
                     PIN_AFIO_AF(PH01_OSC_OUT, 0) | \
                     PIN_AFIO_AF(PH02, 0) | \
                     PIN_AFIO_AF(PH03, 0) | \
                     PIN_AFIO_AF(PH04, 0) | \
                     PIN_AFIO_AF(PH05, 0) | \
                     PIN_AFIO_AF(PH06, 0) | \
                     PIN_AFIO_AF(PH07, 0))

#define VAL_GPIOH_AFRH          (PIN_AFIO_AF(PH08, 0) | \
                     PIN_AFIO_AF(PH09, 0) | \
                     PIN_AFIO_AF(PH10, 0) | \
                     PIN_AFIO_AF(PH11, 0) | \
                     PIN_AFIO_AF(PH12, 0) | \
                     PIN_AFIO_AF(PH13, 0) | \
                     PIN_AFIO_AF(PH14, 0) | \
                     PIN_AFIO_AF(PH15, 0))

#define VAL_GPIOI_MODER                 (PIN_MODE_INPUT(PI00) | \
                     PIN_MODE_INPUT(PI01) | \
                     PIN_MODE_INPUT(PI02) | \
                     PIN_MODE_INPUT(PI03) | \
                     PIN_MODE_INPUT(PI04) | \
                     PIN_MODE_INPUT(PI05) | \
                     PIN_MODE_INPUT(PI06) | \
                     PIN_MODE_INPUT(PI07) | \
                     PIN_MODE_INPUT(PI08) | \
                     PIN_MODE_INPUT(PI09) | \
                     PIN_MODE_INPUT(PI10) | \
                     PIN_MODE_INPUT(PI11) | \
                     PIN_MODE_INPUT(PI12) | \
                     PIN_MODE_INPUT(PI13) | \
                     PIN_MODE_INPUT(PI14) | \
                     PIN_MODE_INPUT(PI15))

#define VAL_GPIOI_OTYPER                (PIN_OTYPE_PUSHPULL(PI00) | \
                     PIN_OTYPE_PUSHPULL(PI01) | \
                     PIN_OTYPE_PUSHPULL(PI02) | \
                     PIN_OTYPE_PUSHPULL(PI03) | \
                     PIN_OTYPE_PUSHPULL(PI04) | \
                     PIN_OTYPE_PUSHPULL(PI05) | \
                     PIN_OTYPE_PUSHPULL(PI06) | \
                     PIN_OTYPE_PUSHPULL(PI07) | \
                     PIN_OTYPE_PUSHPULL(PI08) | \
                     PIN_OTYPE_PUSHPULL(PI09) | \
                     PIN_OTYPE_PUSHPULL(PI10) | \
                     PIN_OTYPE_PUSHPULL(PI11) | \
                     PIN_OTYPE_PUSHPULL(PI12) | \
                     PIN_OTYPE_PUSHPULL(PI13) | \
                     PIN_OTYPE_PUSHPULL(PI14) | \
                     PIN_OTYPE_PUSHPULL(PI15))

#define VAL_GPIOI_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PI00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI01) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PI15))

#define VAL_GPIOI_PUPDR                 (PIN_PUPDR_PULLDOWN(PI00) | \
                     PIN_PUPDR_PULLDOWN(PI01) | \
                     PIN_PUPDR_PULLDOWN(PI02) | \
                     PIN_PUPDR_PULLDOWN(PI03) | \
                     PIN_PUPDR_PULLDOWN(PI04) | \
                     PIN_PUPDR_PULLDOWN(PI05) | \
                     PIN_PUPDR_PULLDOWN(PI06) | \
                     PIN_PUPDR_PULLDOWN(PI07) | \
                     PIN_PUPDR_PULLDOWN(PI08) | \
                     PIN_PUPDR_PULLDOWN(PI09) | \
                     PIN_PUPDR_PULLDOWN(PI10) | \
                     PIN_PUPDR_PULLDOWN(PI11) | \
                     PIN_PUPDR_PULLDOWN(PI12) | \
                     PIN_PUPDR_PULLDOWN(PI13) | \
                     PIN_PUPDR_PULLDOWN(PI14) | \
                     PIN_PUPDR_PULLDOWN(PI15))

#define VAL_GPIOI_ODR                   (PIN_ODR_LEVEL_LOW(PI00) | \
                     PIN_ODR_LEVEL_LOW(PI01) | \
                     PIN_ODR_LEVEL_LOW(PI02) | \
                     PIN_ODR_LEVEL_LOW(PI03) | \
                     PIN_ODR_LEVEL_LOW(PI04) | \
                     PIN_ODR_LEVEL_LOW(PI05) | \
                     PIN_ODR_LEVEL_LOW(PI06) | \
                     PIN_ODR_LEVEL_LOW(PI07) | \
                     PIN_ODR_LEVEL_LOW(PI08) | \
                     PIN_ODR_LEVEL_LOW(PI09) | \
                     PIN_ODR_LEVEL_LOW(PI10) | \
                     PIN_ODR_LEVEL_LOW(PI11) | \
                     PIN_ODR_LEVEL_LOW(PI12) | \
                     PIN_ODR_LEVEL_LOW(PI13) | \
                     PIN_ODR_LEVEL_LOW(PI14) | \
                     PIN_ODR_LEVEL_LOW(PI15))

#define VAL_GPIOI_AFRL          (PIN_AFIO_AF(PI00, 0) | \
                     PIN_AFIO_AF(PI01, 0) | \
                     PIN_AFIO_AF(PI02, 0) | \
                     PIN_AFIO_AF(PI03, 0) | \
                     PIN_AFIO_AF(PI04, 0) | \
                     PIN_AFIO_AF(PI05, 0) | \
                     PIN_AFIO_AF(PI06, 0) | \
                     PIN_AFIO_AF(PI07, 0))

#define VAL_GPIOI_AFRH          (PIN_AFIO_AF(PI08, 0) | \
                     PIN_AFIO_AF(PI09, 0) | \
                     PIN_AFIO_AF(PI10, 0) | \
                     PIN_AFIO_AF(PI11, 0) | \
                     PIN_AFIO_AF(PI12, 0) | \
                     PIN_AFIO_AF(PI13, 0) | \
                     PIN_AFIO_AF(PI14, 0) | \
                     PIN_AFIO_AF(PI15, 0))

#define VAL_GPIOJ_MODER                 (PIN_MODE_INPUT(PJ00) | \
                     PIN_MODE_INPUT(PJ01) | \
                     PIN_MODE_INPUT(PJ02) | \
                     PIN_MODE_INPUT(PJ03) | \
                     PIN_MODE_INPUT(PJ04) | \
                     PIN_MODE_INPUT(PJ05) | \
                     PIN_MODE_INPUT(PJ06) | \
                     PIN_MODE_INPUT(PJ07) | \
                     PIN_MODE_INPUT(PJ08) | \
                     PIN_MODE_INPUT(PJ09) | \
                     PIN_MODE_INPUT(PJ10) | \
                     PIN_MODE_INPUT(PJ11) | \
                     PIN_MODE_INPUT(PJ12) | \
                     PIN_MODE_INPUT(PJ13) | \
                     PIN_MODE_INPUT(PJ14) | \
                     PIN_MODE_INPUT(PJ15))

#define VAL_GPIOJ_OTYPER                (PIN_OTYPE_PUSHPULL(PJ00) | \
                     PIN_OTYPE_PUSHPULL(PJ01) | \
                     PIN_OTYPE_PUSHPULL(PJ02) | \
                     PIN_OTYPE_PUSHPULL(PJ03) | \
                     PIN_OTYPE_PUSHPULL(PJ04) | \
                     PIN_OTYPE_PUSHPULL(PJ05) | \
                     PIN_OTYPE_PUSHPULL(PJ06) | \
                     PIN_OTYPE_PUSHPULL(PJ07) | \
                     PIN_OTYPE_PUSHPULL(PJ08) | \
                     PIN_OTYPE_PUSHPULL(PJ09) | \
                     PIN_OTYPE_PUSHPULL(PJ10) | \
                     PIN_OTYPE_PUSHPULL(PJ11) | \
                     PIN_OTYPE_PUSHPULL(PJ12) | \
                     PIN_OTYPE_PUSHPULL(PJ13) | \
                     PIN_OTYPE_PUSHPULL(PJ14) | \
                     PIN_OTYPE_PUSHPULL(PJ15))

#define VAL_GPIOJ_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PJ00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ01) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PJ15))

#define VAL_GPIOJ_PUPDR                 (PIN_PUPDR_PULLDOWN(PJ00) | \
                     PIN_PUPDR_PULLDOWN(PJ01) | \
                     PIN_PUPDR_PULLDOWN(PJ02) | \
                     PIN_PUPDR_PULLDOWN(PJ03) | \
                     PIN_PUPDR_PULLDOWN(PJ04) | \
                     PIN_PUPDR_PULLDOWN(PJ05) | \
                     PIN_PUPDR_PULLDOWN(PJ06) | \
                     PIN_PUPDR_PULLDOWN(PJ07) | \
                     PIN_PUPDR_PULLDOWN(PJ08) | \
                     PIN_PUPDR_PULLDOWN(PJ09) | \
                     PIN_PUPDR_PULLDOWN(PJ10) | \
                     PIN_PUPDR_PULLDOWN(PJ11) | \
                     PIN_PUPDR_PULLDOWN(PJ12) | \
                     PIN_PUPDR_PULLDOWN(PJ13) | \
                     PIN_PUPDR_PULLDOWN(PJ14) | \
                     PIN_PUPDR_PULLDOWN(PJ15))

#define VAL_GPIOJ_ODR                   (PIN_ODR_LEVEL_LOW(PJ00) | \
                     PIN_ODR_LEVEL_LOW(PJ01) | \
                     PIN_ODR_LEVEL_LOW(PJ02) | \
                     PIN_ODR_LEVEL_LOW(PJ03) | \
                     PIN_ODR_LEVEL_LOW(PJ04) | \
                     PIN_ODR_LEVEL_LOW(PJ05) | \
                     PIN_ODR_LEVEL_LOW(PJ06) | \
                     PIN_ODR_LEVEL_LOW(PJ07) | \
                     PIN_ODR_LEVEL_LOW(PJ08) | \
                     PIN_ODR_LEVEL_LOW(PJ09) | \
                     PIN_ODR_LEVEL_LOW(PJ10) | \
                     PIN_ODR_LEVEL_LOW(PJ11) | \
                     PIN_ODR_LEVEL_LOW(PJ12) | \
                     PIN_ODR_LEVEL_LOW(PJ13) | \
                     PIN_ODR_LEVEL_LOW(PJ14) | \
                     PIN_ODR_LEVEL_LOW(PJ15))

#define VAL_GPIOJ_AFRL          (PIN_AFIO_AF(PJ00, 0) | \
                     PIN_AFIO_AF(PJ01, 0) | \
                     PIN_AFIO_AF(PJ02, 0) | \
                     PIN_AFIO_AF(PJ03, 0) | \
                     PIN_AFIO_AF(PJ04, 0) | \
                     PIN_AFIO_AF(PJ05, 0) | \
                     PIN_AFIO_AF(PJ06, 0) | \
                     PIN_AFIO_AF(PJ07, 0))

#define VAL_GPIOJ_AFRH          (PIN_AFIO_AF(PJ08, 0) | \
                     PIN_AFIO_AF(PJ09, 0) | \
                     PIN_AFIO_AF(PJ10, 0) | \
                     PIN_AFIO_AF(PJ11, 0) | \
                     PIN_AFIO_AF(PJ12, 0) | \
                     PIN_AFIO_AF(PJ13, 0) | \
                     PIN_AFIO_AF(PJ14, 0) | \
                     PIN_AFIO_AF(PJ15, 0))

#define VAL_GPIOK_MODER                 (PIN_MODE_INPUT(PK00) | \
                     PIN_MODE_INPUT(PK01) | \
                     PIN_MODE_INPUT(PK02) | \
                     PIN_MODE_INPUT(PK03) | \
                     PIN_MODE_INPUT(PK04) | \
                     PIN_MODE_INPUT(PK05) | \
                     PIN_MODE_INPUT(PK06) | \
                     PIN_MODE_INPUT(PK07) | \
                     PIN_MODE_INPUT(PK08) | \
                     PIN_MODE_INPUT(PK09) | \
                     PIN_MODE_INPUT(PK10) | \
                     PIN_MODE_INPUT(PK11) | \
                     PIN_MODE_INPUT(PK12) | \
                     PIN_MODE_INPUT(PK13) | \
                     PIN_MODE_INPUT(PK14) | \
                     PIN_MODE_INPUT(PK15))

#define VAL_GPIOK_OTYPER                (PIN_OTYPE_PUSHPULL(PK00) | \
                     PIN_OTYPE_PUSHPULL(PK01) | \
                     PIN_OTYPE_PUSHPULL(PK02) | \
                     PIN_OTYPE_PUSHPULL(PK03) | \
                     PIN_OTYPE_PUSHPULL(PK04) | \
                     PIN_OTYPE_PUSHPULL(PK05) | \
                     PIN_OTYPE_PUSHPULL(PK06) | \
                     PIN_OTYPE_PUSHPULL(PK07) | \
                     PIN_OTYPE_PUSHPULL(PK08) | \
                     PIN_OTYPE_PUSHPULL(PK09) | \
                     PIN_OTYPE_PUSHPULL(PK10) | \
                     PIN_OTYPE_PUSHPULL(PK11) | \
                     PIN_OTYPE_PUSHPULL(PK12) | \
                     PIN_OTYPE_PUSHPULL(PK13) | \
                     PIN_OTYPE_PUSHPULL(PK14) | \
                     PIN_OTYPE_PUSHPULL(PK15))

#define VAL_GPIOK_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PK00) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK01) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK02) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK03) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK04) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK05) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK06) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK07) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK08) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK09) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK10) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK11) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK12) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK13) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK14) | \
                     PIN_OSPEED_SPEED_VERYLOW(PK15))

#define VAL_GPIOK_PUPDR                 (PIN_PUPDR_PULLDOWN(PK00) | \
                     PIN_PUPDR_PULLDOWN(PK01) | \
                     PIN_PUPDR_PULLDOWN(PK02) | \
                     PIN_PUPDR_PULLDOWN(PK03) | \
                     PIN_PUPDR_PULLDOWN(PK04) | \
                     PIN_PUPDR_PULLDOWN(PK05) | \
                     PIN_PUPDR_PULLDOWN(PK06) | \
                     PIN_PUPDR_PULLDOWN(PK07) | \
                     PIN_PUPDR_PULLDOWN(PK08) | \
                     PIN_PUPDR_PULLDOWN(PK09) | \
                     PIN_PUPDR_PULLDOWN(PK10) | \
                     PIN_PUPDR_PULLDOWN(PK11) | \
                     PIN_PUPDR_PULLDOWN(PK12) | \
                     PIN_PUPDR_PULLDOWN(PK13) | \
                     PIN_PUPDR_PULLDOWN(PK14) | \
                     PIN_PUPDR_PULLDOWN(PK15))

#define VAL_GPIOK_ODR                   (PIN_ODR_LEVEL_LOW(PK00) | \
                     PIN_ODR_LEVEL_LOW(PK01) | \
                     PIN_ODR_LEVEL_LOW(PK02) | \
                     PIN_ODR_LEVEL_LOW(PK03) | \
                     PIN_ODR_LEVEL_LOW(PK04) | \
                     PIN_ODR_LEVEL_LOW(PK05) | \
                     PIN_ODR_LEVEL_LOW(PK06) | \
                     PIN_ODR_LEVEL_LOW(PK07) | \
                     PIN_ODR_LEVEL_LOW(PK08) | \
                     PIN_ODR_LEVEL_LOW(PK09) | \
                     PIN_ODR_LEVEL_LOW(PK10) | \
                     PIN_ODR_LEVEL_LOW(PK11) | \
                     PIN_ODR_LEVEL_LOW(PK12) | \
                     PIN_ODR_LEVEL_LOW(PK13) | \
                     PIN_ODR_LEVEL_LOW(PK14) | \
                     PIN_ODR_LEVEL_LOW(PK15))

#define VAL_GPIOK_AFRL          (PIN_AFIO_AF(PK00, 0) | \
                     PIN_AFIO_AF(PK01, 0) | \
                     PIN_AFIO_AF(PK02, 0) | \
                     PIN_AFIO_AF(PK03, 0) | \
                     PIN_AFIO_AF(PK04, 0) | \
                     PIN_AFIO_AF(PK05, 0) | \
                     PIN_AFIO_AF(PK06, 0) | \
                     PIN_AFIO_AF(PK07, 0))

#define VAL_GPIOK_AFRH          (PIN_AFIO_AF(PK08, 0) | \
                     PIN_AFIO_AF(PK09, 0) | \
                     PIN_AFIO_AF(PK10, 0) | \
                     PIN_AFIO_AF(PK11, 0) | \
                     PIN_AFIO_AF(PK12, 0) | \
                     PIN_AFIO_AF(PK13, 0) | \
                     PIN_AFIO_AF(PK14, 0) | \
                     PIN_AFIO_AF(PK15, 0))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
