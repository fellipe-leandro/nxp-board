/*
 * Copyright (c) 2016, NXP B.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of  NXP B.V. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file setup.h
 *
 * This file contains functions for initialization of peripherals, sensors
 * and the Sigfox device.
 */

/*******************************************************************************
 * OL2385 Freedom board connections to the KL43Z freedom board
 *
 * Pin function  | OL2385 pos. | FRDM-KL43 pin | FRDM-KL43 pos.
 * -----------------------------------------------------------------------------
 * CLK           | P14         | PTD5          | J2-12
 * MISO          | P13         | PTD7          | J2-10
 * MOSI          | P16         | PTD6          | J2-8
 * CS            | P15         | PTD4          | J2-6
 * ACK           | P17         | PTD2          | J2-4
 * SW1           | P20         | PTA12         | J1-8
 * SW2           | P10         | PTA4          | J1-10
 *******************************************************************************/

#ifndef SOURCE_SETUP_H_
#define SOURCE_SETUP_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "mag3110/mag.h"
#include "mma8451q/mma.h"
#include "light_sensor/ls.h"
#include "sf/sf.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/
/*! @name ACK GPIO pin
 * GPIO pin used by Sigfox device
 * @{ */
#define SF_ACK_INST                 instanceD
#define SF_ACK_GPIO                 GPIOD
#define SF_ACK_PORT                 PORTD
#define SF_ACK_PIN                  2U
/*! @} */

/*! @name CS GPIO pin
 * GPIO pin used by Sigfox device
 * @{ */
#define SF_CS_INST                  instanceD
#define SF_CS_GPIO                  GPIOD
#define SF_CS_PORT                  PORTD
#define SF_CS_PIN                   4U
/*! @} */

/*! SPI used by Sigfox device (SPI 1). */
#define SF_SPI_INST                 1U

/*! @name I2C settings for the magnetometer
 * @{ */
#define MAG_MMA_I2C_BAUDRATE        100000U
#define MAG_MMA_I2C_BASE            I2C0
#define MAG_I2C_SLAVE_ADDR          0x0EU
/* Accelerometer I2C slave address. */
#define MMA_I2C_SLAVE_ADDR          0x1DU
/*! @} */

/*! @name Light sensor settings
 * @{ */
#define LS_ADC_BASE                 ADC0
#define LS_ADC_CHANNEL_GROUP        0U
#define LS_ADC_CHANNEL              3U /* PTE22, ADC0_SE3 */
/*! @} */

/*!
 * @name Accelerometer's INT1 pin
 * @{
 */
/*! GPIO selection. */
#define MMA_INT1_GPIO               GPIOC
/*! Port selection. */
#define MMA_INT1_PORT               PORTC
/*! Pin selection. */
#define MMA_INT1_PIN                5U
/*! Pin mask. */
#define MMA_INT1_PIN_MASK           (1U << MMA_INT1_PIN)
/*! Interrupt vector number. */
#define MMA_INT1_IRQn               PORTC_PORTD_IRQn
/*! Interrupt handler. */
#define MMA_INT1_IRQ_HANDLER        PORTCD_IRQHandler
/*! @} */

/*!
 * @name LPTMR settings
 * @{
 */
/*! LPTMR interrupt handler. */
#define LPTMR_IRQ_HANDLER           LPTMR0_IRQHandler
/*! Interrupt vector number. */
#define LPTMR_IRQn                  LPTMR0_IRQn
#define LPTMR_DEV                   LPTMR0
#define GET_LPTMR_CLK()             (CLOCK_GetFreq(kCLOCK_LpoClk))
/*! LPTMR prescaler value. LPTMR clock source is 1000 Hz, prescaler is set to
 * 128. */
#define LPTMR_PRESC_VAL             128U
#define GET_LPTMR_PERIOD_TICKS(periodSec) \
    (((uint32_t)(periodSec) * GET_LPTMR_CLK()) / LPTMR_PRESC_VAL)
/*! @} */

/*!
 * @name Settings of OL2385 push button SW1
 * @{
 */
/*! GPIO selection. */
#define SW1_GPIO                    GPIOA
/*! Port selection. */
#define SW1_PORT                    PORTA
/*! Pin selection. */
#define SW1_PIN                     12U
/*! Pin mask. */
#define SW1_PIN_MASK                (1U << SW1_PIN)
/*! @} */

/*!
 * @name Settings of OL2385 push button SW2
 * @{
 */
/*! GPIO selection. */
#define SW2_GPIO                    GPIOA
/*! Port selection. */
#define SW2_PORT                    PORTA
/*! Pin selection. */
#define SW2_PIN                     4U
/*! Pin mask. */
#define SW2_PIN_MASK                (1U << SW2_PIN)
/*! @} */

/*!
 * @name Settings of OL2385 push button SW3
 * @{
 */
/*! GPIO selection. */
#define SW3_GPIO                    GPIOA
/*! Port selection. */
#define SW3_PORT                    PORTA
/*! Pin selection. */
#define SW3_PIN                     5U
/*! Pin mask. */
#define SW3_PIN_MASK                (1U << SW3_PIN)
/*! @} */

/*! IRQ handler for SW1, SW2 and SW3 push buttons. */
#define SW1_2_3_IRQ_HANDLER         PORTA_IRQHandler

/*! Interrupt vector number for SW push buttons 1, 2 and 3. */
#define SW1_2_3_IRQn                PORTA_IRQn

/*******************************************************************************
 * API
 *******************************************************************************/
/*!
 * @brief This function initializes all devices (sensors, peripherals,
 * Sigfox device) used by the application.
 *
 * @return Status of the function (kstatus_Success on success).
 */
status_t SetupDevices(mag_drv_data_t *magDrvPtr, mma_drv_data_t *mmaDrvPtr,
        adc16_channel_config_t *lsAdcChnlCfgPtr, sf_drv_data_t *sfDrvPtr);

/*!
 * @brief This function sets period of LPTMR peripheral and then resets the timer.
 *
 * @param periodTicks Period value in ticks of clock source.
 */
void SetLPTMRPeriodAndReset(uint32_t periodTicks);

/*!
 * @brief This function switches MCU power mode to Very Low Power Stop mode.
 */
void SetVlpsMode(void);

/*!
 * @brief This function switches MCU power mode to Very Low Power Run mode.
 *
 * @return Status of the function (kstatus_Success on success).
 */
status_t SetVlprMode(void);

#endif /* SOURCE_SETUP_H_ */

/*******************************************************************************
 * EOF
 *******************************************************************************/
