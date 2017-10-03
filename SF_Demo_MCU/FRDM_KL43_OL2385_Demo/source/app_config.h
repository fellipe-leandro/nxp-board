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
 * @file app_config.h
 *
 * This file contains general configuration of the application behavior.
 */

#ifndef SOURCE_APP_CONFIG_H_
#define SOURCE_APP_CONFIG_H_

/* This macro enables printing of messages to the virtual serial console.
 * It should be commented out to lower power consumption.
 * Note: it prints message understandable to the user. */
#define PRINT_CONSOLE

/*! This macro enables the sending of data to SigFox network (true - enabled,
 * false - disabled). */
#define SEND_TO_SIGFOX_EN           true

/*! The transient threshold in mg. Admissible range is <0; 8000> mg. Currently
 * the max. is 4000 (measuring in <0; 4000> mg). */
#define MMA_THRESHOLD_MG            2000U

/*! Time in seconds used to wait after an accelerometer/pushbutton event detection.
 * When there is detected an event measured data are sent to SigFox
 * network and the application waits for this time and does not react to any
 * event (accelerometer, buttons). Admissible range is {0, .., 65 535} seconds
 * (0 - no debounce time).
 * Note that the timer starts after data are sent to SigFox. */
#define DEBOUNCE_TIME_S             0U

/*! This macro enables the sending of data SigFox network periodically
 * (true - enabled, false - disabled). */
#define SEND_PERIODICALLY_EN        true
/*! Data are sent to SigFox network periodically. This macro defines the period
 * of data sending. Max. value is 7 200 s. */
#define SEND_PERIOD_S               (30U * 60U)

/*! This macro enables transition of the MCU into the VLPS power mode. */
#define MCU_SLEEP_MODE_EN           true

/*! This macro enables transition of OL2385 device into the low power mode. */
#define SF_SLEEP_MODE_EN            true

/* This macro enables debugging messages printed to the virtual serial console.
 * It should be commented out to lower power consumption.
 * Note: it is intended for development only. */
/* #define DEBUG */

#endif /* SOURCE_APP_CONFIG_H_ */
