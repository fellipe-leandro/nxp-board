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
 * @file sensors.h
 *
 * This file implements high level functions to access sensors.
 */

#ifndef SOURCE_SENSORS_H_
#define SOURCE_SENSORS_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "mag3110/mag.h"
#include "mma8451q/mma.h"
#include "light_sensor/ls.h"
#include "fsl_lptmr.h"

/*******************************************************************************
 * API
 *******************************************************************************/
/*!
 * @brief This function measures the temperature using the magnetometer sensor.
 *
 * @param magDrvData Magnetometer driver data
 * @param tempDegC Temperature in deg. C
 *
 * @return Status of the function (kstatus_Success on success).
 */
status_t MeasureMagTemp(mag_drv_data_t *magDrvData, int8_t *tempDegC);

/*!
 * @brief This function clears the interrupt flag in a accelerometer register.
 *
 * @param mmaDrvData Accelerometer driver data
 *
 * @return Status of the function (kstatus_Success on success).
 */
status_t ClearAccelInt(mma_drv_data_t *mmaDrvData);

/*!
 * @brief This function measures and calculater the total acceleration using
 * the accelerometer sensor.
 *
 * @param magDrvData Accelerometer driver data
 * @param tempDegC Total acceleration in mg.
 *
 * @return Status of the function (kstatus_Success on success).
 */
status_t GetTotalAccel(mma_drv_data_t *mmaDrvData, uint16_t *aTotal);

#endif /* SOURCE_SENSORS_H_ */

/*******************************************************************************
 * EOF
 *******************************************************************************/
