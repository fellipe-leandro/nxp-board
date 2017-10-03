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
 * @file mag.h
 *
 * This module implements the driver for MAG3110 magnetometer.
 *
 * Current implementation includes functions for basic settings of the device,
 * access to registers and reading temperature.
 */

#ifndef SOURCE_MAG_H_
#define SOURCE_MAG_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_common.h"
#include "mag_mag3110.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup mag_struct_group
 * Structures for the magnetometer driver.
 * @{
 */
/*! @brief User configuration of the magnetometer. */
typedef struct
{
    I2C_Type *i2cBase;              /*!< Pointer to a configured I2C peripheral
                                         (I2C base). */
    uint8_t i2cSlaveAddr;           /*!< I2C slave device address */
    bool modeActive;                /*!< Selection of the device operation mode
                                         (true - active, false - standby). */
    bool autoMrstEn;                /*!< Configuration of automatic Magnetic
                                         Sensor Reset. */
    int8_t tempOffsetDegC;          /*!< Temperature offset in °C applied
                                         to measured temperature.*/
} mag_user_config_t;

/*! @brief This data structure is used by the magnetometer driver (this is the first
 * parameter of most functions). */
typedef struct
{
    I2C_Type *i2cBase;              /*!< Pointer to a configured I2C peripheral
                                         (I2C base). */
    uint8_t i2cSlaveAddr;           /*!< I2C slave device address */
    int8_t tempOffsetDegC;          /*!< Temperature offset in °C applied
                                         on measured temperature.*/
} mag_drv_data_t;
/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup mma_functions_group
 * @{
 */
#ifdef DEBUG
/*!
 * @brief This function prints a selected register to the virtual serial output.
 *
 * @param drvData Driver run-time data.
 * @param regAddr Memory data address.
 * @param regName Name of a register.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
void MAG_PrintReg(mag_drv_data_t *drvData, uint8_t regAddr, char *regName);
#endif

/*!
 * @brief Initializes the magnetometer driver based on user configuration.
 *
 * @param drvData    Driver run-time data.
 * @param userConfig User-provided configuration of the driver.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_Init(mag_drv_data_t *drvData, const mag_user_config_t *userConfig);

/*!
 * @brief Writes a byte to the device using an 8bit memory address.
 *
 * @param drvData Driver run-time data.
 * @param regAddr Device memory address.
 * @param value   Data value.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_WriteReg(mag_drv_data_t *drvData, uint8_t regAddr, uint8_t value);

/*!
 * @brief Reads bytes from the device using an 8bit memory address.
 *
 * @param drvData      Driver run-time data.
 * @param regAddr      Device memory address.
 * @param valueArr     Pointer where the read data are stored.
 * @param valueArrSize Size of the data array (parameter valueArr) in bytes.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_ReadRegs(mag_drv_data_t *drvData, uint8_t regAddr,
        uint8_t *valueArr, uint32_t valueArrSize);

/*!
 * @brief Updates a byte of the device using an 8bit memory address.
 *
 * @param drvData Driver run-time data.
 * @param regAddr Device memory address.
 * @param value   Data value.
 * @param mask    Mask of the data value. Only set bits are updated.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_UpdateReg(mag_drv_data_t *drvData, uint8_t regAddr,
        uint8_t value, uint8_t mask);

/*!
 * @brief This function sets the device operation mode (normal or standby).
 *
 * @param drvData Driver run-time data.
 * @param active  Desired device mode (true - active, false - standby).
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_SetOpMode(mag_drv_data_t *drvData, bool active);

/*!
 * @brief This function triggers measurement.
 *
 * Note that TM (Trigger Measurement) is cleared after the measurement.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_TriggerMeas(mag_drv_data_t *drvData);

/*!
 * @brief This function checks if the measured is complete.
 *
 * It reads the DR_STATUS register and checks ZYXDR bit. Note that the X/Y/Z data
 * registers must be read to clear the flag.
 *
 * @param drvData  Driver run-time data.
 * @param complete Pointer where the result is stored (true - measurement
 * complete, false - not complete).
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_IsMeasCompl(mag_drv_data_t *drvData, bool *complete);

/*!
 * @brief This function gets raw values of all axis.
 *
 * @param drvData Driver run-time data.
 * @param x       Pointer where the X-axis data are stored.
 * @param y       Pointer where the Y-axis data are stored.
 * @param z       Pointer where the Z-axis data are stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_GetAllAxisRaw(mag_drv_data_t *drvData, uint16_t *x, uint16_t *y,
        uint16_t *z);

/*!
 * @brief This function gets temperature in °C.
 *
 * @param drvData  Driver run-time data.
 * @param tempDegC Pointer where the resulting temperature is stored in °C.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MAG_GetTempDegC(mag_drv_data_t *drvData, int8_t *tempDegC);
/*! @} */

#endif /* SOURCE_MAG_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
