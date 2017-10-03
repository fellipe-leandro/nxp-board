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
 * @file mag.c
 *
 * This module implements the driver for MAG3110 magnetometer.
 *
 * Current implementation includes functions for basic settings of the device,
 * access to registers and reading temperature.
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "mag.h"
#include "fsl_i2c.h"
#include "wait.h"

/* Enables debugging messages printed to the virtual serial port. */
/* #define DEBUG */

#ifdef DEBUG
#include "fsl_debug_console.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! Boot time applied after SW reset in ms. */
#define MAG_BOOT_TIME_MS            2U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
#ifdef DEBUG
/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_PrintReg
 * Description   : This function prints a selected register to the virtual
 *                 serial output.
 *
 *END**************************************************************************/
void MAG_PrintReg(mag_drv_data_t *drvData, uint8_t regAddr, char *regName)
{
    status_t status = kStatus_Success;
    uint8_t regVal = 0U;

    status = MAG_ReadRegs(drvData, regAddr, &regVal, 1U);
    if (status != kStatus_Success)
    {
        PRINTF("\tAn error occurred in the MAG_PrintReg function (%d), reg. %s (0x%02x)\r\n",
                status, regName, regAddr);
    }
    else
    {
        PRINTF("%s (0x%02x):\t0x%02x\t\r\n", regName, regAddr, regVal);
    }
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_Init
 * Description   : Initializes the magnetometer driver based on user
 *                 configuration.
 *
 *END**************************************************************************/
status_t MAG_Init(mag_drv_data_t *drvData, const mag_user_config_t *userConfig)
{
    status_t status = kStatus_Success;
    uint8_t  regVal = 0U;

    drvData->i2cBase = userConfig->i2cBase;
    drvData->i2cSlaveAddr = userConfig->i2cSlaveAddr;
    drvData->tempOffsetDegC = userConfig->tempOffsetDegC;

    /* Wait for boot time. */
    WaitMS(MAG_BOOT_TIME_MS);

    /* Set the standby mode. */
    status |= MAG_SetOpMode(drvData, false);
#ifdef DEBUG
    MAG_PrintReg(drvData, MAG_REG_CTRL_REG1, "CTRL_REG1");
#endif

    /* Set Automatic Magnetic Sensor Reset.
     * Note: this is a write only bit and always reads back as 0. */
    regVal = (userConfig->autoMrstEn) ? MAG_EN_AUTO_MRST : MAG_DIS_AUTO_MRST;
    status |= MAG_WriteReg(drvData, MAG_REG_CTRL_REG2, regVal);

    /* Set output data rate to 10 Hz and over sampling ration to 8 (CTRL_REG1). */
    regVal = MAG_GET_DATA_RATE(0x00U);
    regVal |= MAG_GET_OVER_SAMP(0x03U);

    /* Set operation mode (CTRL_REG1). */
    regVal |= (userConfig->modeActive) ? MAG_SET_ACTIVE_AC : MAG_SET_STANDBY_AC;

    status |= MAG_WriteReg(drvData, MAG_REG_CTRL_REG1, regVal);
#ifdef DEBUG
    MAG_PrintReg(drvData, MAG_REG_CTRL_REG1, "CTRL_REG1");
#endif

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_WriteReg
 * Description   : Writes a byte to the device using an 8bit memory address.
 *
 *END**************************************************************************/
status_t MAG_WriteReg(mag_drv_data_t *drvData, uint8_t regAddr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = drvData->i2cSlaveAddr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = regAddr;
    masterXfer.subaddressSize = 1U;
    masterXfer.data = &value;
    masterXfer.dataSize = 1U;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    return I2C_MasterTransferBlocking(drvData->i2cBase, &masterXfer);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_ReadRegs
 * Description   : Reads bytes from the device using an 8bit memory address.
 *
 *END**************************************************************************/
status_t MAG_ReadRegs(mag_drv_data_t *drvData, uint8_t regAddr,
        uint8_t *valueArr, uint32_t valueArrSize)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = drvData->i2cSlaveAddr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = regAddr;
    masterXfer.subaddressSize = 1U;
    masterXfer.data = valueArr;
    masterXfer.dataSize = (size_t)valueArrSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    return I2C_MasterTransferBlocking(drvData->i2cBase, &masterXfer);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_UpdateReg
 * Description   : Updates a byte of the device using an 8bit memory address.
 *
 *END**************************************************************************/
status_t MAG_UpdateReg(mag_drv_data_t *drvData, uint8_t regAddr,
        uint8_t value, uint8_t mask)
{
    uint8_t  regVal = 0U;   /* Value of a register. */
    status_t status = kStatus_Success;

    status = MAG_ReadRegs(drvData, regAddr, &regVal, 1U);

    regVal = MAG_UNSET_BIT_VALUE(regVal, mask);
    regVal = MAG_SET_BIT_VALUE(regVal, value & mask);

    status |= MAG_WriteReg(drvData, regAddr, regVal);
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_SetOpMode
 * Description   : This function sets the device operation mode (normal or
 *                 standby).
 *
 *END**************************************************************************/
status_t MAG_SetOpMode(mag_drv_data_t *drvData, bool active)
{
    /* Value of a register. */
    uint8_t  regVal = active ? MAG_SET_ACTIVE_AC : MAG_SET_STANDBY_AC;

    return MAG_UpdateReg(drvData, MAG_REG_CTRL_REG1, regVal, MAG_AC_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_TriggerMeas
 * Description   : This function triggers measurement.
 *
 *END**************************************************************************/
status_t MAG_TriggerMeas(mag_drv_data_t *drvData)
{
    return MAG_UpdateReg(drvData, MAG_REG_CTRL_REG1, MAG_TRIGGER_MEASUREMENT,
            MAG_TM_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_IsMeasCompl
 * Description   : This function checks if the measured is complete.
 *
 *END**************************************************************************/
status_t MAG_IsMeasCompl(mag_drv_data_t *drvData, bool *complete)
{
    status_t status = kStatus_Success;
    uint8_t  regVal = 0U;

    status = MAG_ReadRegs(drvData, MAG_REG_DR_STATUS, &regVal, 1U);
    (*complete) = (regVal & MAG_ZYXDR_MASK) != 0U;

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_GetAllAxisRaw
 * Description   : This function gets raw values of all axis.
 *
 *END**************************************************************************/
status_t MAG_GetAllAxisRaw(mag_drv_data_t *drvData, uint16_t *x, uint16_t *y,
        uint16_t *z)
{
    status_t status = kStatus_Success;
    uint8_t  rxBuffer[6];

    status = MAG_ReadRegs(drvData, MAG_REG_DR_STATUS, rxBuffer, sizeof(rxBuffer));

    (*x) = MAG_GET_REG_OUT_RAW(rxBuffer[0], rxBuffer[1]);
    (*y) = MAG_GET_REG_OUT_RAW(rxBuffer[2], rxBuffer[3]);
    (*z) = MAG_GET_REG_OUT_RAW(rxBuffer[4], rxBuffer[5]);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MAG_GetTempDegC
 * Description   : This function gets temperature in °C.
 *
 *END**************************************************************************/
status_t MAG_GetTempDegC(mag_drv_data_t *drvData, int8_t *tempDegC)
{
    status_t status = kStatus_Success;
    uint8_t  regVal = 0U;

    status = MAG_ReadRegs(drvData, MAG_REG_DIE_TEMP, &regVal, 1U);
    (*tempDegC) = MAG_GET_TEMP_DEGC(regVal);
    /* Apply the temperature offset. */
    (*tempDegC) += drvData->tempOffsetDegC;

    return status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
