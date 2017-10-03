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
 * @file mma.c
 *
 * This module implements the driver for MMA8451Q accelerometer.
 *
 * Current implementation includes functions for basic settings of the device,
 * transient detection configuration, access to registers and its interpretation.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "mma.h"
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
/*! Boot time applied after SW reset in us. */
#define MMA_BOOT_TIME_US            500U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief This function converts a raw unsigned axis value to signed mg.
 *
 * @param drvData Driver run-time data.
 * @param axisRaw Raw value of an axis.
 *
 * @return Axis value in mg.
 */
static int16_t MMA_ConvAxisRawTomg(mma_drv_data_t *drvData, uint16_t axisRaw);

/*******************************************************************************
 * Code - internal functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_ConvAxisRawTomg
 * Description   : This function converts a raw unsigned axis value to signed mg.
 *
 *END**************************************************************************/
static int16_t MMA_ConvAxisRawTomg(mma_drv_data_t *drvData, uint16_t axisRaw)
{
    int16_t axismg = 0;

    if (drvData->bitRes == mmaBitRes14)
    {   /* Remove 2 bits (16 bits to 14 bits) with MSb extension. */
        axisRaw = ((axisRaw & 0x8000U) == 0U) ? (axisRaw >> 2U) :
                ((axisRaw >> 2U) | 0xC000U);
    }
    else
    {   /* Not implemented for the 8 bit resolution. */
        assert("Not implemented for the 8 bit resolution.");
    }

    axismg = (int16_t)axisRaw;

    /* Conversion from raw signed value to mg. Note that mg/LSB depends on
     * bit resolution (8 or 14 bits) and full-scale range value (2, 4 or 8 g). */
    axismg = ((int32_t)axismg * (int32_t)drvData->stepmg) / MMA_OUTDATA_FACTOR;

    /*if (axismg > 8191)
    {
        axismg = axismg - 16384;
    }*/

    return axismg;
}

/*******************************************************************************
 * Code - external functions
 ******************************************************************************/
#ifdef DEBUG
/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_PrintReg
 * Description   : This function prints a selected register to the virtual
 *                 serial output.
 *
 *END**************************************************************************/
void MMA_PrintReg(mma_drv_data_t *drvData, uint8_t regAddr, char *regName)
{
    status_t status = kStatus_Success;
    uint8_t regVal = 0U;

    status = MMA_ReadRegs(drvData, regAddr, &regVal, 1U);
    if (status != kStatus_Success)
    {
        PRINTF("\tAn error occurred in the MMA_PrintReg function (%d), reg. %s (0x%02x)\r\n",
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
 * Function Name : MMA_Init
 * Description   : Initializes the acceleromter driver based on user configuration.
 *
 *END**************************************************************************/
status_t MMA_Init(mma_drv_data_t *drvData, const mma_user_config_t *userConfig)
{
    status_t status = kStatus_Success;
    uint8_t  regVal = 0U;

    drvData->i2cBase = userConfig->i2cBase;
    drvData->i2cSlaveAddr = userConfig->i2cSlaveAddr;
    drvData->bitRes = userConfig->bitRes;
    /* Updated in the MMA_SetFullScaleRange function. */
    drvData->stepmg = 0U;

    /* SW reset of the device. */
    status |= MMA_ResetDevice(drvData);
    /* Wait after reset (boot time). */
    WaitUS(MMA_BOOT_TIME_US);

#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_CTRL_REG1, "CTRL_REG1");
#endif

    /* Note: the device is already in the standby mode (SW reset). */

    /* Set full scale value range. */
    status |= MMA_SetFullScaleRange(drvData, userConfig->fullScale);

#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_XYZ_DATA_CFG, "XYZ_DATA_CFG");
#endif

    /* Set bit resolution of measured acceleration data. */
    regVal = (userConfig->bitRes == mmaBitRes8) ? MMA_EN_F_READ : MMA_DIS_F_READ;
    /* Set output data rate (ODR). */
    regVal |= userConfig->odr;
    /* Enable accelerometer. */
    regVal |= MMA_SET_ACTIVE;

    status |= MMA_WriteReg(drvData, MMA_REG_CTRL_REG1, regVal);

#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_CTRL_REG1, "CTRL_REG1");
#endif

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_ConfTransDet
 * Description   : This function configures the accelerometer for the transient
 *                 detection function.
 *
 *END**************************************************************************/
status_t MMA_ConfTransDet(mma_drv_data_t *drvData, mma_trans_det_conf_t *conf)
{
    status_t status = kStatus_Success;
    uint8_t  regVal = 0U;

    /* Setup axis flag events. */
    regVal = conf->axisEventEn;

    /* Setup the event latch. */
    regVal |= (conf->latchEn) ? MMA_EN_ELE : MMA_DIS_ELE;
    status |= MMA_WriteReg(drvData, MMA_REG_TRANSIENT_CFG, regVal);
#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_TRANSIENT_CFG, "TRANSIENT_CFG");
#endif

    /* Set the threshold common for all axis. */
    regVal = MMA_GET_THS_REG(conf->thsmg);
    status |= MMA_UpdateReg(drvData, MMA_REG_TRANSIENT_THS, regVal,
            MMA_THS_VAL_MASK);
#ifdef DEBUG
    PRINTF("THS %u = 0x%02x (calculated)\r\n", conf->thsmg, regVal);
    MMA_PrintReg(drvData, MMA_REG_TRANSIENT_THS, "TRANSIENT_THS");
#endif

    /* Set the debounce counter. */
    status |= MMA_UpdateReg(drvData, MMA_REG_TRANSIENT_COUNT,
            conf->debounceVal, MMA_COUNT_VAL_MASK);
#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_TRANSIENT_COUNT, "TRANSIENT_COUNT");
#endif

    /* Enable high-pass output data. */
    status |= MMA_UpdateReg(drvData, MMA_REG_XYZ_DATA_CFG,
            MMA_EN_HPF_OUT, MMA_HPF_OUT_MASK);
#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_XYZ_DATA_CFG, "XYZ_DATA_CFG");
#endif

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_ConfFifo
 * Description   : This function configures FIFO (mode, watermark and triggers).
 *
 *END**************************************************************************/
status_t MMA_ConfFifo(mma_drv_data_t *drvData, mma_fifo_conf_t *fifoConf)
{
    uint8_t  regVal = 0U;
    status_t status = kStatus_Success;

    assert(fifoConf->watermarkCnt <= 32U);

    /* Set F_SETUP register. */
    regVal = (uint8_t)fifoConf->fifoMode | MMA_GET_F_WATERMARK(fifoConf->watermarkCnt);
    status = MMA_WriteReg(drvData, MMA_REG_F_SETUP, regVal);

    /* Set TRIG_CFG register. */
    status |= MMA_WriteReg(drvData, MMA_REG_TRIG_CFG, fifoConf->fifoTriggers);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_WriteReg
 * Description   : Writes a byte to the device using an 8bit memory address.
 *
 *END**************************************************************************/
status_t MMA_WriteReg(mma_drv_data_t *drvData, uint8_t regAddr, uint8_t value)
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
 * Function Name : MMA_ReadRegs
 * Description   : Reads bytes from the device using an 8bit memory address.
 *
 *END**************************************************************************/
status_t MMA_ReadRegs(mma_drv_data_t *drvData, uint8_t regAddr,
        uint8_t *valueArr, uint8_t valueArrSize)
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
 * Function Name : MMA_UpdateReg
 * Description   : Updates a byte of the device using an 8bit memory address.
 *
 *END**************************************************************************/
status_t MMA_UpdateReg(mma_drv_data_t *drvData, uint8_t regAddr,
        uint8_t value, uint8_t mask)
{
    uint8_t  regVal = 0U;   /* Value of a register. */
    status_t status = kStatus_Success;

    status = MMA_ReadRegs(drvData, regAddr, &regVal, 1U);

    regVal = MMA_UNSET_BIT_VALUE(regVal, mask);
    regVal = MMA_SET_BIT_VALUE(regVal, value & mask);

    status |= MMA_WriteReg(drvData, regAddr, regVal);
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_SetFullScaleRange
 * Description   : This function sets full-scale value range.
 *
 *END**************************************************************************/
status_t MMA_SetFullScaleRange(mma_drv_data_t *drvData, mma_full_scale_t scale)
{
    /* Update acceleration data step size, which is used for register to mg
     * conversion. */
    if (drvData->bitRes == mmaBitRes14)
    {
        switch (scale)
        {
            case mmaFullScale2g:
                drvData->stepmg = MMA_OUTDATA_14B_DIV_2G;
                break;

            case mmaFullScale4g:
                drvData->stepmg = MMA_OUTDATA_14B_DIV_4G;
                break;

            case mmaFullScale8g:
                drvData->stepmg = MMA_OUTDATA_14B_DIV_8G;
                break;
        }
    }
    else
    {   /* Not implemented for 8 bit resolution. */
        assert("Not implemented for 8 bit resolution");
    }

    return MMA_WriteReg(drvData, MMA_REG_XYZ_DATA_CFG, (uint8_t)scale);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_EnableInt
 * Description   : This function enables interrupt events.
 *
 *END**************************************************************************/
status_t MMA_EnableInt(mma_drv_data_t *drvData, mma_int_block_t blockSel,
        mma_int_pin_t pinSel)
{
    status_t status = kStatus_Success;
    uint8_t  regVal = 0U;
    /* Bit shift of a functional block common for CTRL_REG4/5 registers. */
    const uint8_t bitShift = (uint8_t)blockSel;
    /* Bit mask of a functional block common for CTRL_REG4/5 registers. */
    const uint8_t bitMask = 1U << bitShift;

    /* Set transient detection interrupt (routing event detection flags to the
     * system's interrupt controller). */
    regVal = 1U << bitShift;
    status |= MMA_UpdateReg(drvData, MMA_REG_CTRL_REG4, regVal, bitMask);
#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_CTRL_REG4, "CTRL_REG4");
#endif

    /* Route the transient interrupt to a hardware pin (system's interrupt
     * controller routes the enabled functional block interrupt to the INT1/2). */
    regVal = (uint8_t)pinSel << bitShift;
    status |= MMA_UpdateReg(drvData, MMA_REG_CTRL_REG5, regVal, bitMask);
#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_CTRL_REG5, "CTRL_REG5");
#endif

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_SetOpMode
 * Description   : This function sets the device operation mode (sleep or standby).
 *
 *END**************************************************************************/
status_t MMA_SetOpMode(mma_drv_data_t *drvData, bool active)
{
    status_t status = kStatus_Success;
    /* Value of a register. */
    uint8_t  regVal = active ? MMA_SET_ACTIVE : MMA_SET_STANDBY;

    status = MMA_UpdateReg(drvData, MMA_REG_CTRL_REG1, regVal, MMA_ACTIVE_MASK);

#ifdef DEBUG
    MMA_PrintReg(drvData, MMA_REG_CTRL_REG1, "CTRL_REG1");
#endif

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_ResetDevice
 * Description   : This function resets the device.
 *
 *END**************************************************************************/
status_t MMA_ResetDevice(mma_drv_data_t *drvData)
{
    return MMA_WriteReg(drvData, MMA_REG_CTRL_REG2, MMA_RST_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_MeasureRawAxis
 * Description   : This function performs a measurement on a channel and returns
 *                 the raw value.
 *
 *END**************************************************************************/
status_t MMA_MeasureRawAxis(mma_drv_data_t *drvData, mma_axis_t axis, uint16_t *val)
{
    status_t status = kStatus_Success;
    uint8_t rxBuffer[2];    /* Value from device is in big endian */
    uint8_t regAddr;        /* Register address. */

    /* Calculate address of the MSB byte (LSB byte occupies the next register). */
    regAddr = MMA_REG_OUT_X_MSB + ((uint8_t)axis << 1U);

    status = MMA_ReadRegs(drvData, regAddr, rxBuffer, sizeof(rxBuffer));

    /* Transform into little endiad value (Kinetis MCUs). */
    *val = MMA_GET_REG_OUT_RAW(rxBuffer[0], rxBuffer[1]);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_GetAxismg
 * Description   : This function gets value of an axis in mg.
 *
 *END**************************************************************************/
status_t MMA_GetAxismg(mma_drv_data_t *drvData, mma_axis_t axis, int16_t *val)
{
    status_t status = kStatus_Success;
    uint16_t valTemp = 0U;

    status = MMA_MeasureRawAxis(drvData, axis, &valTemp);
    *val = MMA_ConvAxisRawTomg(drvData, valTemp);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMA_GetAllAxismg
 * Description   : This function gets value of all axis in mg.
 *
 *END**************************************************************************/
status_t MMA_GetAllAxismg(mma_drv_data_t *drvData, mma_axis_data_t *val)
{
    status_t status = kStatus_Success;
    uint8_t  rxBuffer[6];    /* Value from device is in big endian */
    uint16_t axisRaw = 0U;

    status = MMA_ReadRegs(drvData, MMA_REG_OUT_X_MSB, rxBuffer,
            sizeof(rxBuffer));

    /* Convert raw values to mg. */
    axisRaw = MMA_GET_REG_OUT_RAW(rxBuffer[0], rxBuffer[1]);
    val->x = MMA_ConvAxisRawTomg(drvData, axisRaw);

    axisRaw = MMA_GET_REG_OUT_RAW(rxBuffer[2], rxBuffer[3]);
    val->y = MMA_ConvAxisRawTomg(drvData, axisRaw);


    axisRaw = MMA_GET_REG_OUT_RAW(rxBuffer[4], rxBuffer[5]);
    val->z = MMA_ConvAxisRawTomg(drvData, axisRaw);

    return status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
