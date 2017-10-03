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
 * @file sensors.c
 *
 * This file implements high level functions to access sensors.
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "sensors.h"
#include "app_config.h"
#include "math.h"

#ifdef DEBUG
#include "fsl_debug_console.h"
#endif

/*******************************************************************************
 * Prototypes
 *******************************************************************************/

/*!
 * @brief It calculates total acceleration without square root.
 *
 * @param axisData Acceleration measured on axis.
 *
 * @return Total acceleration.
 */
static uint32_t CalcTotalAccel(mma_axis_data_t *axisData);

/*!
 * @brief This function measures and returns total acceleration in (mg)^(1/2).
 *
 * @param mmaDrvData Accelerometer driver data.
 * @param maxTotalAccelmg Resulting value containing the max. total acceleration
 * read from the accelerometer FIFO buffer.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t GetMaxAccelMeas(mma_drv_data_t *mmaDrvData, uint32_t *maxTotalAccelmg);

/*******************************************************************************
 * Code
 *******************************************************************************/

static uint32_t CalcTotalAccel(mma_axis_data_t *axisData)
{
    uint32_t aTemp = 0;

    aTemp = (uint32_t)((int32_t)axisData->x * (int32_t)axisData->x);
    aTemp += (uint32_t)((int32_t)axisData->y * (int32_t)axisData->y);
    aTemp += (uint32_t)((int32_t)axisData->z * (int32_t)axisData->z);

    return aTemp;
}

static status_t GetMaxAccelMeas(mma_drv_data_t *mmaDrvData, uint32_t *maxTotalAccelmg)
{
    uint8_t samplesCnt = 0U;
    uint8_t i;
    status_t status = kStatus_Success;
    mma_axis_data_t axisData = { };
#ifdef DEBUG
    mma_axis_data_t axisDataTemp = { };
#endif
    uint32_t accelTempmg = 0U;

    (*maxTotalAccelmg) = 0U;

    /* Get number of samples in FIFO. */
    status |= MMA_ReadRegs(mmaDrvData, MMA_REG_F_STATUS, &samplesCnt, 1U);
    samplesCnt &= MMA_F_CNT_MASK;

    for (i = 0U; i < samplesCnt; i++)
    {   /* Flush FIFO and find the highest value of acceleration. */
        if ((status = MMA_GetAllAxismg(mmaDrvData, &axisData)) != kStatus_Success)
        {
            return status;
        }

        accelTempmg = CalcTotalAccel(&axisData);
        if ((*maxTotalAccelmg) < accelTempmg)
        {
            (*maxTotalAccelmg) = accelTempmg;
#ifdef DEBUG
            axisDataTemp.x = axisData.x;
            axisDataTemp.y = axisData.y;
            axisDataTemp.z = axisData.z;
#endif
        }
    }

#ifdef DEBUG
    PRINTF("\tx = %d mg\r\n", axisDataTemp.x);
    PRINTF("\ty = %d mg\r\n", axisDataTemp.y);
    PRINTF("\tz = %d mg\r\n", axisDataTemp.z);
#endif

    return kStatus_Success;
}

status_t MeasureMagTemp(mag_drv_data_t *magDrvData, int8_t *tempDegC)
{
    status_t status = kStatus_Success;
    uint16_t x, y, z;
    bool measCompl = false;

    /* Trigger measurement. */
    if ((status = MAG_TriggerMeas(magDrvData)) != kStatus_Success)
    {
        return status;
    }

    /* Wait for measurement results. */
    do
    {
        if ((status = MAG_IsMeasCompl(magDrvData, &measCompl)) != kStatus_Success)
        {
            return status;
        }
    }
    while (!measCompl);

    /* Read axis data to clear the data ready flag. */
    status |= MAG_GetAllAxisRaw(magDrvData, &x, &y, &z);

    /* Read temperature. */
    status |= MAG_GetTempDegC(magDrvData, tempDegC);

    return status;
}

status_t ClearAccelInt(mma_drv_data_t *mmaDrvData)
{
    status_t status = kStatus_Success;
    uint8_t regVal = 0U;

    /* Clear transient detection flags. */
    status = MMA_ReadRegs(mmaDrvData, MMA_REG_TRANSIENT_SRC, &regVal, 1U);

    /* Clear FIFO flags. */
    status |= MMA_ReadRegs(mmaDrvData, MMA_REG_F_STATUS, &regVal, 1U);

    return status;
}

status_t GetTotalAccel(mma_drv_data_t *mmaDrvData, uint16_t *aTotal)
{
    status_t status = kStatus_Success;
    uint32_t aTemp = 0;

    status = GetMaxAccelMeas(mmaDrvData, &aTemp);
    (*aTotal) = (uint16_t)sqrt((float)aTemp);

    return status;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
