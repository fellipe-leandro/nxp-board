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
 * @file ls.c
 *
 * This module implements the driver for ambient light sensor
 * ALS-PT19-315C/L177/TR8.
 *
 * Current implementation includes functions for ADC measurement and conversion
 * to illuminance.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "ls.h"

/* Enables debugging messages printed to the virtual serial console. */
/* #define DEBUG */

#ifdef DEBUG
#include "fsl_debug_console.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Illuminance factor used in the Light current definition above. */
#define LS_ILLUM_FACTOR             1000U
/* Irradiance factor is Light current factor * Illuminance factor. */
#define LS_IRRADIANCE_FACTOR        (1000000U * LS_ILLUM_FACTOR)
/* Final factor is Light current factor / Illuminance factor / voltage factor (mv). */
#define LS_FACTOR                   ((LS_IRRADIANCE_FACTOR / LS_R_LOADING_OHM) / 1000U)

/*! Factor used in voltage ratio calculations. */
#define LS_VOLT_RATIO_FACTOR        1000U
/*! Voltage ratio (saturation voltage / reference voltage). */
#define LS_VOLT_RATIO               ((LS_VOLT_RATIO_FACTOR * LS_VOLT_SATUR_MV) / \
    LS_VOLT_REF_MV)
/*! Max. value of ADC result register. */
#define LS_ADC_MAX_CNT              ((1U << 16U) - 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief This function converts a raw ADC value to mV.
 *
 * @param adcRaw Raw measured ADC value.
 *
 * @return Resulting value in mV.
 */
static uint16_t LS_ConvAdcRawTomv(uint16_t adcRaw);

/*!
 * @brief This function converts a raw ADC value to lux.
 *
 * @param adcRaw Raw measured ADC value.
 *
 * @return Resulting value in lx.
 */
static uint16_t LS_ConvAdcRawToLx(uint16_t adcRaw);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : LS_ConvAdcRawTomv
 * Description   : This function converts a raw ADC value to mV.
 *
 *END**************************************************************************/
static uint16_t LS_ConvAdcRawTomv(uint16_t adcRaw)
{
    uint16_t mv = (uint16_t)(((uint32_t)adcRaw * LS_VOLT_REF_MV) / LS_ADC_MAX_CNT);
    return (uint16_t)(((uint32_t)mv * LS_VOLT_RATIO) / LS_VOLT_RATIO_FACTOR);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LS_ConvAdcRawToLx
 * Description   : This function converts a raw ADC value to lux.
 *
 *END**************************************************************************/
static uint16_t LS_ConvAdcRawToLx(uint16_t adcRaw)
{
    uint16_t mv = 0U;
    uint16_t lx = 0U;
    uint16_t adcRawInv = LS_ADC_MAX_CNT - adcRaw;

    mv = LS_ConvAdcRawTomv(adcRawInv);

    /* Calculate illuminance:
     * Ev = Ip / Ee = (U / R) / Ee = (U / Ee) / R
     * Note: Ee (irradiance) is equal to LS_CURRENT_1000LX_UA div Illuminance. */
    lx = (uint16_t)(((uint32_t)mv * LS_FACTOR) / LS_CURRENT_1000LX_UA);

#ifdef DEBUG
    PRINTF("Raw =   %u\r\n", adcRaw);
    PRINTF("Volt =  %u mV\r\n", mv);
    PRINTF("Illum = %u lx\r\n\r\n", lx);
#endif

    return lx;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LS_GetRawAdcMeas
 * Description   : This function starts an ADC measurement, waits for conversion
 *                 complete and returns resulting value.
 *
 *END**************************************************************************/
uint16_t LS_GetRawAdcMeas(ADC_Type *base, uint32_t chnlGrp,
        const adc16_channel_config_t *chnlConfig)
{
    uint32_t chnlStat = 0U;

    ADC16_SetChannelConfig(base, chnlGrp, chnlConfig);

    do
    {
        chnlStat = ADC16_GetChannelStatusFlags(base, chnlGrp);
    }
    while ((chnlStat & kADC16_ChannelConversionDoneFlag) == 0U);

    return ADC16_GetChannelConversionValue(base, chnlGrp);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LS_GetLx
 * Description   : This function performs an ADC measurement and converts value
 *                 to lx (illuminance).
 *
 *END**************************************************************************/
uint16_t LS_GetLx(ADC_Type *base, uint32_t chnlGrp,
        const adc16_channel_config_t *chnlConfig)
{
    /* Measured value is inverse. */
    uint16_t rawVal = LS_GetRawAdcMeas(base, chnlGrp, chnlConfig);

    return LS_ConvAdcRawToLx(rawVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LS_ConvLxToRaw
 * Description   : This function converts a value in lx (illumination) to raw
 *                 ADC value.
 *
 *END**************************************************************************/
uint16_t LS_ConvLxToRaw(uint16_t lxVal)
{
    uint16_t rawVal = 0U;
    uint16_t mv = 0U;

    /* Calculate mV. */
    mv = (lxVal * LS_CURRENT_1000LX_UA) / LS_FACTOR;

    /* Calculate raw value. */
    rawVal = (uint16_t)(((uint32_t)mv * LS_VOLT_RATIO_FACTOR) / LS_VOLT_RATIO);
    rawVal = (uint16_t)(((uint32_t)rawVal * LS_ADC_MAX_CNT) / LS_VOLT_REF_MV);

#ifdef DEBUG
    PRINTF("Illum = %u lx\r\n\r\n", lxVal);
    PRINTF("Volt =  %u mV\r\n", mv);
    PRINTF("Raw =   %u\r\n", rawVal);
#endif

    /* Register value is inverse. */
    return LS_ADC_MAX_CNT - rawVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LS_StopAdcContConv
 * Description   : This function stops continuous ADC conversions.
 *
 *END**************************************************************************/
void LS_StopAdcContConv(ADC_Type *base, uint32_t chnlGrp)
{
    adc16_channel_config_t chnlConfig = {};

    chnlConfig.channelNumber = 0U;
    chnlConfig.enableDifferentialConversion = false;
    chnlConfig.enableInterruptOnConversionCompleted = false;

    ADC16_SetChannelConfig(base, chnlGrp, &chnlConfig);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
