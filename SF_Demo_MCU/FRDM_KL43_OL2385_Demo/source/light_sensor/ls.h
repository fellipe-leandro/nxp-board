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
 * @file ls.h
 *
 * This module implements the driver for ambient light sensor
 * ALS-PT19-315C/L177/TR8.
 *
 * Current implementation includes functions for ADC measurement and conversion
 * to illuminance.
 */

#ifndef SOURCE_LS_H_
#define SOURCE_LS_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_common.h"
#include "fsl_adc16.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @addtogroup ls_functions_group
 * @{
 */

/* Value of light sensor loading resistor in Ohms. */
#define LS_R_LOADING_OHM            10000U
/* Light current in uA when the illuminance is 1000 lx (typ. 200 uA). */
#define LS_CURRENT_1000LX_UA        200U
/*! Voltage reference voltage in mV. */
#define LS_VOLT_REF_MV              3300U
/*! Saturation output voltage of the light sensor in mV. */
#define LS_VOLT_SATUR_MV            4600U
/*! ADC resolution (bits). */
#define LS_ADC_RES_BITS             16U
/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup mma_functions_group
 * @{
 */
/*!
 * @brief This function starts an ADC measurement, waits for conversion complete
 * and returns resulting value.
 *
 * It can be used to restart continuous conversions (when stopped).
 *
 * @param base       ADC base pointer.
 * @param chnlGrp    ADC channel group number.
 * @param chnlConfig Configuration of an ADC channel.
 *
 * @return Raw measured value.
 */
uint16_t LS_GetRawAdcMeas(ADC_Type *base, uint32_t chnlGrp,
        const adc16_channel_config_t *chnlConfig);

/*!
 * @param This function performs an ADC measurement and converts value to lx
 * (illuminance).
 *
 * @param base       ADC base pointer.
 * @param chnlGrp    ADC channel group number.
 * @param chnlConfig Configuration of an ADC channel.
 *
 * @return Resulting value in lx
 */
uint16_t LS_GetLx(ADC_Type *base, uint32_t chnlGrp,
        const adc16_channel_config_t *chnlConfig);

/*!
 * @brief This function converts a value in lx (illumination) to raw ADC value.
 *
 * It can be used to calculate a threshold value for ADC compare function.
 *
 * @param lxVal Value in lx.
 *
 * @return Resulting raw value that can be placed into a ADC compare value
 * register.
 */
uint16_t LS_ConvLxToRaw(uint16_t lxVal);

/*!
 * @brief This function stops continuous ADC conversions.
 *
 * @param base       ADC base pointer.
 * @param chnlGrp    ADC channel group number.
 *
 * @return Raw measured value.
 */
void LS_StopAdcContConv(ADC_Type *base, uint32_t chnlGrp);
/*! @} */

#endif /* SOURCE_LS_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
