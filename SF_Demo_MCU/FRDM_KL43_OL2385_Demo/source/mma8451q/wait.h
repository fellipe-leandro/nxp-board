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
 * @file wait.h
 *
 * This module implements MCU delay functions.
 */

#ifndef SOURCE_WAIT_H_
#define SOURCE_WAIT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "stdint.h"
#include "../aml/common_aml.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
/*!
 * @addtogroup wait_group
 * @{
 */

/*!
 * @brief Gets needed cycles for specified number of milliseconds,
 * calculation is based on core clock frequency.
 *
 * @param ms number of milliseconds
 * @param freq frequency of the MCU
 */
#define GET_CYCLES_FOR_MS(ms, freq) (((freq) / 1000U) * (ms))

/*!
 * @brief Gets needed cycles for specified number of microseconds,
 * calculation is based on core clock frequency.
 *
 * @param us number of milliseconds
 * @param freq frequency of the MCU
 */
#define GET_CYCLES_FOR_US(us, freq) (((freq) / 1000U) * (us) / 1000U)

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief This function waits for specified amount of cycles which is
 *        given by 32bit value range. Assumption for this function is
 *        that target architecture is using 32bit general purpose registers.
 *
 * @param cycles Number of cycles to wait.
 *
 * @return None.
 */
void WaitCycles(uint32_t cycles);

/*!
 * @brief This function waits for specified amount of seconds.
 *
 * @param sec Number of seconds to wait.
 *
 * @return None.
 */
void WaitSec(uint16_t sec);

/*!
 * @brief This function waits for specified amount of milliseconds.
 *
 * @param ms Number of milliseconds to wait.
 *
 * @return None.
 */
void WaitMS(uint16_t ms);

/*!
 * @brief This function waits for specified amount of microseconds.
 *
 * @param us Number of microseconds to wait.
 *
 * @return None.
 */
void WaitUS(uint16_t us);
/*! @} */

#endif /* SOURCE_WAIT_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
