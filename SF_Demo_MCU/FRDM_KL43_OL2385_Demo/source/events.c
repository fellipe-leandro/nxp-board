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
 * @file events.c
 *
 * This file implements interrupt handlers of the application.
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "events.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_lptmr.h"
#include "setup.h"

/*******************************************************************************
 * Code
 *******************************************************************************/
void MMA_INT1_IRQ_HANDLER(void)
{
    uint32_t gpioIntFlags = 0U;

    gpioIntFlags = GPIO_GetPinsInterruptFlags(MMA_INT1_GPIO);

    /* Clear an external interrupt flag. */
    GPIO_ClearPinsInterruptFlags(MMA_INT1_GPIO, MMA_INT1_PIN_MASK);

    if ((gpioIntFlags & MMA_INT1_PIN_MASK) != 0U)
    {   /* Interrupt occurred on the INT1 pin. */
        g_accelInt = true;
    }
}

void SW1_2_3_IRQ_HANDLER(void)
{
    uint32_t gpioIntFlags = 0U;

    /* Note: SW1 and SW2 have the same GPIO base. */
    gpioIntFlags = GPIO_GetPinsInterruptFlags(SW1_GPIO);

    /* Clear an external interrupt flag. */
    GPIO_ClearPinsInterruptFlags(SW1_GPIO, gpioIntFlags);

    if ((gpioIntFlags & SW1_PIN_MASK) != 0U)
    {   /* SW1 interrupt. */
        g_sw1Int = true;
    }
    if ((gpioIntFlags & SW2_PIN_MASK) != 0U)
    {   /* SW2 interrupt. */
        g_sw2Int = true;
    }
    if ((gpioIntFlags & SW3_PIN_MASK) != 0U)
    {   /* SW3 interrupt. */
        g_sw3Int = true;
    }
}

void LPTMR_IRQ_HANDLER(void)
{
    LPTMR_ClearStatusFlags(LPTMR_DEV, kLPTMR_TimerCompareFlag);
    g_lptmrPeriodInt = true;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
