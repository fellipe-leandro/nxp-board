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
 * @file wait.c
 *
 * This module implements MCU delay functions.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "wait.h"
#if (SDK_VERSION == SDK_2_0)
#include "fsl_clock.h"
#elif (SDK_VERSION == SDK_S32)
#include "fsl_device_registers.h"
#include "fsl_scg_hal.h"
#endif

#if defined(__thumb__) && !defined(__thumb2__) /* Thumb instruction set only */

/*******************************************************************************
 * Defines
 ******************************************************************************/
/*!
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * MOV - 1 cycle
 * SUB - 1 cycle
 * BNE - 1 cycle or 2 cycles if jump is realized
 *
 * Output list (empty) - which registers are output and how to map them to C code.
 * Input list (Cycles) - which registers are input and how to map them to C code.
 * Clobber list (r0, r1, cc) - which registers might have changed during
 * execution of asm code (compiler will have to reload them).
 *
 * @param Cycles Number of cycles to wait.
 */
#define WAIT_FOR_MUL4_CYCLES(cycles) \
    __asm( \
      "mov r0, %[cycles] \n\t" \
      "0: \n\t" \
        "sub r0, #4 \n\t" \
        "nop \n\t" \
      "bne 0b \n\t" \
       : \
       : [cycles] "r" (cycles) \
       : "r0", "r1", "cc" \
    ) \

#else /* Thumb2 or A32 instruction set */

/*!
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * @param cycles Number of cycles to wait.
 */
#define WAIT_FOR_MUL4_CYCLES(cycles) \
    __asm( \
      "movs r0, %[cycles] \n" \
      "0: \n" \
        "subs r0, r0, #4 \n" \
        "nop \n\t" \
      "bne 0b \n"  \
       : \
       : [cycles] "r" (cycles) \
       : "r0", "r1", "cc" \
    ) \

#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : WaitCycles
 * Description : This function waits for specified amount of cycles which is
**               given by 32bit value range. Assumption for this function is
**               that target architecture is using 32bit general purpose registers.
 *
 *END**************************************************************************/
inline void WaitCycles(uint32_t cycles)
{
    /* Advance to next multiple of 4. Value 0x04U ensures that the number
     * is not zero. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;

    WAIT_FOR_MUL4_CYCLES(cycles);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WaitSec
 * Description : This function waits for specified amount of seconds.
 *
 *END**************************************************************************/
void WaitSec(uint16_t sec)
{
    for (; sec > 0U; sec--) 
    {
        WaitMS(1000U);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WaitMS
 * Description : This function waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void WaitMS(uint16_t ms)
{
#if (SDK_VERSION == SDK_2_0)
    uint32_t cycles = (uint32_t)GET_CYCLES_FOR_MS(1U, CLOCK_GetCoreSysClkFreq());
#elif (SDK_VERSION == SDK_S32)
    uint32_t cycles = (uint32_t)GET_CYCLES_FOR_MS(1U, \
            SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_CORE));
#endif

    /* Advance to multiple of 4. */
    cycles = cycles & 0xFFFFFFFCU;

    for (; ms > 0U; ms--) 
    {
        WAIT_FOR_MUL4_CYCLES(cycles);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WaitUS
 * Description : This function waits for specified amount of microseconds.
 *
 *END**************************************************************************/
void WaitUS(uint16_t us)
{
#if (SDK_VERSION == SDK_2_0)
    uint32_t cycles = (uint32_t)GET_CYCLES_FOR_US(us, CLOCK_GetCoreSysClkFreq());
#elif (SDK_VERSION == SDK_S32)
    uint32_t cycles = (uint32_t)GET_CYCLES_FOR_US(us, \
            SCG_HAL_GetSystemClockFreq(SCG, SCG_SYSTEM_CLOCK_CORE));
#endif

    /* Advance to next multiple of 4. Value 0x04U ensures that the number
     * is not zero. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;
    WAIT_FOR_MUL4_CYCLES(cycles);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
 
