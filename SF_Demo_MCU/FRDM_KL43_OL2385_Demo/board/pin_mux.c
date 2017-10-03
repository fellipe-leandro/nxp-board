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

#include "pin_mux.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "../source/setup.h"
#include "../source/app_config.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Function Name : BOARD_InitPins */
void BOARD_InitPins(void)
{
    /* Declare and initialise for pull up configuration */
    port_pin_config_t pinConfigPullUp = {0};
    port_pin_config_t pinConfigPullDown = {0};

    pinConfigPullUp.pullSelect = kPORT_PullUp;
    pinConfigPullDown.pullSelect = kPORT_PullDown;

    /* Ungate the port clocks. */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    /* Initialize LPUART0 pins below */
    /* Affects PORTA_PCR1 register */
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);
    /* Affects PORTA_PCR2 register */
    PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);
#endif

    /* Release I2C bus */
    BOARD_I2C_ReleaseBus();

    /* I2C0 pull up resistor setting */
    PORT_SetPinConfig(PORTE, 24U, &pinConfigPullUp);
    PORT_SetPinConfig(PORTE, 25U, &pinConfigPullUp);
    /* I2C0 PIN_MUX Configuration */
    PORT_SetPinMux(PORTE, 24U, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 25U, kPORT_MuxAlt5);

    /* INT1 pin used for accelerometer. */
    PORT_SetPinConfig(MMA_INT1_PORT, MMA_INT1_PIN, &pinConfigPullUp);
    PORT_SetPinMux(MMA_INT1_PORT, MMA_INT1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinInterruptConfig(MMA_INT1_PORT, MMA_INT1_PIN, kPORT_InterruptFallingEdge);

    /* Configure red LED. */
    PORT_SetPinMux(BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, kPORT_MuxAsGpio);

    /* Sigfox SPI configuration. */
    PORT_SetPinConfig(PORTD, 7U, &pinConfigPullDown);
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAlt2);               /* MISO */
    PORT_SetPinConfig(PORTD, 6U, &pinConfigPullUp);
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAlt2);               /* MOSI */
    PORT_SetPinConfig(PORTD, 5U, &pinConfigPullDown);
    PORT_SetPinMux(PORTD, 5U, kPORT_MuxAlt2);               /* CLK */
    PORT_SetPinConfig(SF_CS_PORT, SF_CS_PIN, &pinConfigPullUp);
    PORT_SetPinMux(SF_CS_PORT, SF_CS_PIN, kPORT_MuxAsGpio); /* CS pin. */

    /* Sigfox ACK pin. */
    PORT_SetPinConfig(SF_ACK_PORT, SF_ACK_PIN, &pinConfigPullUp);
    PORT_SetPinMux(SF_ACK_PORT, SF_ACK_PIN, kPORT_MuxAsGpio);

    /* Pushbutton SW1. */
    PORT_SetPinConfig(SW1_PORT, SW1_PIN, &pinConfigPullUp);
    PORT_SetPinMux(SW1_PORT, SW1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinInterruptConfig(SW1_PORT, SW1_PIN, kPORT_InterruptFallingEdge);

    /* Pushbutton SW2. */
    PORT_SetPinConfig(SW2_PORT, SW2_PIN, &pinConfigPullUp);
    PORT_SetPinMux(SW2_PORT, SW2_PIN, kPORT_MuxAsGpio);
    PORT_SetPinInterruptConfig(SW2_PORT, SW2_PIN, kPORT_InterruptFallingEdge);

    /* Pushbutton SW3. */
    PORT_SetPinConfig(SW3_PORT, SW3_PIN, &pinConfigPullUp);
    PORT_SetPinMux(SW3_PORT, SW3_PIN, kPORT_MuxAsGpio);
    PORT_SetPinInterruptConfig(SW3_PORT, SW3_PIN, kPORT_InterruptFallingEdge);
}

void BOARD_SwitchAppPortMux(bool muxToGpio)
{
    port_mux_t portMux = kPORT_MuxAsGpio;

    /* Set port mux of SPI CLK, MOSI and MISO. */
    portMux = (muxToGpio) ? kPORT_MuxAsGpio : kPORT_MuxAlt2;

    PORT_SetPinMux(PORTD, 5U, portMux);               /* CLK */
    PORT_SetPinMux(PORTD, 6U, portMux);               /* MOSI */
    PORT_SetPinMux(PORTD, 7U, portMux);               /* MISO */
}

void BOARD_I2C_ReleaseBus(void)
{
    port_pin_config_t i2c_pin_config = {0};
    gpio_pin_config_t pin_config;
    uint8_t i = 0;
    uint8_t j = 0;

    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;

    PORT_SetPinConfig(PORTE, 24U, &i2c_pin_config);
    PORT_SetPinConfig(PORTE, 25U, &i2c_pin_config);

    GPIO_PinInit(GPIOE, 24U, &pin_config);
    GPIO_PinInit(GPIOE, 25U, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(GPIOE, 25U, 0U);

    /* Send 8 pulses on SCL and keep SDA low */
    for (i = 0; i < 8; i++)
    {
        GPIO_WritePinOutput(GPIOE, 24U, 0U);
        for (j = 0; j < 255; j++)
        {
            __asm("nop");
        }
        GPIO_WritePinOutput(GPIOE, 24U, 1U);
        for (j = 0; j < 255; j++)
        {
            __asm("nop");
        }
    }

    /* Drive SDA high to simulate a nak */
    GPIO_WritePinOutput(GPIOE, 24U, 0U);
    GPIO_WritePinOutput(GPIOE, 25U, 1U);
    for (j = 0; j < 255; j++)
    {
        __asm("nop");
    }
    GPIO_WritePinOutput(GPIOE, 24U, 1U);
    for (j = 0; j < 255; j++)
    {
        __asm("nop");
    }
    GPIO_WritePinOutput(GPIOE, 24U, 0U);
    for (j = 0; j < 255; j++)
    {
        __asm("nop");
    }

    /* Send stop */
    GPIO_WritePinOutput(GPIOE, 25U, 0U);
    for (j = 0; j < 255; j++)
    {
        __asm("nop");
    }
    GPIO_WritePinOutput(GPIOE, 24U, 1U);
    for (j = 0; j < 255; j++)
    {
        __asm("nop");
    }
}
