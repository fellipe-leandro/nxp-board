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
 * @file setup.c
 *
 * This file contains functions for initialization of peripherals, sensors
 * and the Sigfox device.
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "setup.h"
#include "app_config.h"
#include "board.h"

#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_lptmr.h"
#include "fsl_smc.h"

#if defined(PRINT_CONSOLE) || defined(DEBUG)
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#endif

/*******************************************************************************
 * Defines
 *******************************************************************************/
/*!
 * @brief Gets source clock for the SPI 1 peripheral.
 */
#define GET_SPI1_MODULE_CLK() \
    (CLOCK_GetFreq(SPI1_CLK_SRC))

/*******************************************************************************
 * Prototypes
 *******************************************************************************/
/*!
 * @brief This function is used for power mode switch. It enables virtual serial
 * port.
 *
 * @param originMode Original power mode (before the switching).
 * @param targetMode Target power mode.
 */
static void PowerPostSwitchHook(smc_power_state_t originMode, smc_power_state_t targetMode);

/*!
 * @brief This function is used for power mode switch. It disables virtual serial
 * port.
 *
 * @param originMode Original power mode (before the switching).
 * @param targetMode Target power mode.
 */
static void PowerPreSwitchHook(smc_power_state_t originMode, smc_power_state_t targetMode);

/*!
 * @brief This function initializes the I2C peripheral used by the magnetometer
 * and accelerometer.
 */
static void SetupI2CMmaMag(void);

/*!
 * @brief This functions initializes magnetometer sensor.
 *
 * @param magDrvData Magnetometer driver data. The user should put a pointer to
 * an empty driver data structure.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SetupMag(mag_drv_data_t *magDrvData);

/*!
 * @brief This functions initializes ADC peripheral used by light sensor.
 *
 * @param adcChnlConfig Configuration structure of an ADC channel.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SetupAdcLS(adc16_channel_config_t *adcChnlConfig);

/*!
 * @brief This function sets GPIO pins used by the application. It configures
 * an interrupt pin used for the accelerometer and red LED.
 */
static void SetupGpioPins(void);

/*!
 * @brief This function initializes the SIGFOX device.
 *
 * @param drvData SIGFOX driver data. The user should put a pointer to
 * an empty driver data structure.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SetupSigfoxDriver(sf_drv_data_t *drvData);

/*!
 * @brief This function initializes LPTMR peripheral.
 *
 * @param periodSec Timer period in seconds.
 */
static void SetupLPTMR(uint16_t periodSec);

/*!
 * @brief This function sets the accelerometer.
 *
 * @param mmaDrvData The accelerometer driver data.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SetupAccel(mma_drv_data_t *mmaDrvData);

/*******************************************************************************
 * Code - internal functions
 *******************************************************************************/

static void PowerPostSwitchHook(smc_power_state_t originMode, smc_power_state_t targetMode)
{
    smc_power_state_t powerState = SMC_GetPowerModeState(SMC);

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    if ((kSMC_PowerStateRun != targetMode) && (kSMC_PowerStateVlpr != targetMode))
    {
        /*
         * Debug console RX pin is set to disable for current leakage,
         * need to re-configure pinmux.
         * Debug console TX pin: Don't need to change.
         */
        PORT_SetPinMux(DEBUG_CONSOLE_RX_PORT, DEBUG_CONSOLE_RX_PIN, DEBUG_CONSOLE_RX_PINMUX);
    }

    /* Set debug console clock source. */
    if (kSMC_PowerStateVlpr == powerState)
    {
        BOARD_InitVlprDebugConsole();
    }
#else
    /* Suppress a warning. */
    (void)powerState;
#endif
}

static void PowerPreSwitchHook(smc_power_state_t originMode, smc_power_state_t targetMode)
{
#if defined(DEBUG) || defined(PRINT_CONSOLE)
    /* Wait for debug console output finished. */
    while (!(kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)))
    {
    }
    DbgConsole_Deinit();

    if ((kSMC_PowerStateRun != targetMode) && (kSMC_PowerStateVlpr != targetMode))
    {
        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set to pinmux to disable.
         * Debug console TX pin: Don't need to change.
         */
        PORT_SetPinMux(DEBUG_CONSOLE_RX_PORT, DEBUG_CONSOLE_RX_PIN, kPORT_PinDisabledOrAnalog);
    }
#endif
}

static void SetupI2CMmaMag(void)
{
    uint32_t sourceClock = 0;
    i2c_master_config_t masterConfig;

    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = MAG_MMA_I2C_BAUDRATE;
    sourceClock = CLOCK_GetFreq(I2C0_CLK_SRC);

    I2C_MasterInit(MAG_MMA_I2C_BASE, &masterConfig, sourceClock);
}

static status_t SetupMag(mag_drv_data_t *magDrvData)
{
    mag_user_config_t magUserData;

    /* General magnetometer configuration. */
    magUserData.i2cBase = MAG_MMA_I2C_BASE;
    magUserData.i2cSlaveAddr = MAG_I2C_SLAVE_ADDR;
    magUserData.autoMrstEn = true;
    magUserData.modeActive = false;
    magUserData.tempOffsetDegC = 20;

    return MAG_Init(magDrvData, &magUserData);
}

static status_t SetupAdcLS(adc16_channel_config_t *adcChnlConfig)
{
    adc16_config_t adcConfig = {};
    status_t status = kStatus_Success;

    /* General configuration of the ADC. */
    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_ResolutionSE16Bit;
    /* Reference voltage is MCU internal voltage. */
    adcConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;

    /* Initialize the ADC driver. */
    ADC16_Init(LS_ADC_BASE, &adcConfig);
    /* Disable HW trigger (SW trigger only). */
    ADC16_EnableHardwareTrigger(LS_ADC_BASE, false);

    /* Calibration. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    status |= ADC16_DoAutoCalibration(LS_ADC_BASE);
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Configuration of an ADC channel (used in ADC function calls). */
    adcChnlConfig->channelNumber = LS_ADC_CHANNEL;
    adcChnlConfig->enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnlConfig->enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

    return status;
}

static void SetupGpioPins(void)
{
    gpio_pin_config_t gpioConfig = { };

    /* Setup red LED. */
    gpioConfig.outputLogic = 0U;    /* Default on. */
    gpioConfig.pinDirection = kGPIO_DigitalOutput;
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &gpioConfig);

    /* Setup SW1, SW2 and SW3 push buttons. */
    EnableIRQ(SW1_2_3_IRQn);
    gpioConfig.outputLogic = 0U;
    gpioConfig.pinDirection = kGPIO_DigitalInput;
    GPIO_PinInit(SW1_GPIO, SW1_PIN, &gpioConfig);
    GPIO_PinInit(SW2_GPIO, SW2_PIN, &gpioConfig);
    GPIO_PinInit(SW3_GPIO, SW3_PIN, &gpioConfig);

    /* Setup INT1 interrupt pin. */
    EnableIRQ(MMA_INT1_IRQn);
    gpioConfig.outputLogic = 0U;
    gpioConfig.pinDirection = kGPIO_DigitalInput;
    GPIO_PinInit(MMA_INT1_GPIO, MMA_INT1_PIN, &gpioConfig);

    /* Set GPIO output value and direction of SPI pins. The application
     * changes pin mux. in runtime from SPI to GPIO. */
    gpioConfig.outputLogic = 0U;
    gpioConfig.pinDirection = kGPIO_DigitalOutput;

    GPIO_PinInit(GPIOD, 5U, &gpioConfig);               /* CLK */
    GPIO_PinInit(GPIOD, 6U, &gpioConfig);               /* MOSI */
    GPIO_PinInit(GPIOD, 7U, &gpioConfig);               /* MISO */
}

static status_t SetupAccel(mma_drv_data_t *mmaDrvData)
{
    mma_trans_det_conf_t transConfig;
    mma_fifo_conf_t      fifoConf;
    mma_user_config_t    mmaUserData;
    status_t             status = kStatus_Success;

    /* General accelerometer configuration. */
    mmaUserData.i2cBase = MAG_MMA_I2C_BASE;
    mmaUserData.i2cSlaveAddr = MMA_I2C_SLAVE_ADDR;
    mmaUserData.bitRes = mmaBitRes14;
    mmaUserData.fullScale = mmaFullScale4g;
    mmaUserData.odr = mmaOdr6Hz;

    /* Transient detection configuration. */
    transConfig.axisEventEn = (uint8_t)mmaTransEventEnZ |
            (uint8_t)mmaTransEventEnY | (uint8_t)mmaTransEventEnX;
    transConfig.debounceVal = 1U;
    transConfig.latchEn = true;
    transConfig.thsmg = MMA_THRESHOLD_MG;

    /* FIFO configuration. */
    fifoConf.fifoMode = mmaFifoTrigger;
    fifoConf.fifoTriggers = mmaFifoTrigTrans;
    fifoConf.watermarkCnt = 0U;

    status |= MMA_Init(mmaDrvData, &mmaUserData);

    /* Put the device into the standby mode. */
    status |= MMA_SetOpMode(mmaDrvData, false);

    /* Configure the device for the transient detection mode. */
    status |= MMA_ConfTransDet(mmaDrvData, &transConfig);

    /* Enable FIFO trigger mode, no watermark. */
    status |= MMA_ConfFifo(mmaDrvData, &fifoConf);

    /* Enable INT1 interrupt. */
    status |= MMA_EnableInt(mmaDrvData, mmaIntBlockTrans, mmaIntPinInt1);

    /* Put the device into the active mode. */
    status |= MMA_SetOpMode(mmaDrvData, true);

    return status;
}

static void SetupLPTMR(uint16_t periodSec)
{
    lptmr_config_t lptmrConfig;         /* LPTMR configuration. */

    /* Preconditions. */
    assert(periodSec < (UINT16_MAX / GET_LPTMR_CLK()));

    LPTMR_GetDefaultConfig(&lptmrConfig);
    lptmrConfig.value = kLPTMR_Prescale_Glitch_6;
    lptmrConfig.bypassPrescaler = false;

    /* Initialize the LPTMR. */
    LPTMR_Init(LPTMR_DEV, &lptmrConfig);

    LPTMR_StopTimer(LPTMR_DEV);

    /* Set timer period */
    LPTMR_SetTimerPeriod(LPTMR_DEV, GET_LPTMR_PERIOD_TICKS(periodSec));

    LPTMR_ClearStatusFlags(LPTMR_DEV, kLPTMR_TimerCompareFlag);

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR_DEV, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(LPTMR_IRQn);
}

static status_t SetupSigfoxDriver(sf_drv_data_t *drvData)
{
    sf_user_config_t userConfig;

    SF_GetDefaultConfig(&userConfig);

    /* GPIOs initialization.
     * Note: GPIO settings are place in pin_mux.h file. */
    /* ACK pin. */
    drvData->gpioConfig.ackPin.gpioInstance = SF_ACK_INST;
    drvData->gpioConfig.ackPin.gpioPinNumber = SF_ACK_PIN;

    /* CS pin. */
    drvData->gpioConfig.csPin.gpioInstance = SF_CS_INST;
    drvData->gpioConfig.csPin.gpioPinNumber = SF_CS_PIN;

    SF_SetupGPIOs(&(drvData->gpioConfig));

    /* SPI initialization. */
    drvData->spiConfig.baudRate = 100000U;
    drvData->spiConfig.sourceClkHz = GET_SPI1_MODULE_CLK();
    drvData->spiConfig.spiInstance = SF_SPI_INST;

    SF_SetupSPI(&(drvData->spiConfig), NULL);

    /* Sigfox driver initialization.
     * Note: drvData->gpioConfig and drvData->spiConfig structures are used
     * by SF_SetupGPIOs, SF_SetupSPI and SF_Init. */
    return SF_Init(drvData, &userConfig);
}

/*******************************************************************************
 * Code - public functions
 *******************************************************************************/

status_t SetupDevices(mag_drv_data_t *magDrvPtr, mma_drv_data_t *mmaDrvPtr,
        adc16_channel_config_t *lsAdcChnlCfgPtr, sf_drv_data_t *sfDrvPtr)
{
    status_t status = kStatus_Success;

    /* Setup I2C used by accelerometer and magnetomenter. */
    SetupI2CMmaMag();

    /* Setup GPIO pins used in the application. */
    SetupGpioPins();

    /* Setup magnetometer. */
    if ((status = SetupMag(magDrvPtr)) != kStatus_Success)
    {
        return status;
    }

    /* Setup ADC used by light sensor. */
    if ((status = SetupAdcLS(lsAdcChnlCfgPtr)) != kStatus_Success)
    {
        return status;
    }

    /* Setup Sigfox driver, SPI and GPIOs used be the driver. */
    if ((status = SetupSigfoxDriver(sfDrvPtr)) != kStatus_Success)
    {
            return status;
    }

    /* Setup accelerometer. */
    if ((status = SetupAccel(mmaDrvPtr)) != kStatus_Success)
    {
        return status;
    }

    /* LPTMR is used to send data periodically. */
    SetupLPTMR(DEBOUNCE_TIME_S);

    return status;
}

void SetLPTMRPeriodAndReset(uint32_t periodSec)
{
    assert(((periodSec * GET_LPTMR_CLK()) / LPTMR_PRESC_VAL) < UINT16_MAX);

    LPTMR_StopTimer(LPTMR_DEV);
    LPTMR_SetTimerPeriod(LPTMR_DEV, GET_LPTMR_PERIOD_TICKS(periodSec));
    LPTMR_StartTimer(LPTMR_DEV);
}

void SetVlpsMode(void)
{
    PowerPreSwitchHook(kSMC_PowerStateVlpr, kSMC_PowerStateVlps);
    /* Note that SMC_SetPowerModeVlps returns kStatus_SMC_StopAbort,
     * which indicates that the stop mode was aborted by an interrupt
     * (normal condition, no error). */
    (void)SMC_SetPowerModeVlps(SMC);
    PowerPostSwitchHook(kSMC_PowerStateVlpr, kSMC_PowerStateVlps);
}

status_t SetVlprMode(void)
{
    status_t status = kStatus_Success;
    uint32_t timeout = CLOCK_GetFreq(kCLOCK_CoreSysClk);

    /* Set VLPR mode. It is expected that clocks are set appropriately. */
    if ((status = SMC_SetPowerModeVlpr(SMC)) != kStatus_Success)
    {
        return status;
    }

    /* Check transition to VLPR mode. */
    while ((kSMC_PowerStateVlpr != SMC_GetPowerModeState(SMC)) && (timeout > 0U))
    {
        timeout--;
    }
    return (timeout == 0U) ? kStatus_Timeout : kStatus_Success;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
