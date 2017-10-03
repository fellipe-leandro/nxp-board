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
 * @file main.c
 *
 * This file contains application main code.
 *
 * This project demonstrates usage of the SIGFOX device. It sends data from
 * sensors to SIGFOX network.
 *
 * Data are sent in the following cases:
 * 1) The user shakes the MCU board which raises an interrupt in an accelerometer.
 * The accelerometer is configured for transient detection with a defined
 * threshold (see MMA_THRESHOLD_MG in app_config.h). Then the total acceleration
 * in mg is calculated and sent to SIGFOX.
 *
 * 2) When the user presses a pushbutton SW1 placed on the OL2385 board, it will
 * send the temperature in deg. C. A magnetometer's temperature sensor is used for
 * this purpose.
 *
 * 3) A SW2 pushbutton is associated with the illuminance measurement. The device will
 * send the illuminance in lx to SIGFOX. The application utilizes an ambient light
 * sensor.
 *
 * 4) LTPMR timer expired. The device sends the illuminance and temperature.
 * The timer is enabled by default and timeout is set
 * to 30 min. You can change these settings using SEND_PERIODICALLY_EN and
 * SEND_PERIOD_S (see app_config.h).
 *
 * Note that all of the mentioned sensors are placed on the KL43 MCU board.
 * There are not needed any changes or jumper settings on the board.
 *
 * MCU operates in VLPR mode and switches to VLPS when it is waiting for
 * the periodic sending timer event. The user can affect this with use of
 * the MCU_SLEEP_MODE_EN macro placed in app_config.h file.
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

/* MCU power mode settings. */
#include "fsl_smc.h"

/* Devices includes. */
#include "fsl_i2c.h"
#include "mag3110/mag.h"

/* Application includes. */
#include "sensors.h"
#include "setup.h"
#include "events.h"

/* Serial console. */
#if defined(DEBUG) || defined(PRINT_CONSOLE)
#include "fsl_debug_console.h"
#include "fsl_port.h"
#endif

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/*!
 * @name SIGFOX frame definition
 * Following macros defines a format of the SIGFOX frame used in this application.
 * @{
 */
/*! SIGFOX frame length in bytes. */
#define SF_PAYLOAD_LEN_B            5U
/*! Index of the application command field in the SIGFOX frame payload. */
#define SF_PAYLOAD_APP_CMD_ID       0
/*! Index of the value field in the SIGFOX frame payload. It's the LSB part
 * of 16 bit value. */
#define SF_PAYLOAD_VAL_LOW_ID       4
/*! @} */

/*!
 * @name Application commands
 * The application commands are used to identify a type of the SIGFOX frame
 * in a web application.
 * @{
 */
/*! Temperature command. Data represent temperature in °C. */
#define APP_CMD_TEMP                1U
/*! Accelerometer command. Data represent acceleration in mg. */
#define APP_CMD_ACCEL               2U
/*! Illumance command. Data represent illuminance in lx. */
#define APP_CMD_ILLUM               3U
/*! @} */

/*! Value of the first command in a command list (ascending order). */
#define APP_USER_CMD_FIRST          '0'
/*! Value of the last command in a command list (ascending order). */
#define APP_USER_CMD_LAST           '5'

/*!
 * @brief This macro is true when an interrupt occurred.
 */
#define APP_EVENT_OCCURRED() \
    (g_accelInt | g_lptmrPeriodInt | g_sw1Int | g_sw2Int | g_sw3Int)

/*!
 * @brief This macro is true when an interrupt causing transmission to
 * SIGFOX network occurred.
 */
#define APP_EVENT_SEND_OCCURRED() \
    (g_accelInt | g_lptmrPeriodInt | g_sw1Int | g_sw2Int)

/*!
 * @brief This macro is true when an interrupt occurred. It does not include
 * the LPTMR event.
 */
#define APP_EVENT_NONPERIOD_OCCURRED() \
    (g_accelInt | g_sw1Int | g_sw2Int | g_sw3Int)

/*! Application states. */
typedef enum
{
    appStateInitDebounce = 0U,  /*!< Start the debounce timer. When the accelerometer
                                     detects the move measured data are sent to SIGFOX
                                     network and the application waits for this time
                                     and does not react to any accelerometer event. */
    appStateWaitDebounce,       /*!< In this state, the application waits for expiration
                                     of the debounce timer. It does not respond to any
                                     event (accelerometer detection, periodic sending). */
    appStateInitPeriodic,       /*!< Start a timer used to send data to SIGFOX
                                     periodically. It places MCU into the VLPS mode. */
    appStateReady,              /*!< The application is ready to send data to SIGFOX
                                     network when an event occurs (accelerometer detection,
                                     periodic sending). */
} app_state_t;

/*! User commands. */
typedef enum
{
    userCmdPrintDevIdPac = 0U,  /*!< It prints the device ID and PAC. */
    userCmdSetETSI,             /*!< It sets European standard ETSI. */
    userCmdSetFCC_USA,          /*!< It sets USA standard FCC. */
    userCmdSetARIB,             /*!< It sets Japanese\Korean standard ARIB. */
    userCmdSetFCC_SouthAmerica, /*!< It sets South American standard FCC. */
    userCmdAppStart,            /*!< It starts the application. */
    userCmdNotValid             /*!< Invalid command number. */
} user_cmd_t;

typedef struct
{

    bool sfOpModeNormal;        /*!< Current operating mode of SIGFOX device
                                     (true - normal, false - sleep). */
    sf_net_standard_t sfNetStandard; /*!< Current SIGFOX networking standard set
                                     by the user. */
    app_state_t appState;       /*!< Current state of application state machine. */
} app_data_t;

/*******************************************************************************
 * Prototypes
 *******************************************************************************/

#if defined(DEBUG) || defined(PRINT_CONSOLE)
/*!
 * @brief This function prints an interrupt source to the virtual serial console.
 */
static void PrintEvent(void);
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
/*!
 * @brief This function prints a command list. See the user_cmd_t enumeration
 * for possible commands.
 */
static void PrintCmdList(void);
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
/*!
 * @brief This function reads the Device ID and PAC and prints them to the
 * virtual serial console.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t PrintDevIdPac(void);
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
/*!
 * @brief This function reads input from virtual serial console returns a
 * command number written by the user.
 *
 * @return A command number.
 */
static user_cmd_t GetUserCmd(void);
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
/*!
 * @brief This function handles user commands.
 * The user can use prepared commands (see user_cmd_t enumeration) to
 * control the device at the beginning of the application.
 *
 * @param appDataPtr Current application state.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessUserCmds(app_data_t *appDataPtr);
#endif

/*!
 * @brief This function clears all interrupt variables, which indicate
 * interrupt events. It accesses global variables.
 */
static void ClearAllIntFlags(void);

/*!
 * @brief This function starts blinking the red LED in an infinite loop.
 * It signalizes that an error occurred.
 */
static void StarBlinkingLED(void);

/*!
 * @brief This function measures and sends temperature to SIGFOX in deg. C.
 *
 * @param payload Pointer to the SIGFOX frame payload. Its value is updated by
 * this function.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SendTemp(sf_msg_payload_t *payload);

/*!
 * @brief This function measures and sends illuminance to SIGFOX in lx.
 *
 * @param payload Pointer to the SIGFOX frame payload. Its value is updated by
 * this function.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SendIllum(sf_msg_payload_t *payload);

/*!
 * @brief This function measures and sends acceleration value to SIGFOX in mg.
 *
 * @param payload Pointer to the SIGFOX frame payload. Its value is updated by
 * this function.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SendAccel(sf_msg_payload_t *payload);

#if (SEND_TO_SIGFOX_EN == true)
/*!
 * @brief This function creates the frame payload.
 *
 * @param appCmd Application command. Possible values are defined by the following
 * macros: APP_CMD_TEMP, APP_CMD_ACCEL, APP_CMD_ILLUM.
 * @param value 32 bit value containing a value which interpretation depends on
 * the selected application command.
 * @param framePld Resulting frame payload.
 */
static void PackData(uint8_t appCmd, uint32_t value, sf_msg_payload_t *framePld);
#endif

/*!
 * @brief This function sends data to SIGFOX network. It uses global variables
 * to check if an interrupt occurred and sends data associated with
 * the interrupt.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SendData(void);

/*!
 * @brief It wakes up the device and sets a networking standard defined by the user.
 * Note that SIGFOX device settings are reset in the sleep mode.
 *
 * @param appDataPtr Pointer to data used by the application (current state).
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SetSigfoxWakeupMode(app_data_t *appDataPtr);

/*!
 * @brief It puts the SIGFOX device into the sleep mode.
 * Note that SIGFOX device settings are reset in the sleep mode.
 *
 * @param appDataPtr Pointer to data used by the application (current state).
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t SetSigfoxSleepMode(app_data_t *appDataPtr);

/*!
 * @brief This function runs the application state machine. It contains an
 * infinite loop.
 *
 * @param appDataPtr Pointer to data used by the application (current state).
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t StartApp(app_data_t *appDataPtr);

/*******************************************************************************
 * Prototypes - application state machine
 *******************************************************************************/

/*!
 * @brief This function processes the Init debounce state. It starts the LPTMR timer.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessInitDebounce(void);

/*!
 * @brief This function processes the Wait debounce state. It clears all global
 * interrupt variables, clears an accelerometer event via I2C and turns off
 * the red LED.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessWaitDebonce(void);

/*!
 * @brief This function processes the Init period state. It resets the LPTMR
 * timer.
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessInitPeriodic(void);

/*!
 * @brief This function is a helper function used in processing of the Ready state.
 * It does the following:
 *   Turns on the red LED to signalize that the device is ready to send data.
 *   Stops LPTMR timer.
 *   Clears accelerometer event (if occurred).
 *   Sends data to SIGFOX and turns off the red LED.
 *
 * @param appDataPtr Pointer to data used by the application (current state).
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessReadyEvent(app_data_t *appDataPtr);

/*!
 * @brief This function processes the Ready state. It updates the application
 * state variable.
 *
 * @param appDataPtr Pointer to data used by the application (current state).
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessReady(app_data_t *appDataPtr);

/*!
 * @brief This function implements the application state machine.
 *
 * @param appDataPtr Pointer to data used by the application (current state).
 *
 * @return Status of the function (kstatus_Success on success).
 */
static status_t ProcessAppState(app_data_t *appDataPtr);

/*******************************************************************************
 * Variables
 *******************************************************************************/

/*! Interrupt flag indicating an accelerometer event. */
volatile bool g_accelInt = false;
/*! Interrupt flag indicating a LPTMR event. */
volatile bool g_lptmrPeriodInt = false;
/*! Interrupt flag indicating a SW1 push button event. */
volatile bool g_sw1Int = false;
/*! Interrupt flag indicating a SW2 push button event. */
volatile bool g_sw2Int = false;
/*! Interrupt flag indicating a SW3 push button event. */
volatile bool g_sw3Int = false;

/*! Magnetometer driver data. */
static mag_drv_data_t *g_magDrvPtr = NULL;
/*! ADC channel configuration used by the light sensor. */
static adc16_channel_config_t *g_lsAdcChnlCfgPtr = NULL;
/*! Accelerometer driver data. */
static mma_drv_data_t *g_mmaDrvPtr = NULL;
/*! SIGFOX driver data. */
static sf_drv_data_t *g_sfDrvPtr = NULL;

/*******************************************************************************
 * Code
 *******************************************************************************/
#if defined(DEBUG) || defined(PRINT_CONSOLE)
static void PrintEvent(void)
{
    if (g_accelInt)
    {
        PRINTF("Acceleration detected\r\n");
    }
    if (g_lptmrPeriodInt)
    {
        PRINTF("Timer expired (periodic sending)\r\n");
    }
    if (g_sw1Int)
    {
        PRINTF("SW1 pushed\r\n");
    }
    if (g_sw2Int)
    {
        PRINTF("SW2 pushed\r\n");
    }
    if (g_sw3Int)
    {
        PRINTF("SW3 pushed\r\n");
    }
}
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
static void PrintCmdList(void)
{
    PRINTF("List of commands:\r\n");
    PRINTF("\t0 - it prints the Device ID and PAC\r\n");
    PRINTF("\t1 - it sets European standard ETSI (default)\r\n");
    PRINTF("\t2 - it sets USA standard FCC\r\n");
    PRINTF("\t3 - it sets Japanese/Korean standard ARIB\r\n");
    PRINTF("\t4 - it sets South American standard FCC\r\n");
    PRINTF("\t5 - it starts the application\r\n");
    PRINTF("Select a command and press enter: ");
}
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
static status_t PrintDevIdPac(void)
{
    status_t status = kStatus_Fail;
    uint8_t rcvPayloadData[SF_ACK_SPI_MSG_MAX_B];  /* Buffer containing received payload. */
    sf_msg_payload_t sndPayload = {0};             /* Dummy data to be sent. */
    sf_msg_payload_t rcvPayload =                  /* Received payload. */
    {
            SF_ACK_SPI_MSG_MAX_B,
            rcvPayloadData
    };
    uint8_t pldOf = 0U;                            /* Offset in the payload. */

    status = SF_SendCommand(g_sfDrvPtr, SF_CMD_GET_INFO_ID, &sndPayload, &rcvPayload,
            SF_ACK_SPI_MSG_MAX_B);

    if (kStatus_Success == status)
    {
        pldOf = SF_DEV_ID_OF;
        PRINTF("\tDevice ID = 0x");
        PRINTF("%02x%02x ", rcvPayloadData[pldOf + 3], rcvPayloadData[pldOf + 2]);
        PRINTF("%02x%02x\r\n", rcvPayloadData[pldOf + 1], rcvPayloadData[pldOf + 0]);

        pldOf = SF_DEV_PAC_OF;
        PRINTF("\tPAC = 0x");
        PRINTF("%02x%02x ", rcvPayloadData[pldOf], rcvPayloadData[pldOf + 1]);
        PRINTF("%02x%02x ", rcvPayloadData[pldOf + 2], rcvPayloadData[pldOf + 3]);
        PRINTF("%02x%02x ", rcvPayloadData[pldOf + 4], rcvPayloadData[pldOf + 5]);
        PRINTF("%02x%02x\r\n\r\n", rcvPayloadData[pldOf + 6], rcvPayloadData[pldOf + 7]);
    }

    return status;
}
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
static user_cmd_t GetUserCmd(void)
{
    int oldUserInput = 0U;              /* A character received in a previous reading. */
    int newUserInput = 0U;              /* Current read character. */
    user_cmd_t cmd = userCmdNotValid;   /* A command selected by the user. */

    do
    {   /* Read input until the user presses enter. */
        oldUserInput = newUserInput;
        newUserInput = GETCHAR();
        if (('\n' != newUserInput) && ('\r' != newUserInput))
        {
            PRINTF("%c", newUserInput);
        }
    }
    while (('\n' != newUserInput) && ('\r' != newUserInput));

    PRINTF("\r\n");

    /* Check if a character read in the previous reading is a valid command. */
    if ((APP_USER_CMD_FIRST <= oldUserInput) && (APP_USER_CMD_LAST >= oldUserInput))
    {
        cmd = (user_cmd_t)(oldUserInput - APP_USER_CMD_FIRST);
    }

    return cmd;
}
#endif

#if defined(DEBUG) || defined(PRINT_CONSOLE)
static status_t ProcessUserCmds(app_data_t *appDataPtr)
{
    status_t status = kStatus_Success;
    user_cmd_t cmd = userCmdNotValid;   /* User command. */
    bool startApp = false;              /* If the command processing should continue. */

    PrintCmdList();

    while ((false == startApp) && (kStatus_Success == status))
    {
        cmd = GetUserCmd();
        switch (cmd)
        {
            case userCmdAppStart:
                startApp = true;
                break;

            case userCmdPrintDevIdPac:
                status = PrintDevIdPac();
                if (kStatus_Success == status)
                {
                    PrintCmdList();
                }
                break;

            case userCmdSetETSI:
            case userCmdSetFCC_USA:
            case userCmdSetARIB:
            case userCmdSetFCC_SouthAmerica:
                appDataPtr->sfNetStandard = (sf_net_standard_t)((uint32_t)cmd - (uint32_t)userCmdSetETSI);
                status = SF_ChangeNetworkStandard(g_sfDrvPtr, appDataPtr->sfNetStandard);

                if (kStatus_Success == status)
                {
                    PRINTF("\tThe standard has been changed successfully\r\n\r\n");
                    PrintCmdList();
                }
                break;

            case userCmdNotValid:
            default:
                PRINTF("\tInvalid command number\r\n\r\n");
                PrintCmdList();
                break;
        }
    }

    return status;
}
#endif

static void ClearAllIntFlags(void)
{
    g_accelInt = false;
    g_lptmrPeriodInt = false;
    g_sw1Int = false;
    g_sw2Int = false;
    g_sw3Int = false;
}

static void StarBlinkingLED(void)
{
    uint32_t i = 0U;
    uint32_t coreClkHz = 0U;            /* Core clock in Hz. */

    /* Frequency of red blinking. */
    coreClkHz = CLOCK_GetCoreSysClkFreq() / 30U;

    for (;;)
    { /* Infinite loop to avoid leaving the main function */
        for (i = 0U; i < coreClkHz; i++)
        {
            ; /* Wait for a while. */
        }
        GPIO_TogglePinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN);
    }
}

static status_t SendTemp(sf_msg_payload_t *payload)
{
    int8_t   tempDegC = 0U;
    status_t status = kStatus_Success;

    status = MeasureMagTemp(g_magDrvPtr, &tempDegC);

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    PRINTF("\tTemperature:  %d deg. C\r\n", tempDegC);
#endif

#if (SEND_TO_SIGFOX_EN == true)
    PackData(APP_CMD_TEMP, tempDegC, payload);
    status |= SF_SendPayload(g_sfDrvPtr, payload);
#endif

    return status;
}

static status_t SendIllum(sf_msg_payload_t *payload)
{
    uint16_t illumLx = 0U;
    status_t status = kStatus_Success;

    illumLx = LS_GetLx(LS_ADC_BASE, LS_ADC_CHANNEL_GROUP, g_lsAdcChnlCfgPtr);

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    PRINTF("\tIlluminance:  %u lx\r\n", illumLx);
#endif

#if (SEND_TO_SIGFOX_EN == true)
    PackData(APP_CMD_ILLUM, illumLx, payload);
    status = SF_SendPayload(g_sfDrvPtr, payload);
#endif

    return status;
}

static status_t SendAccel(sf_msg_payload_t *payload)
{
    uint16_t accelmg = 0U;
    status_t status = kStatus_Success;

    status = GetTotalAccel(g_mmaDrvPtr, &accelmg);

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    PRINTF("\tAcceleration:  %u mg\r\n", accelmg);
#endif

#if (SEND_TO_SIGFOX_EN == true)
    PackData(APP_CMD_ACCEL, accelmg, payload);
    status |= SF_SendPayload(g_sfDrvPtr, payload);
#endif

    return status;
}

#if (SEND_TO_SIGFOX_EN == true)
static void PackData(uint8_t appCmd, uint32_t value, sf_msg_payload_t *framePld)
{
    (*(framePld->payload + SF_PAYLOAD_APP_CMD_ID)) = appCmd;
    (*(framePld->payload + (SF_PAYLOAD_VAL_LOW_ID - 3U))) = (uint8_t)(value >> 24U);
    (*(framePld->payload + (SF_PAYLOAD_VAL_LOW_ID - 2U))) = (uint8_t)(value >> 16U);
    (*(framePld->payload + (SF_PAYLOAD_VAL_LOW_ID - 1U))) = (uint8_t)(value >> 8U);
    (*(framePld->payload + SF_PAYLOAD_VAL_LOW_ID)) = (uint8_t)(value & 0xFFU);
}
#endif

static status_t SendData(void)
{
    status_t       status = kStatus_Success;
    uint8_t        pldData[SF_PAYLOAD_LEN_B];   /* Payload data. */
    sf_msg_payload_t payload =                  /* Payload to be sent. */
    {
            SF_PAYLOAD_LEN_B,
            pldData
    };

    /* Clear payload data. */
    memset((void *)pldData, 0x00U, SF_PAYLOAD_LEN_B);

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    PRINTF("Sending data to SIGFOX network, please wait\r\n");
#endif

    if (g_sw1Int | g_lptmrPeriodInt)
    {   /* Send temperature value when SW1 button pressed or LPTMR expired. */
        if ((status = SendTemp(&payload)) != kStatus_Success)
        {
            return status;
        }
    }

    if (g_sw2Int | g_lptmrPeriodInt)
    {   /* Send illuminance value when SW2 button pressed or LPTMR expired. */
        if ((status = SendIllum(&payload)) != kStatus_Success)
        {
            return status;
        }
    }

    if (g_accelInt)
    {   /* Send acceleration value when an acceleration event occurred. */
        if ((status = SendAccel(&payload)) != kStatus_Success)
        {
            return status;
        }
    }

    return status;
}

static status_t SetSigfoxWakeupMode(app_data_t *appDataPtr)
{
    status_t status = kStatus_Success;

#if (SF_SLEEP_MODE_EN == true)
    if (false == appDataPtr->sfOpModeNormal)
    {
        BOARD_SwitchAppPortMux(false);

        if (kStatus_Success == status)
        {
            status = SF_WakeUp(g_sfDrvPtr);
        }
        if (kStatus_Success == status)
        {
            appDataPtr->sfOpModeNormal = true;
        }

#if defined(DEBUG) || defined(PRINT_CONSOLE)
        if (kStatus_Success == status)
        {
            PRINTF("SIGFOX device is in the normal mode\r\n");
        }
#endif
    }

    if (sfNetStandardETSI != appDataPtr->sfNetStandard)
    {
        if (kStatus_Success == status)
        {
            status = SF_ChangeNetworkStandard(g_sfDrvPtr, appDataPtr->sfNetStandard);
        }
    }
#endif

    return status;
}

static status_t SetSigfoxSleepMode(app_data_t *appDataPtr)
{
    status_t status = kStatus_Success;

#if (SF_SLEEP_MODE_EN == true)
    if (true == appDataPtr->sfOpModeNormal)
    {
        if (kStatus_Success == status)
        {
            status = SF_Sleep(g_sfDrvPtr);
        }
        if (kStatus_Success == status)
        {
            appDataPtr->sfOpModeNormal = false;

            /* Change pin mux of SPI CLK, MOSI and MISO pins to GPIO.
             * Output value of the GPIO pins is LOW. The reason is to lower
             * power consumption of the SIGFOX board. */
            BOARD_SwitchAppPortMux(true);
        }
    }

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    if (kStatus_Success == status)
    {
        PRINTF("SIGFOX device is in the sleep mode\r\n");
    }
#endif

#endif

    return status;
}

static status_t ProcessInitDebounce(void)
{
    /* Start the debounce timer, the periodic sending starts when the
     * debounce timer expires. */
    SetLPTMRPeriodAndReset(DEBOUNCE_TIME_S);

#ifdef DEBUG
    PRINTF("\r\nTransition from InitDebounce to WaitDebounce state\r\n");
#endif

    return kStatus_Success;
}

static status_t ProcessWaitDebonce(void)
{
    status_t status = kStatus_Success;

    /* Cancel previous interrupts. */
    status = ClearAccelInt(g_mmaDrvPtr);

    /* Enter critical section. */
    __disable_irq();

    ClearAllIntFlags();

    /* Leave critical section. */
    __enable_irq();

    /* Signalize that the device is ready. */
    LED_RED_OFF();

#ifdef DEBUG
    PRINTF("\r\nTransition from WaitDebounce to InitPeriodic state\r\n", status);
#endif

    return status;
}

static status_t ProcessInitPeriodic(void)
{
#if (SEND_PERIODICALLY_EN == true)
    /* Start the periodic sending timer. */
    SetLPTMRPeriodAndReset(SEND_PERIOD_S);
#else
    /* Stop the timer. */
    LPTMR_StopTimer(LPTMR_DEV);
#endif


#if defined(PRINT_CONSOLE) || defined(DEBUG)
    PRINTF("\r\nApplication is ready\r\n");
#endif

#ifdef DEBUG
    PRINTF("\r\nTransition from InitPeriodic to Ready state\r\n");

#if (MCU_SLEEP_MODE_EN == true)
    PRINTF("Go to VLPS power mode\r\n");
#endif

#endif

#if (MCU_SLEEP_MODE_EN == true)
    SetVlpsMode();
#endif

    return kStatus_Success;
}

static status_t ProcessReadyEvent(app_data_t *appDataPtr)
{
    status_t status = kStatus_Success;
    bool accelIntTemp;

    /* Signalize that the device is busy. */
    LED_RED_ON();

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    PrintEvent();
#endif

    /* Stop the timer. */
    LPTMR_StopTimer(LPTMR_DEV);

    if (true == APP_EVENT_SEND_OCCURRED())
    {
        status = SetSigfoxWakeupMode(appDataPtr);
    }

    /* Send data to SIGFOX network. */
    if ((kStatus_Success == status) && (true == APP_EVENT_SEND_OCCURRED()))
    {
        status = SendData();
    }

    if ((kStatus_Success == status) && (g_sw3Int))
    {   /* Put the SIGFOX device into the sleep mode. */

        status = SetSigfoxSleepMode(appDataPtr);
    }

    if (g_lptmrPeriodInt == true)
    {   /* Signalize that the device is in waiting state. */
        LED_RED_OFF();
    }

    /* Enter critical section. */
    __disable_irq();

    /* Clear flags. */
    accelIntTemp = g_accelInt;
    ClearAllIntFlags();

    /* Leave critical section. */
    __enable_irq();

    if ((kStatus_Success == status) && (accelIntTemp))
    {   /* Accelerometer detected an event, clear the status register. */
        if ((status = ClearAccelInt(g_mmaDrvPtr)) != kStatus_Success)
        {
            return status;
        }
    }

    return status;
}

static status_t ProcessReady(app_data_t *appDataPtr)
{
    status_t status = kStatus_Fail;

    if (true == APP_EVENT_NONPERIOD_OCCURRED())
    {   /* Accel. or pushbutton event occurred, start the debounce timer. */
        appDataPtr->appState = appStateInitDebounce;
        status = ProcessReadyEvent(appDataPtr);

#ifdef DEBUG
        PRINTF("\r\nTransition from Ready to InitDebounce state\r\n", status);
#endif

    }
    else if (g_lptmrPeriodInt)
    {   /* The periodic sending timer expired, start the timer. */
        appDataPtr->appState = appStateInitPeriodic;
        status = ProcessReadyEvent(appDataPtr);

#ifdef DEBUG
        PRINTF("\r\nTransition from Ready to InitPeriodic state\r\n", status);
#endif
    }
    else
    {
        assert("Unknown interrupt source");
        status = kStatus_Fail;
    }

    return status;
}

static status_t ProcessAppState(app_data_t *appDataPtr)
{
    status_t status = kStatus_Success;

    switch (appDataPtr->appState)
    {
        case appStateInitDebounce:
            status = ProcessInitDebounce();
            appDataPtr->appState = appStateWaitDebounce;
            break;

        case appStateWaitDebounce:
            if (g_lptmrPeriodInt)
            {
                g_lptmrPeriodInt = false;
                status = ProcessWaitDebonce();
                appDataPtr->appState = appStateInitPeriodic;
            }
            break;

        case appStateInitPeriodic:
            status = ProcessInitPeriodic();
            appDataPtr->appState = appStateReady;
            break;

        case appStateReady:
            if (true == APP_EVENT_OCCURRED())
            {   /* An interrupt occurred. */
                status = ProcessReady(appDataPtr);
            }
            break;

        default:
            assert("Undefined application state");
            status = kStatus_Fail;
    }

    return status;
}

static status_t StartApp(app_data_t *appDataPtr)
{
    status_t    status = kStatus_Success;

    /* Initialize application state. */
    appDataPtr->sfNetStandard = sfNetStandardETSI;
    appDataPtr->sfOpModeNormal = true;
    appDataPtr->appState = appStateInitDebounce;

    /* Enter critical section. */
    __disable_irq();

    /* Clear event flags. */
    ClearAllIntFlags();

    /* Leave critical section. */
    __enable_irq();

    while (true)
    {
        status = ProcessAppState(appDataPtr);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    return status;
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    status_t status = kStatus_Success;  /* Error code. */
    mag_drv_data_t magDrvData = {0};    /* Magnetometer driver data. */
    /* ADC channel configuration (light sensor). */
    adc16_channel_config_t lsAdcChnlConfig = {0};
    mma_drv_data_t mmaDrvData = {0};    /* Accelerometer driver data. */
    sf_drv_data_t sfDrvData;            /* SIGFOX driver data. */
    app_data_t appData = {0};           /* Application data. */

    /* The application uses global variables to access driver data structures. */
    g_magDrvPtr = &magDrvData;
    g_lsAdcChnlCfgPtr = &lsAdcChnlConfig;
    g_mmaDrvPtr = &mmaDrvData;
    g_sfDrvPtr = &sfDrvData;

    /* Enable transition to VLPR mode (it has to be done after reset). */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);

    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockVLPR();

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    BOARD_InitVlprDebugConsole();
#endif

    /* Set power mode. */
    if ((status = SetVlprMode()) != kStatus_Success)
    {
#if defined(DEBUG) || defined(PRINT_CONSOLE)
        PRINTF("An error occurred in the SetVlprMode function (%d)\r\n", status);
#endif
    }

#if defined(DEBUG) || defined(PRINT_CONSOLE)
    if (kStatus_Success == status)
    {
        PRINTF("------------------------------\r\n"
               "    Start demo\r\n"
               "------------------------------\r\n");
    }
#endif

    /* Initialize peripherals and sensors. */
    if (kStatus_Success == status)
    {
        status = SetupDevices(&magDrvData, &mmaDrvData, &lsAdcChnlConfig, &sfDrvData);
        if (kStatus_Success != status)
        {
#if defined(DEBUG) || defined(PRINT_CONSOLE)
            PRINTF("An error occurred in the SetupDevices function (%d)\r\n", status);
#endif
        }
    }

    /* Process user commands. */
#if defined(DEBUG) || defined(PRINT_CONSOLE)
    if (kStatus_Success == status)
    {
        if ((status = ProcessUserCmds(&appData)) != kStatus_Success)
        {
#if defined(DEBUG) || defined(PRINT_CONSOLE)
            PRINTF("An error occurred in the ProcessUserCmds function (%d)\r\n", status);
#endif
        }
    }
#endif

    /* Start application. */
    if (kStatus_Success == status)
    {
        if ((status = StartApp(&appData)) != kStatus_Success)
        {
#if defined(DEBUG) || defined(PRINT_CONSOLE)
            PRINTF("An error occurred in the StartApp function (%d)\r\n", status);
#endif
        }
    }

    /* An error occurred, start blinking the red LED. */
    StarBlinkingLED();
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
