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
 * @file mma.h
 *
 * This module implements the driver for MMA8451Q accelerometer.
 *
 * Current implementation includes functions for basic settings of the device,
 * transient detection configuration, access to registers and its interpretation.
 */

#ifndef SOURCE_MMA_H_
#define SOURCE_MMA_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_common.h"
#include "mma_mma8451q.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup mma_enum_group
 * Enumerations for the accelerometer driver.
 * @{
 */
/*! Resolution of measured acceleration data. */
typedef enum
{
    mmaBitRes8 = 0U,                /*!< Resolution is 8 bit. */
    mmaBitRes14,                    /*!< Resolution is 14 bit. */
} mma_bit_res_t;

/*! Full-scale value range. */
typedef enum
{
    mmaFullScale2g = MMA_FS_2G,     /*!< Range is 2 g. */
    mmaFullScale4g = MMA_FS_4G,     /*!< Range is 4 g. */
    mmaFullScale8g = MMA_FS_8G,     /*!< Range is 8 g. */
} mma_full_scale_t;

/*! System output data rate. */
typedef enum
{
    mmaOdr800Hz = MMA_SET_DR_800_HZ, /*!<  Data rate is 800 Hz. */
    mmaOdr400Hz = MMA_SET_DR_400_HZ, /*!<  Data rate is 400 Hz. */
    mmaOdr200Hz = MMA_SET_DR_200_HZ, /*!<  Data rate is 200 Hz. */
    mmaOdr100Hz = MMA_SET_DR_100_HZ, /*!<  Data rate is 100 Hz. */
    mmaOdr50Hz = MMA_SET_DR_50_HZ,  /*!<  Data rate is 50 Hz. */
    mmaOdr12Hz = MMA_SET_DR_12_HZ,  /*!<  Data rate is 12.5 Hz. */
    mmaOdr6Hz = MMA_SET_DR_6_HZ,    /*!<  Data rate is 6.25 Hz. */
    mmaOdr1Hz = MMA_SET_DR_1_HZ,    /*!<  Data rate is 1.56 Hz. */
} mma_odr_t;

/*! Selection of an axis. It is used to calculate address of device registers. */
typedef enum
{
    mmaAxisX = 0U,                  /*!< X-axis. */
    mmaAxisY,                       /*!< Y-axis. */
    mmaAxisZ,                       /*!< Z-axis. */
} mma_axis_t;

/*! This enumeration lists functional blocks with the interrupt function.
 * Values of the items are bit shifts of the CTRL_REG4 and CTRL_REG5 registers
 * (they uses same shift values).
 * Note: This enumeration is incomplete. */
typedef enum
{
    mmaIntBlockTrans = MMA_INT_EN_TRANS_SHIFT, /*!< Transient detection block. */
    mmaIntBlockFifo = MMA_INT_EN_FIFO_SHIFT, /*!< FIFO block. */
} mma_int_block_t;

/*! This enumeration contains interrupt pins available for the device.
 * Note that corresponding block should be selected with use of the
 * mma_int_block_t enumeration. */
typedef enum
{
    mmaIntPinInt2 = 0U,             /*!< INT2 pin. */
    mmaIntPinInt1 = 1U,             /*!< INT1 pin. */
} mma_int_pin_t;

/*! This enumeration lists event flags on X/Y/Z axis transient acceleration
 * greater than transient threshold. It is used to enable an event. */
typedef enum
{
    mmaTransEventEnZ = MMA_EN_ZTEFE, /*!< Event flag enable on Z axis. */
    mmaTransEventEnY = MMA_EN_YTEFE, /*!< Event flag enable on Y axis. */
    mmaTransEventEnX = MMA_EN_XTEFE, /*!< Event flag enable on X axis. */
} mma_trans_event_en_t;

/*! FIFO modes. */
typedef enum
{
    mmaFifoDisabled = MMA_F_MODE_DIS,        /*!< FIFO is disabled. */
    mmaFifoCircularBuffer = MMA_F_MODE_CIRC, /*!< FIFO is in circular buffer mode. */
    mmaFifoFillBuffer = MMA_F_MODE_FILL,     /*!< FIFO is in fill buffer mode. */
    mmaFifoTrigger = MMA_F_MODE_TRIG,        /*!< FIFO is trigger mode. */
} mma_fifo_mode_t;

/*! FIFO triggers. */
typedef enum
{
    mmaFifoTrigTrans = MMA_EN_TRIG_TRANS,   /*!< Transient interrupt trigger. */
    mmaFifoTrigLNDPRT = MMA_EN_TRIG_LNDPRT, /*!< Landscape/portrait orientation interrupt trigger. */
    mmaFifoTrigPulse = MMA_EN_TRIG_PULSE,   /*!< Pulse interrupt trigger. */
    mmaFifoTrigFFMT = MMA_EN_TRIG_FF_MT,    /*!< Freefall/motion trigger. */
} mma_fifo_trig_t;
/*! @} */

/*!
 * @addtogroup mma_struct_group
 * Structures for the accelerometer driver.
 * @{
 */
/*! Axis data. */
typedef struct
{
    int16_t x;                      /*!< X axis data. */
    int16_t y;                      /*!< Y axis data. */
    int16_t z;                      /*!< Z axis data. */
} mma_axis_data_t;

/*! @brief User configuration of the accelerometer. */
typedef struct
{
    I2C_Type *i2cBase;              /*!< Pointer to a configured I2C peripheral
                                         (I2C base). */
    uint8_t i2cSlaveAddr;           /*!< I2C slave device address */
    mma_bit_res_t bitRes;           /*!< Resolution of measured data. */
    mma_odr_t odr;                  /*!< Output data rate. */
    mma_full_scale_t fullScale;     /*!< Full-scale value range. */
} mma_user_config_t;

/*! Configuration of the transition detection mode. */
typedef struct
{
    uint8_t axisEventEn;            /*!< Event flags enable on X/Y/Y axis. The
                                         user can use ORed values of the
                                         mma_trans_event_en_t enumeration. Axis
                                         events not listed here are disabled. */
    bool latchEn;                   /*!< Enables event flag latch (true - enabled,
                                         false - disabled). */
    uint16_t thsmg;                 /*!< The transient threshold in mg.
                                         Admissible range is <0; 8000> mg. */
    uint8_t debounceVal;            /*!< Value of the debounce counter register
                                         (TRANSIENT_COUNT). */
} mma_trans_det_conf_t;

/*! Configuration of FIFO. */
typedef struct
{
    mma_fifo_mode_t fifoMode;       /*!< FIFO mode. */
    uint8_t watermarkCnt;           /*! Watermark value. It is used in FIFO
                                        trigger mode. The FIFO data will contain
                                        (32 - watermark) samples of data leading
                                        up to an event and (watermark) samples
                                        after the event. Admissible range is
                                        {0,..., 32} (0 - disabled). */
    uint8_t fifoTriggers;           /*!< FIFO trigger configuration. The user
                                         can use mma_fifo_trig_t enumeration.
                                         Triggers listed here will be enabled,
                                         other triggers will be disabled. */
} mma_fifo_conf_t;

/*! @brief This data structure is used by the Accelerometer driver (this is the
 * first parameter of most functions). */
typedef struct
{
    I2C_Type *i2cBase;              /*!< Pointer to a configured I2C peripheral
                                         (I2C base). */
    uint8_t i2cSlaveAddr;           /*!< I2C slave device address */
    mma_bit_res_t bitRes;           /*!< Resolution of measured data. It is used
                                         in calculations. */
    uint8_t stepmg;                 /*!< Acceleration data step size in
                                         0.00001 g/LSb. */
} mma_drv_data_t;
/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup mma_functions_group
 * @{
 */

#ifdef DEBUG
/*!
 * @brief This function prints a selected register to the virtual serial output.
 *
 * @param drvData Driver run-time data.
 * @param regAddr Memory data address.
 * @param regName Name of a register.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
void MMA_PrintReg(mma_drv_data_t *drvData, uint8_t regAddr, char *regName);
#endif

/*!
 * @brief Initializes the acceleromter driver based on user configuration.
 *
 * @param drvData    Driver run-time data.
 * @param userConfig User-provided configuration of the driver.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_Init(mma_drv_data_t *drvData, const mma_user_config_t *userConfig);

/*!
 * @brief This function configures the accelerometer for the transient detection
 * function.
 *
 * The user must put the device into the standby mode before calling
 * this function.
 * This function does not change Output data rate, full-scale value nor
 * interrupt settings.
 *
 * @param drvData Driver run-time data.
 * @param conf    Configuratio of the transient detection function.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_ConfTransDet(mma_drv_data_t *drvData, mma_trans_det_conf_t *conf);

/*!
 * @brief This function configures FIFO (mode, watermark and triggers).
 *
 * The user must put the device into the standby mode before calling
 * this function.
 *
 * @param drvData  Driver run-time data.
 * @param fifoConf FIFO configuration structure.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_ConfFifo(mma_drv_data_t *drvData, mma_fifo_conf_t *fifoConf);

/*!
 * @brief Writes a byte to the device using an 8bit memory address.
 *
 * @param drvData Driver run-time data.
 * @param regAddr Device memory address.
 * @param value   Data value.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_WriteReg(mma_drv_data_t *drvData, uint8_t regAddr, uint8_t value);

/*!
 * @brief Reads bytes from the device using an 8bit memory address.
 *
 * @param drvData      Driver run-time data.
 * @param regAddr      Device memory address.
 * @param valueArr     Pointer where the read data are stored.
 * @param valueArrSize Size of the data array (parameter valueArr) in bytes.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_ReadRegs(mma_drv_data_t *drvData, uint8_t regAddr,
        uint8_t *valueArr, uint8_t valueArrSize);

/*!
 * @brief Updates a byte of the device using an 8bit memory address.
 *
 * @param drvData Driver run-time data.
 * @param regAddr Device memory address.
 * @param value   Data value.
 * @param mask    Mask of the data value. Only set bits are updated.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_UpdateReg(mma_drv_data_t *drvData, uint8_t regAddr,
        uint8_t value, uint8_t mask);

/*!
 * @brief This function sets full-scale value range.
 *
 * It updates acceleration data step stored in the driver data structure.
 *
 * @param drvData Driver run-time data.
 * @param scale   Full-scale value range.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_SetFullScaleRange(mma_drv_data_t *drvData, mma_full_scale_t scale);

/*!
 * @brief This function enables interrupt events.
 *
 * User must put the device into the standby mode before calling this function.
 * It sets the routing of event detection flags to the system's interrupt controller
 * and routes the transient interrupt in the system's interrupt controller
 * to a hardware pin. It updates CTRL_REG4 and CTRL_REG5 registers.
 *
 * @param drvData  Driver run-time data.
 * @param blockSel Selection of a functional block.
 * @param pinSel   Selection of a HW interrupt pin.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_EnableInt(mma_drv_data_t *drvData, mma_int_block_t blockSel,
        mma_int_pin_t pinSel);

/*!
 * @brief This function sets the device operation mode (sleep or standby).
 *
 * @param drvData Driver run-time data.
 * @param active  Desired device mode (true - active, false - standby).
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_SetOpMode(mma_drv_data_t *drvData, bool active);

/*!
 * @brief This function resets the device.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_ResetDevice(mma_drv_data_t *drvData);

/*!
 * @brief This function performs a measurement on a channel and returns the raw
 * value.
 *
 * @param drvData Driver run-time data.
 * @param axis    An axis to be measured.
 * @param val     Pointer where the resulting measured data are stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_MeasureRawAxis(mma_drv_data_t *drvData, mma_axis_t axis, uint16_t *val);

/*!
 * @brief This function gets value of an axis in mg.
 *
 * @param drvData Driver run-time data.
 * @param axis    An axis to be measured.
 * @param val     Pointer where the resulting data are stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_GetAxismg(mma_drv_data_t *drvData, mma_axis_t axis, int16_t *val);

/*!
 * @brief This function gets value of all axis in mg.
 *
 * @param drvData Driver run-time data.
 * @param val     Pointer where the resulting data are stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
status_t MMA_GetAllAxismg(mma_drv_data_t *drvData, mma_axis_data_t *val);
/*! @} */

#endif /* SOURCE_MMA_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
