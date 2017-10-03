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
 * @file mag_mag3110.h
 *
 * Register map for MAG3110 magnetometer.
 */

#ifndef SOURCE_MAG_MAG3110_H_
#define SOURCE_MAG_MAG3110_H_

/*******************************************************************************
 * General definitions
 ******************************************************************************/
/*!
 * @name General bit operations */
/*! @{ */

/*!
 * @brief Macro for setting value of the bit given by the mask.
 *
 * @param Msg Message to be modified
 * @param Mask Bit selection in message
 */
#define MAG_SET_BIT_VALUE(Msg, Mask) ((Msg) | (Mask))

/*!
 * @brief Macro for unsetting value of the bit given by the mask.
 *
 * @param Msg Message to be modified
 * @param Mask Bit selection in message
 */
#define MAG_UNSET_BIT_VALUE(Msg, Mask) ((Msg) & ~(Mask))

/*!
 * @brief Macro for getting value of the bit given by the mask.
 *
 * @param Msg Message to be read
 * @param Mask Bit selection in message
 * @param Shift Bit shift in message
 */
#define MAG_GET_BIT_VALUE(Msg, Mask, Shift) (((Msg) & (Mask)) >> (Shift))

/*!
 * @brief Macro for setting value of bits given by the mask.
 *
 * @param Msg Message to be modified
 * @param Mask Bits selection in message
 * @param Shift Bits shift in message
 * @param Val Value to be applied
 * @param Range Admissible range of value
 */
#define MAG_SET_BITS_VALUE(Msg, Mask, Shift, Val, Range) (((Msg) & ~(Mask)) | (((Val) & (Range)) << (Shift)))

/*!
 * @brief Macro for getting value of bits given by the mask.
 *
 * @param Msg Message to be read
 * @param Mask Bits selection in message
 * @param Shift Bits shift in message
 */
#define MAG_GET_BITS_VALUE(Msg, Mask, Shift) (((Msg) & (Mask)) >> (Shift))
/*! @} */

/*******************************************************************************
 * Registers definitions
 ******************************************************************************/
/*!
 * @name DR_STATUS register
 * @{
 */
/*! Register address. */
#define MAG_REG_DR_STATUS           0x00U

/*! Mask for X or Y or Z-axis new Data Ready. */
#define MAG_ZYXDR_MASK              0x08U
/*! @} */

/*!
 * @name Sample data registers
 * It includes OUT_X_MSB, OUT_X_LSB, OUT_Y_MSB, OUT_Y_LSB, OUT_Z_MSB, OUT_Z_LSB
 * registers.
 * @{
 */
/*! OUT_X_MSB register address. */
#define MAG_REG_OUT_X_MSB           0x01U
/*! OUT_X_LSB register address. */
#define MAG_REG_OUT_X_LSB           0x02U
/*! OUT_Y_MSB register address. */
#define MAG_REG_OUT_Y_MSB           0x03U
/*! OUT_Y_LSB register address. */
#define MAG_REG_OUT_Y_LSB           0x04U
/*! OUT_Z_MSB register address. */
#define MAG_REG_OUT_Z_MSB           0x05U
/*! OUT_Z_LSB register address. */
#define MAG_REG_OUT_Z_LSB           0x06U

/*!
 * @brief This macro returns raw value of an axis.
 *
 * @param msbReg OUT_x_MSB register where x is X/Y/Z.
 * @param lsbReg OUT_x_LSB register where x is X/Y/Z.
 */
#define MAG_GET_REG_OUT_RAW(msbReg, lsbReg) \
    (((uint16_t)(msbReg) << 8U) | (uint16_t)(lsbReg))
/*! @} */

/*!
 * @name DIE_TEMP register
 * @{
 */
/*! Register address. */
#define MAG_REG_DIE_TEMP            0x0FU

/*!
 * @brief This macro converts a raw temperature value to °C.
 *
 * @param tempRaw Raw value of the temperature.
 */
#define MAG_GET_TEMP_DEGC(tempRaw) \
    ((int8_t)(tempRaw))
/*! @} */

/*!
 * @name CTRL_REG1 register
 * @{
 */
/*! Register address. */
#define MAG_REG_CTRL_REG1           0x10U

/*! Mask for operating mode selection. */
#define MAG_AC_MASK                 0x01U
/*! Mask for trigger immediate measurement. */
#define MAG_TM_MASK                 0x02U
/*! Mask for over sampling ratio for the measurement. */
#define MAG_OSx_MASK                0x18U
/*! Mask for output data rate selection. */
#define MAG_DRx_MASK                0xE0U

/*! Shift for operating mode selection. */
#define MAG_AC_SHIFT                0x00U
/*! Shift for trigger immediate measurement. */
#define MAG_TM_SHIFT                0x01U
/*! Shift for over sampling ratio for the measurement. */
#define MAG_OSx_SHIFT               0x03U
/*! Shift for Output data rate selection. */
#define MAG_DRx_SHIFT               0x5U

/*! Set standby mode. */
#define MAG_SET_STANDBY_AC          (0U << MAG_AC_SHIFT)
/*! Set active mode. */
#define MAG_SET_ACTIVE_AC           (1U << MAG_AC_SHIFT)

/*! Trigger measurement. */
#define MAG_TRIGGER_MEASUREMENT     (1U << MAG_TM_SHIFT)

/*!
 * @brief This macro gets value of the over sampling ratio for the measurement,
 * which can be written into the CTRL_REG1 register.
 *
 * @param Value of the OS1 and OS0 bits.
 */
#define MAG_GET_OVER_SAMP(os) \
    (((os) << MAG_OSx_SHIFT) & MAG_OSx_MASK)

/*!
 * @brief This macro gets value of the output data rate, which can be written
 * into the CTRL_REG1 register.
 *
 * @param Value of the DR2, DR1 and DR0 bits.
 */
#define MAG_GET_DATA_RATE(dr) \
    (((dr) << MAG_DRx_SHIFT) & MAG_DRx_MASK)
/*! @} */

/*!
 * @name CTRL_REG2 register
 * @{
 */
/*! Register address. */
#define MAG_REG_CTRL_REG2           0x11U

/*! Mask for automatic Magnetic Sensor Reset. */
#define MAG_AUTO_MRST_EN_MASK       0x80U
/*! Mask for Magnetic Sensor Reset. */
#define MAG_MAG_RST_MASK            0x10U

/*! Shift for automatic Magnetic Sensor Reset. */
#define MAG_AUTO_MRST_EN_SHIFT      0x07U
/*! Shift for Magnetic Sensor Reset. */
#define MAG_MAG_RST_SHIFT           0x04U

/*! Enable automatic Magnetic Sensor Reset. */
#define MAG_EN_AUTO_MRST            (1U << MAG_AUTO_MRST_EN_SHIFT)
/*! Disable automatic Magnetic Sensor Reset. */
#define MAG_DIS_AUTO_MRST           (0U << MAG_AUTO_MRST_EN_SHIFT)

/*! Magnetic Sensor Reset. */
#define MAG_SET_MAG_RST             (1U << MAG_MAG_RST_SHIFT)
/*! @} */

#endif /* SOURCE_MAG_MAG3110_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
