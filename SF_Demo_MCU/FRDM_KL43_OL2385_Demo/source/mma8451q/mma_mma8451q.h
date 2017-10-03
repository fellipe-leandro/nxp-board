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
 * @file mma_mma8451q.h
 *
 * Register map for MMA8451Q accelerometer.
 */

#ifndef SOURCE_MMA_MMA8451Q_H_
#define SOURCE_MMA_MMA8451Q_H_


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
#define MMA_SET_BIT_VALUE(Msg, Mask) ((Msg) | (Mask))

/*!
 * @brief Macro for unsetting value of the bit given by the mask.
 *
 * @param Msg Message to be modified
 * @param Mask Bit selection in message
 */
#define MMA_UNSET_BIT_VALUE(Msg, Mask) ((Msg) & ~(Mask))

/*!
 * @brief Macro for getting value of the bit given by the mask.
 *
 * @param Msg Message to be read
 * @param Mask Bit selection in message
 * @param Shift Bit shift in message
 */
#define MMA_GET_BIT_VALUE(Msg, Mask, Shift) (((Msg) & (Mask)) >> (Shift))

/*!
 * @brief Macro for setting value of bits given by the mask.
 *
 * @param Msg Message to be modified
 * @param Mask Bits selection in message
 * @param Shift Bits shift in message
 * @param Val Value to be applied
 * @param Range Admissible range of value
 */
#define MMA_SET_BITS_VALUE(Msg, Mask, Shift, Val, Range) (((Msg) & ~(Mask)) | (((Val) & (Range)) << (Shift)))

/*!
 * @brief Macro for getting value of bits given by the mask.
 *
 * @param Msg Message to be read
 * @param Mask Bits selection in message
 * @param Shift Bits shift in message
 */
#define MMA_GET_BITS_VALUE(Msg, Mask, Shift) (((Msg) & (Mask)) >> (Shift))
/*! @} */

/*******************************************************************************
 * Registers definitions
 ******************************************************************************/
/*!
 * @name STATUS register
 * This register definition applies when the accelerometer is NOT configured
 * in a FIFO mode.
 * @{
 */
#define MMA_REG_STATUS              0x00U
/*! @} */

/*!
 * @name F_STATUS register
 * This register definition applies when the accelerometer is configured
 * in a FIFO mode.
 * @{
 */
#define MMA_REG_F_STATUS            0x00U

/*! Mask for FIFO overflow event flag. */
#define MMA_F_OVF_FLAG_MASK         0x80U
/*! Mask for FIFO watermark event flag. */
#define MMA_F_WMRK_FLAG_MASK        0x40U
/*! Mask for FIFO sample counter. */
#define MMA_F_CNT_MASK              0x3FU

/*! Shift for FIFO overflow event flag. */
#define MMA_F_OVF_FLAG_SHIFT        0x07U
/*! Shift for FIFO watermark event flag. */
#define MMA_F_WMRK_FLAG_SHIFT       0x06U
/*! Shift for FIFO sample counter. */
#define MMA_F_CNT_SHIFT             0x00U
/*! @} */

/*!
 * @name Sample data registers
 * It includes OUT_X_MSB, OUT_X_LSB, OUT_Y_MSB, OUT_Y_LSB, OUT_Z_MSB, OUT_Z_LSB
 * registers.
 * @{
 */
/*! OUT_X_MSB register address. */
#define MMA_REG_OUT_X_MSB           0x01U
/*! OUT_X_LSB register address. */
#define MMA_REG_OUT_X_LSB           0x02U
/*! OUT_Y_MSB register address. */
#define MMA_REG_OUT_Y_MSB           0x03U
/*! OUT_Y_LSB register address. */
#define MMA_REG_OUT_Y_LSB           0x04U
/*! OUT_Z_MSB register address. */
#define MMA_REG_OUT_Z_MSB           0x05U
/*! OUT_Z_LSB register address. */
#define MMA_REG_OUT_Z_LSB           0x06U

/*! Max. value of signed 14 bit value of an output axis register. */
#define MMA_OUT_AXIS_MAX_CNT        8191U

/*! A factor value used to convert raw output data to mg. */
#define MMA_OUTDATA_FACTOR          100
/*! Accelerometer output data step [0.00001 g/LSb], 240 mg/LSb.
 * It is intended for 14 bit resolution and 2 g full-scale range. */
#define MMA_OUTDATA_14B_DIV_2G      24
/*! Accelerometer output data step [0.00001 g/LSb], 480 mg/LSb.
 * It is intended for 14 bit resolution and 4 g full-scale range. */
#define MMA_OUTDATA_14B_DIV_4G      48
/*! Accelerometer output data step [0.00001 g/LSb], 970 mg/LSb.
 * It is intended for 14 bit resolution and 8 g full-scale range. */
#define MMA_OUTDATA_14B_DIV_8G      97

/*!
 * @brief This macro returns raw value of an axis.
 *
 * @param msbReg OUT_x_MSB register where x is X/Y/Z.
 * @param lsbReg OUT_x_LSB register where x is X/Y/Z.
 */
#define MMA_GET_REG_OUT_RAW(msbReg, lsbReg) \
    (((uint16_t)(msbReg) << 8U) | (uint16_t)(lsbReg))
/*! @} */

/*!
 * @name F_SETUP register
 * @{
 */
/*! Register address. */
#define MMA_REG_F_SETUP             0x09U

/*! Mask for FIFO buffer overflow mode. */
#define MMA_F_MODE_MASK             0xC0U
/*! Mask for FIFO event sample count watermark. */
#define MMA_F_WMRK_CNT_MASK         0x3FU

/*! Shift for FIFO buffer overflow mode. */
#define MMA_F_MODE_SHIFT            0x06U
/*! Shift for FIFO event sample count watermark. */
#define MMA_F_WMRK_CNT_SHIFT        0x00U

/*! FIFO is disabled. */
#define MMA_F_MODE_DIS              (0x00U << MMA_F_MODE_SHIFT)
/*! FIFO is configured for circular buffer mode. */
#define MMA_F_MODE_CIRC             (0x01U << MMA_F_MODE_SHIFT)
/*! FIFO is configured for fill buffer mode. */
#define MMA_F_MODE_FILL             (0x02U << MMA_F_MODE_SHIFT)
/*! FIFO is configured for trigger buffer mode. */
#define MMA_F_MODE_TRIG             (0x03U << MMA_F_MODE_SHIFT)

/*!
 * @brief This macro returns value of the F_SETUP register with watermark value.
 *
 * @param watermark Watermark value (6 bit value, but admissible range is
 * {0,... , 32}).
 */
#define MMA_GET_F_WATERMARK(watermark) \
    (((watermark) & MMA_F_WMRK_CNT_MASK) << MMA_F_WMRK_CNT_SHIFT)
/*! @} */

/*!
 * @name TRIG_CFG register
 * @{
 */
/*! Register address. */
#define MMA_REG_TRIG_CFG            0x0AU

/*! Mask for transient interrupt trigger bit. */
#define MMA_TRIG_TRANS_MASK         0x20U
/*! Mask for landscape/portrait orientation interrupt trigger bit. */
#define MMA_TRIG_LNDPRT_MASK        0x10U
/*! Mask for pulse interrupt trigger bit. */
#define MMA_TRIG_PULSE_MASK         0x08U
/*! Mask for freefall/motion trigger bit. */
#define MMA_TRIG_FF_MT_MASK         0x04U

/*! Shift for transient interrupt trigger bit. */
#define MMA_TRIG_TRANS_SHIFT        0x05U
/*! Shift for landscape/portrait orientation interrupt trigger bit. */
#define MMA_TRIG_LNDPRT_SHIFT       0x04U
/*! Shift for pulse interrupt trigger bit. */
#define MMA_TRIG_PULSE_SHIFT        0x03U
/*! Shift for freefall/motion trigger bit. */
#define MMA_TRIG_FF_MT_SHIFT        0x02U

/*! Enables transient interrupt trigger. */
#define MMA_EN_TRIG_TRANS           (1U << MMA_TRIG_TRANS_SHIFT)
/*! Enables landscape/portrait orientation interrupt trigger. */
#define MMA_EN_TRIG_LNDPRT          (1U << MMA_TRIG_LNDPRT_SHIFT)
/*! Enables pulse interrupt trigger. */
#define MMA_EN_TRIG_PULSE           (1U << MMA_TRIG_PULSE_SHIFT)
/*! Enables freefall/motion trigger. */
#define MMA_EN_TRIG_FF_MT            (1U << MMA_TRIG_FF_MT_SHIFT)
/*! @} */

/*!
 * @name INT_SOURCE register
 * @{
 */
/*! Register address. */
#define MMA_REG_INT_SOURCE          0x0CU

/*! Mask for transient interrupt status bit. */
#define MMA_SRC_TRANS_MASK          0x20U
/*! Mask for FIFO interrupt status bit. */
#define MMA_SRC_FIFO_MASK           0x40U
/*! @} */

/*!
 * @name WHO_AM_I register
 * @{
 */
/*! Register address. */
#define MMA_REG_WHO_AM_I            0x0DU
/*! @} */

/*!
 * @name XYZ_DATA_CFG register
 * @{
 */
/*! Register address. */
#define MMA_REG_XYZ_DATA_CFG        0x0EU

/*! Mask for output buffer data format full scale. */
#define MMA_FS_MASK                 0x03U
/*! Mask for enable high-pass output data. */
#define MMA_HPF_OUT_MASK            0x10U

/*! Shift for output buffer data format full scale. */
#define MMA_FS_SHIFT                0x00U
/*! Shift for enable high-pass output data. */
#define MMA_HPF_OUT_SHIFT           0x4U

/*! Enabling high-pass output data. */
#define MMA_EN_HPF_OUT              (1U << MMA_HPF_OUT_SHIFT)

/*! Full-scale value range is 2 g. */
#define MMA_FS_2G                   (0U << MMA_FS_SHIFT)
/*! Full-scale value range is 4 g. */
#define MMA_FS_4G                   (1U << MMA_FS_SHIFT)
/*! Full-scale value range is 8 g. */
#define MMA_FS_8G                   (2U << MMA_FS_SHIFT)
/*! @} */

/*!
 * @name TRANSIENT_CFG register
 * @{
 */
/*! Register address. */
#define MMA_REG_TRANSIENT_CFG       0x1DU

/*! Mask for transient event flags that are latched into the TRANSIENT_SRC register. */
#define MMA_ELE_MASK                0x10U
/*! Mask for event flag enable on Z transient acceleration greater than transient
 * threshold event. */
#define MMA_ZTEFE_MASK              0x08U
/*! Mask for event flag enable on Y transient acceleration greater than transient
 * Mask for threshold event. */
#define MMA_YTEFE_MASK              0x04U
/*! Mask for event flag enable on X transient acceleration greater than transient
 * threshold event. */
#define MMA_XTEFE_MASK              0x02U
/*! Mask for bypass high-pass filter. */
#define MMA_HPF_BYP_MASK            0x01U

/*! Shift for transient event flags that are latched into the TRANSIENT_SRC register. */
#define MMA_ELE_SHIFT               0x04U
/*! Shift for event flag enable on Z transient acceleration greater than transient
 * threshold event. */
#define MMA_ZTEFE_SHIFT             0x03U
/*! Shift for event flag enable on Y transient acceleration greater than transient
 * threshold event. */
#define MMA_YTEFE_SHIFT             0x02U
/*! Shift for event flag enable on X transient acceleration greater than transient
 * threshold event. */
#define MMA_XTEFE_SHIFT             0x01U

/*! Disable event flag latch. */
#define MMA_DIS_ELE                 (0U << MMA_ELE_SHIFT)
/*! Enable event flag latch. */
#define MMA_EN_ELE                  (1U << MMA_ELE_SHIFT)

/*! Enable event flag on Z transient acceleration. */
#define MMA_EN_ZTEFE                (1U << MMA_ZTEFE_SHIFT)
/*! Disable event flag on Z transient acceleration. */
#define MMA_DIS_ZTEFE               (0U << MMA_ZTEFE_SHIFT)

/*! Enable event flag on Y transient acceleration. */
#define MMA_EN_YTEFE                (1U << MMA_YTEFE_SHIFT)
/*! Disable event flag on Y transient acceleration. */
#define MMA_DIS_YTEFE               (0U << MMA_YTEFE_SHIFT)

/*! Enable event flag on X transient acceleration. */
#define MMA_EN_XTEFE                (1U << MMA_XTEFE_SHIFT)
/*! Disable event flag on X transient acceleration. */
#define MMA_DIS_XTEFE               (0U << MMA_XTEFE_SHIFT)
/*! @} */

/*!
 * @name TRANSIENT_SRC register
 * @{
 */
/*! Register address. */
#define MMA_REG_TRANSIENT_SRC       0x1EU

/*! Mask for polarity of X-transient event that triggered interrupt. */
#define MMA_X_TRANS_POL_MASK        0x01U
/*! Mask for X-transient event. */
#define MMA_XTRANSE_MASK            0x02U
/*! Mask for polarity of Y-transient event that triggered interrupt. */
#define MMA_Y_TRANS_POL_MASK        0x04U
/*! Mask for Y-transient event. */
#define MMA_YTRANSE_MASK            0x08U
/*! Mask for polarity of Z-transient event that triggered interrupt. */
#define MMA_Z_TRANS_POL_MASK        0x10U
/*! Mask for Z-transient event. */
#define MMA_ZTRANSE_MASK            0x20U
/*! Mask for event active flag. */
#define MMA_EA_MASK                 0x40U

/*! Shift for polarity of X-transient event that triggered interrupt. */
#define MMA_X_TRANS_POL_SHIFT       0x00U
/*! Shift for X-transient event. */
#define MMA_XTRANSE_SHIFT           0x01U
/*! Shift for polarity of Y-transient event that triggered interrupt. */
#define MMA_Y_TRANS_POL_SHIFT       0x02U
/*! Shift for Y-transient event. */
#define MMA_YTRANSE_SHIFT           0x03U
/*! Shift for polarity of Z-transient event that triggered interrupt. */
#define MMA_Z_TRANS_POL_SHIFT       0x04U
/*! Shift for Z-transient event. */
#define MMA_ZTRANSE_SHIFT           0x05U
/*! Shift for event active flag. */
#define MMA_EA_SHIFT                0x06U

/*!
 * @brief This macro returns true when polarity of a transient event is
 * positive. Otherwise returns false.
 *
 * @param reg   Value of the TRANSIENT_SRC register.
 * @param mask  Bit mask value used to check a polarity bit.
 */
#define MMA_IS_TRANS_POL_POS(reg, mask) \
    (((reg) && (mask)) == 0U)
/*! @} */

/*!
 * @name TRANSIENT_THS register
 * @{
 */
/*! Register address. */
#define MMA_REG_TRANSIENT_THS       0x1FU

/*! Mask for transient threshold. */
#define MMA_THS_VAL_MASK            0x7FU
/*! Mask for debounce counter mode selection. */
#define MMA_DBCNTM_MASK             0x80U

/*! Shift for debounce counter mode selection. */
#define MMA_THS_VAL_SHIFT           0x00U

/*!
 * @ brief This macro converts threshold in [mg] to a register value.
 *
 * @param thsmg Threshold in mg. Admissible range is <0; 8000> mg.
 */
#define MMA_GET_THS_REG(thsmg) \
    ((thsmg) / 63U)
/*! @} */

/*!
 * @name TRANSIENT_COUNT register
 * @{
 */
/*! Register address. */
#define MMA_REG_TRANSIENT_COUNT     0x20U

/*! Mask for count value. */
#define MMA_COUNT_VAL_MASK          0xFFU

/*! Shift for count value. */
#define MMA_COUNT_VAL_SHIFT         0x00U
/*! @} */

/*!
 * @name CTRL_REG1 register
 * @{
 */
/*! Register address. */
#define MMA_REG_CTRL_REG1           0x2AU

/*! Mask for mode selection. */
#define MMA_ACTIVE_MASK             0x01U
/*! Mask for fast read mode. */
#define MMA_F_READ_MASK             0x02U
/*! Mask for Data-rate selection. */
#define MMA_DRx_MASK                0x38U

/*! Shift for mode selection. */
#define MMA_ACTIVE_SHIFT            0x00U
/*! Shift for fast read mode. */
#define MMA_F_READ_SHIFT            0x01U
/*! Shift for Data-rate selection. */
#define MMA_DRx_SHIFT               0x03U

/*! Enabling active mode. */
#define MMA_SET_ACTIVE              (1U << MMA_ACTIVE_SHIFT)
/*! Enabling standby mode. */
#define MMA_SET_STANDBY             (0U << MMA_ACTIVE_SHIFT)

/*! Enable fast read mode (8-bit resolution of measured data). */
#define MMA_EN_F_READ               (1U << MMA_F_READ_SHIFT)
/*! Disable fast read mode (14-bit resolution of measured data). */
#define MMA_DIS_F_READ              (0U << MMA_F_READ_SHIFT)

/*! System output data rate is 800 Hz. */
#define MMA_SET_DR_800_HZ           (0x00U << MMA_DRx_SHIFT)
/*! System output data rate is 400 Hz. */
#define MMA_SET_DR_400_HZ           (0x01U << MMA_DRx_SHIFT)
/*! System output data rate is 200 Hz. */
#define MMA_SET_DR_200_HZ           (0x02U << MMA_DRx_SHIFT)
/*! System output data rate is 100 Hz. */
#define MMA_SET_DR_100_HZ           (0x03U << MMA_DRx_SHIFT)
/*! System output data rate is 50 Hz. */
#define MMA_SET_DR_50_HZ            (0x04U << MMA_DRx_SHIFT)
/*! System output data rate is 12.5 Hz. */
#define MMA_SET_DR_12_HZ            (0x05U << MMA_DRx_SHIFT)
/*! System output data rate is 6.25 Hz. */
#define MMA_SET_DR_6_HZ             (0x06U << MMA_DRx_SHIFT)
/*! System output data rate is 1.56 Hz. */
#define MMA_SET_DR_1_HZ             (0x07U << MMA_DRx_SHIFT)
/*! @} */

/*!
 * @name CTRL_REG2 register
 * @{
 */
/*! Register address. */
#define MMA_REG_CTRL_REG2           0x2BU

/*! Software reset mask. */
#define MMA_RST_MASK                0x40U
/*! @} */

/*!
 * @name CTRL_REG4 register
 * @{
 */
/*! Register address. */
#define MMA_REG_CTRL_REG4           0x2DU

/*! Mask for transient interrupt enable. */
#define MMA_INT_EN_TRANS_MASK       0x20U
/*! Mask for FIFO enable. */
#define MMA_INT_EN_FIFO_MASK        0x40U

/*! Shift for transient interrupt enable. */
#define MMA_INT_EN_TRANS_SHIFT      0x05U
/*! Shift for FIFO enable. */
#define MMA_INT_EN_FIFO_SHIFT       0x06U

/*! Enables transient interrupt. */
#define MMA_EN_INT_EN_TRANS         (1U << MMA_INT_EN_TRANS_SHIFT)
/*! Enables FIFO interrupt. */
#define MMA_EN_INT_EN_FIFO          (1U << MMA_INT_EN_FIFO_SHIFT)
/*! @} */

/*!
 * @name CTRL_REG5 register
 * @{
 */
/*! Register address. */
#define MMA_REG_CTRL_REG5           0x2EU

/*! Mask for INT1/INT2 configuration for transient detection. */
#define MMA_INT_CFG_TRANS_MASK      0x20U
/*! Mask for INT1/INT2 configuration for FIFO. */
#define MMA_INT_CFG_FIFO_MASK       0x40U

/*! Shift for INT1/INT2 configuration for transient detection. */
#define MMA_INT_CFG_TRANS_SHIFT     0x05U
/*! Shift for INT1/INT2 configuration for FIFO. */
#define MMA_INT_CFG_FIFO_SHIFT      0x06U

/*! Use INT1 pin for the transient function. */
#define MMA_ROUTE_INT1_TRANS        (1U << MMA_INT_CFG_TRANS_SHIFT)
/*! Use INT2 pin for the transient function. */
#define MMA_ROUTE_INT2_TRANS        (0U << MMA_INT_CFG_TRANS_SHIFT)

/*! Use INT1 pin for the FIFO function. */
#define MMA_ROUTE_INT1_FIFO         (1U << MMA_INT_CFG_FIFO_SHIFT)
/*! Use INT2 pin for the FIFO function. */
#define MMA_ROUTE_INT2_FIFO         (0U << MMA_INT_CFG_FIFO_SHIFT)
/*! @} */

#endif /* SOURCE_MMA_MMA8451Q_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
