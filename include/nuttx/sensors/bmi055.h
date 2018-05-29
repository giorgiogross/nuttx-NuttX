/****************************************************************************
 * include/nuttx/sensors/bmi055.h
 *
 *   Copyright (C) 2018 Giorgio Gross. All rights reserved.
 *   Author: Giorgio Gross <giorgio.gross@robodev.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_BMI055_H
#define __INCLUDE_NUTTX_SENSORS_BMI055_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMI055)

#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI055_ACCEL_I2C_ADDR 0x18
#define BMI055_GYRO_I2C_ADDR  0x68

/* IOCTL Commands ***********************************************************/
/**************************************************************************
 * Acceleration functions following:
 **************************************************************************/

/* BMI055_ACCEL_READ_ACCEL_X
 *                 - This API reads acceleration data X values
 *                   from location 02h and 03h
 *
 *   ioctl-argument: accel_x_s16 : pointer holding the data of accel X
 *                   value       |   resolution
 *                   ----------------- | --------------
 *                   0          | BMA2x2_12_RESOLUTION
 *                   1          | BMA2x2_10_RESOLUTION
 *                   2          | BMA2x2_14_RESOLUTION
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_X 1

/* BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_X
 *                 - This API reads acceleration data X values
 *                   from location 02h and 03h bit resolution support 8bit
 *
 *   ioctl-argument: accel_x_s8 : pointer holding the data of accel X
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_X 2

/* BMI055_ACCEL_READ_ACCEL_Y
 *                 - This API reads acceleration data Y values
 *                   from location 04h and 05h
 *
 *   ioctl-argument: accel_y_s16 : pointer holding the data of accel Y
 *                   value       |   resolution
 *                   ----------------- | --------------
 *                   0          | BMA2x2_12_RESOLUTION
 *                   1          | BMA2x2_10_RESOLUTION
 *                   2          | BMA2x2_14_RESOLUTION
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_Y 3

/* BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_Y
 *                 - This API reads acceleration data Y values of
 *                   8bit  resolution  from location 05h
 *
 *   ioctl-argument: accel_y_s8   The data of y
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_Y 4

/* BMI055_ACCEL_READ_ACCEL_Z
 *                 - This API reads acceleration data Z values
 *                   from location 06h and 07h
 *
 *   ioctl-argument: accel_z_s16 : pointer holding the data of accel Z
 *                   value       |   resolution
 *                   ----------------- | --------------
 *                   0          | BMA2x2_12_RESOLUTION
 *                   1          | BMA2x2_10_RESOLUTION
 *                   2          | BMA2x2_14_RESOLUTION
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_Z 5

/* BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_Z
 *                 - This API reads acceleration data Z values of
 *                   8bit  resolution  from location 07h
 *   ioctl-argument: accel_z_s8 : the data of z
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_Z 6

/* BMI055_ACCEL_READ_ACCEL_XYZ
 *                 - This API reads acceleration data X,Y,Z values
 *                   from location 02h to 07h
 *
 *   ioctl-argument: accel : pointer holding the data of accel
 *                   value       |   resolution
 *                   ----------------- | --------------
 *                   0          | BMA2x2_12_RESOLUTION
 *                   1          | BMA2x2_10_RESOLUTION
 *                   2          | BMA2x2_14_RESOLUTION
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_XYZ 7

/* BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_XYZ
 *                 - This API reads acceleration of 8 bit resolution
 *                   data of X,Y,Z values
 *                   from location 03h , 05h and 07h
 *
 *   ioctl-argument: accel : pointer holding the data of accel
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_XYZ 8

/* BMI055_ACCEL_GET_INTR_TAP_STAT
 *                 - This API read tap-sign, tap-first-xyz
 *                   slope-sign, slope-first-xyz status register byte
 *                   from location 0Bh
 *
 *   ioctl-argument: stat_tap_u8 : The status of tap and slope
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_TAP_STAT 9

/* BMI055_ACCEL_GET_INTR_ORIENT_STAT
 *                 - This API read orient, high-sign and high-first-xyz
 *                   status register byte from location 0Ch
 *
 *   ioctl-argument: stat_orient_u8 : The status of orient and high
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_ORIENT_STAT 10

/* BMI055_ACCEL_GET_FIFO_STAT
 *                 - This API reads fifo overrun and fifo frame counter
 *                   status register byte  from location 0Eh
 *
 *   ioctl-argument: stat_fifo_u8 : The status of fifo overrun and frame counter
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FIFO_STAT 11

/* BMI055_ACCEL_GET_FIFO_FRAME_COUNT
 *                 - This API read fifo frame count
 *                   from location 0Eh bit position 0 to 6
 *
 *   ioctl-argument: frame_count_u8 : The status of fifo frame count
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FIFO_FRAME_COUNT 12

/* BMI055_ACCEL_GET_FIFO_OVERRUN
 *                 - This API read fifo overrun
 *                   from location 0Eh bit position 7
 *
 *   ioctl-argument: fifo_overrun_u8 : The status of fifo overrun
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FIFO_OVERRUN 13

/* BMI055_ACCEL_GET_INTR_STAT
 *                 - This API read interrupt status of flat, orient, single tap,
 *                   double tap, slow no motion, slope, highg and lowg from location 09h
 *
 *   ioctl-argument: intr_stat_u8 : The value of interrupt status
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_STAT 14

/* BMI055_ACCEL_GET_RANGE
 *                 - This API is used to get the ranges(g values) of the sensor
 *                   in the register 0x0F bit from 0 to 3
 *
 *   ioctl-argument: range_u8 : The value of range
 *                   range_u8       |   result
 *                   ----------------- | --------------
 *                   0x03       | BMA2x2_RANGE_2G
 *                   0x05       | BMA2x2_RANGE_4G
 *                   0x08       | BMA2x2_RANGE_8G
 *                   0x0C       | BMA2x2_RANGE_16G
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_RANGE 15

/* BMI055_ACCEL_SET_RANGE
 *                 - This API is used to set the ranges(g values) of the sensor
 *                   in the register 0x0F bit from 0 to 3
 *
 *   ioctl-argument: range_u8 : The value of range
 *                   range_u8       |   result
 *                   ----------------- | --------------
 *                   0x03       | BMA2x2_RANGE_2G
 *                   0x05       | BMA2x2_RANGE_4G
 *                   0x08       | BMA2x2_RANGE_8G
 *                   0x0C       | BMA2x2_RANGE_16G
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_RANGE 16

/* BMI055_ACCEL_GET_BW
 *                 - This API is used to get the bandwidth of the sensor in the register
 *                   0x10 bit from 0 to 4
 *
 *   ioctl-argument: bw_u8 : The value of bandwidth
 *                   bw_u8          |   result
 *                   ----------------- | --------------
 *                   0x08       | BMA2x2_BW_7_81HZ
 *                   0x09       | BMA2x2_BW_15_63HZ
 *                   0x0A       | BMA2x2_BW_31_25HZ
 *                   0x0B       | BMA2x2_BW_62_50HZ
 *                   0x0C       | BMA2x2_BW_125HZ
 *                   0x0D       | BMA2x2_BW_250HZ
 *                   0x0E       | BMA2x2_BW_500HZ
 *                   0x0F       | BMA2x2_BW_1000HZ
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_BW 17

/* BMI055_ACCEL_SET_BW
 *                 - This API is used to set the bandwidth of the sensor in the register
 *                   0x10 bit from 0 to 4
 *
 *   ioctl-argument: bw_u8 : The value of bandwidth
 *                   bw_u8          |   result
 *                   ----------------- | --------------
 *                   0x08       | BMA2x2_BW_7_81HZ
 *                   0x09       | BMA2x2_BW_15_63HZ
 *                   0x0A       | BMA2x2_BW_31_25HZ
 *                   0x0B       | BMA2x2_BW_62_50HZ
 *                   0x0C       | BMA2x2_BW_125HZ
 *                   0x0D       | BMA2x2_BW_250HZ
 *                   0x0E       | BMA2x2_BW_500HZ
 *                   0x0F       | BMA2x2_BW_1000HZ
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_BW 18

/* BMI055_ACCEL_GET_POWER_MODE
 *                 - This API is used to get the operating
 *                   modes of the sensor in the register 0x11 and 0x12
 *
 *                   NOTE: Register 0x11 - bit from 5 to 7
 *
 *                   NOTE: Register 0x12 - bit from 5 and 6
 *
 *   ioctl-argument: power_mode_u8 : The value of power mode
 *                   power_mode_u8           |value  |   0x11  |   0x12
 *                   ------------------------- |-------| --------|--------
 *                   BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *                   BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *                   BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *                   BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *                   BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *                   BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_POWER_MODE 19

/* BMI055_ACCEL_SET_POWER_MODE
 *                 - This API is used to set the operating
 *                   modes of the sensor in the register 0x11 and 0x12
 *
 *                   NOTE: Register 0x11 - bit from 5 to 7
 *
 *                   NOTE: Register 0x12 - bit from 5 and 6
 *
 *   ioctl-argument: power_mode_u8 : The value of power mode
 *                   power_mode_u8           |value  |   0x11  |   0x12
 *                   ------------------------- |-------| --------|--------
 *                   BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *                   BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *                   BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *                   BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *                   BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *                   BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_POWER_MODE 20

/* BMI055_ACCEL_SET_MODE_VALUE
 *                 - This API is used to assign the power mode values
 *                   modes of the sensor in the register 0x11 and 0x12
 *
 *                   NOTE: Register 0x11 - bit from 5 to 7
 *
 *                   NOTE: Register 0x12 - bit from 5 and 6
 *
 *   ioctl-argument: power_mode_u8 : The value of power mode
 *                   power_mode_u8           |value  |   0x11  |   0x12
 *                   ------------------------- |-------| --------|--------
 *                   BMA2x2_MODE_NORMAL        |  0    |  0x00   |  0x00
 *                   BMA2x2_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *                   BMA2x2_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *                   BMA2x2_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *                   BMA2x2_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *                   BMA2x2_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_MODE_VALUE 21

/* BMI055_ACCEL_GET_SLEEP_DURN
 *                 - This API is used to get
 *                   the sleep duration of the sensor in the register 0x11
 *                   Register 0x11 - bit from 0 to 3
 *
 *   ioctl-argument: sleep_durn_u8 : The value of sleep duration time
 *                   sleep_durn_u8  |   result
 *                   ----------------- | ----------------------
 *                   0x05       | BMA2x2_SLEEP_DURN_0_5MS
 *                   0x06       | BMA2x2_SLEEP_DURN_1MS
 *                   0x07       | BMA2x2_SLEEP_DURN_2MS
 *                   0x08       | BMA2x2_SLEEP_DURN_4MS
 *                   0x09       | BMA2x2_SLEEP_DURN_6MS
 *                   0x0A       | BMA2x2_SLEEP_DURN_10MS
 *                   0x0B       | BMA2x2_SLEEP_DURN_25MS
 *                   0x0C       | BMA2x2_SLEEP_DURN_50MS
 *                   0x0D       | BMA2x2_SLEEP_DURN_100MS
 *                   0x0E       | BMA2x2_SLEEP_DURN_500MS
 *                   0x0F       | BMA2x2_SLEEP_DURN_1S
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SLEEP_DURN 22

/* BMI055_ACCEL_SET_SLEEP_DURN
 *                 - This API is used to set
 *                   the sleep duration of the sensor in the register 0x11
 *                   Register 0x11 - bit from 0 to 3
 *
 *   ioctl-argument: sleep_durn_u8 : The value of sleep duration time
 *                   sleep_durn_u8  |   result
 *                   ----------------- | ----------------------
 *                   0x05       | BMA2x2_SLEEP_DURN_0_5MS
 *                   0x06       | BMA2x2_SLEEP_DURN_1MS
 *                   0x07       | BMA2x2_SLEEP_DURN_2MS
 *                   0x08       | BMA2x2_SLEEP_DURN_4MS
 *                   0x09       | BMA2x2_SLEEP_DURN_6MS
 *                   0x0A       | BMA2x2_SLEEP_DURN_10MS
 *                   0x0B       | BMA2x2_SLEEP_DURN_25MS
 *                   0x0C       | BMA2x2_SLEEP_DURN_50MS
 *                   0x0D       | BMA2x2_SLEEP_DURN_100MS
 *                   0x0E       | BMA2x2_SLEEP_DURN_500MS
 *                   0x0F       | BMA2x2_SLEEP_DURN_1S
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SLEEP_DURN 23

/* BMI055_ACCEL_GET_SLEEP_TIMER_MODE
 *                 - This API is used to get the sleep timer mode
 *                   in the register 0x12 bit 5
 *
 *   ioctl-argument: sleep_timer_u8 : The value of sleep timer mode
 *                   sleep_timer_u8 |   result
 *                   ----------------- | ----------------------
 *                   0          | enable EventDrivenSampling(EDT)
 *                   1          | enable Equidistant sampling mode(EST)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SLEEP_TIMER_MODE 24

/* BMI055_ACCEL_SET_SLEEP_TIMER_MODE
 *                 - This API is used to set the sleep timer mode
 *                   in the register 0x12 bit 5
 *
 *   ioctl-argument: sleep_timer_u8 : The value of sleep timer mode
 *                   sleep_timer_u8 |   result
 *                   ----------------- | ----------------------
 *                   0          | enable EventDrivenSampling(EDT)
 *                   1          | enable Equidistant sampling mode(EST)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SLEEP_TIMER_MODE 25

/* BMI055_ACCEL_GET_HIGH_BW
 *                 - This API is used to get high bandwidth
 *                   in the register 0x13 bit 7
 *
 *   ioctl-argument: high_bw_u8 : The value of high bandwidth
 *                   high_bw_u8    |   result
 *                   ----------------- | ----------------------
 *                   0          | Unfiltered High Bandwidth
 *                   1          | Filtered Low Bandwidth
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_HIGH_BW 26

/* BMI055_ACCEL_SET_HIGH_BW
 *                 - This API is used to write high bandwidth
 *                   in the register 0x13 bit 7
 *
 *   ioctl-argument: high_bw_u8 : The value of high bandwidth
 *                   high_bw_u8    |   result
 *                   ----------------- | ----------------------
 *                   0          | Unfiltered High Bandwidth
 *                   1          | Filtered Low Bandwidth
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_HIGH_BW 27

/* BMI055_ACCEL_GET_SHADOW_DIS
 *                 - This API is used to get shadow dis
 *                   in the register 0x13 bit 6
 *
 *   ioctl-argument: shadow_dis_u8 : The value of shadow dis
 *                   shadow_dis_u8  |   result
 *                   ----------------- | ------------------
 *                   0          | MSB is Locked
 *                   1          | No MSB Locking
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SHADOW_DIS 28

/* BMI055_ACCEL_SET_SHADOW_DIS
 *                 - This API is used to set shadow dis
 *                   in the register 0x13 bit 6
 *
 *   ioctl-argument: shadow_dis_u8 : The value of shadow dis
 *                   shadow_dis_u8  |   result
 *                   ----------------- | ------------------
 *                   0          | MSB is Locked
 *                   1          | No MSB Locking
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SHADOW_DIS 29

/* BMI055_ACCEL_SOFT_RST
 *                 - This function is used for the soft reset
 *                   The soft reset register will be written
 *                   with 0xB6 in the register 0x14.
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SOFT_RST 30

/* BMI055_ACCEL_UPDATE_IMAGE
 *                 - This API is used to update the register values
 *
 *   ioctl-argument: : None
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_UPDATE_IMAGE 31

/* BMI055_ACCEL_GET_INTR_ENABLE
 *                 - This API is used to get
 *                   interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *
 *                   NOTE: It reads the flat enable, orient enable,
 *
 *                   NOTE: single tap enable, double tap enable
 *
 *                   NOTE: slope-x enable, slope-y enable, slope-z enable,
 *
 *                   NOTE: fifo watermark enable,
 *
 *                   NOTE: fifo full enable, data enable, low-g enable,
 *
 *                   NOTE: high-z enable, high-y enable
 *
 *                   NOTE: high-z enable
 *
 *   ioctl-argument: intr_type_u8: The value of interrupts
 *                   intr_type_u8   |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_LOW_G_INTR
 *                   1          | BMA2x2_HIGH_G_X_INTR
 *                   2          | BMA2x2_HIGH_G_Y_INTR
 *                   3          | BMA2x2_HIGH_G_Z_INTR
 *                   4          | BMA2x2_DATA_ENABLE
 *                   5          | SLOPE_X_INTR
 *                   6          | SLOPE_Y_INTR
 *                   7          | SLOPE_Z_INTR
 *                   8          | SINGLE_TAP_INTR
 *                   9          | SINGLE_TAP_INTR
 *                   10         | ORIENT_INT
 *                   11         | FLAT_INT
 *
 *                   value_u8 : The value of interrupts enable
 *                   value_u8       |   result
 *                   ----------------- | ------------------
 *                   0x00       | INTR_DISABLE
 *                   0x01       | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_ENABLE 32

/* BMI055_ACCEL_SET_INTR_ENABLE
 *                 - This API is used to set
 *                   interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *
 *                   NOTE: It reads the flat enable, orient enable,
 *
 *                   NOTE: single tap enable, double tap enable
 *
 *                   NOTE: slope-x enable, slope-y enable, slope-z enable,
 *
 *                   NOTE: fifo watermark enable,
 *
 *                   NOTE: fifo full enable, data enable, low-g enable,
 *
 *                   NOTE: high-z enable, high-y enable
 *
 *                   NOTE: high-z enable
 *
 *   ioctl-argument: intr_type_u8: The value of interrupts
 *                   intr_type_u8   |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_LOW_G_INTR
 *                   1          | BMA2x2_HIGH_G_X_INTR
 *                   2          | BMA2x2_HIGH_G_Y_INTR
 *                   3          | BMA2x2_HIGH_G_Z_INTR
 *                   4          | BMA2x2_DATA_ENABLE
 *                   5          | SLOPE_X_INTR
 *                   6          | SLOPE_Y_INTR
 *                   7          | SLOPE_Z_INTR
 *                   8          | SINGLE_TAP_INTR
 *                   9          | SINGLE_TAP_INTR
 *                   10         | ORIENT_INT
 *                   11         | FLAT_INT
 *
 *                   value_u8 : The value of interrupts enable
 *                   value_u8       |   result
 *                   ----------------- | ------------------
 *                   0x00       | INTR_DISABLE
 *                   0x01       | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_ENABLE 33

/* BMI055_ACCEL_GET_INTR_FIFO_FULL
 *                 - This API is used to get
 *                   the interrupt fifo full enable interrupt status
 *                   in the register 0x17 bit 5
 *
 *   ioctl-argument: fifo_full_u8 The value of fifo full interrupt enable
 *                   fifo_full_u8   |   result
 *                   ----------------- | ------------------
 *                   0x00       | INTR_DISABLE
 *                   0x01       | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_FIFO_FULL 34

/* BMI055_ACCEL_SET_INTR_FIFO_FULL
 *                 - This API is used to set
 *                   the interrupt fifo full enable interrupt status
 *                   in the register 0x17 bit 5
 *
 *   ioctl-argument: fifo_full_u8 The value of fifo full interrupt enable
 *                   fifo_full_u8   |   result
 *                   ----------------- | ------------------
 *                   0x00       | INTR_DISABLE
 *                   0x01       | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_FIFO_FULL 35

/* BMI055_ACCEL_GET_INTR_FIFO_WM
 *                 - This API is used to get
 *                   the interrupt fifo watermark enable interrupt status
 *                   in the register 0x17 bit 6
 *
 *   ioctl-argument: fifo_wm_u8 : the value FIFO Water Mark
 *                   fifo_wm_u8     |   result
 *                   ----------------- | ------------------
 *                   0x00       | INTR_DISABLE
 *                   0x01       | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_FIFO_WM 36

/* BMI055_ACCEL_SET_INTR_FIFO_WM
 *                 - This API is used to set
 *                   the interrupt fifo watermark enable interrupt status
 *                   in the register 0x17 bit 6
 *
 *   ioctl-argument: fifo_wm_u8 : the value FIFO Water Mark
 *                   fifo_wm_u8     |   result
 *                   ----------------- | ------------------
 *                   0x00       | INTR_DISABLE
 *                   0x01       | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_FIFO_WM 37

/* BMI055_ACCEL_GET_SLOW_NO_MOTION
 *                 - This API is used to get
 *                   the interrupt status of slow/no motion select and slow no motion
 *                   enable xyz interrupt in the register 0x18 bit from 0 to 3
 *
 *   ioctl-argument: channel_u8 : The value of slow/no motion select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_X
 *                   1          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Y
 *                   2          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Z
 *                   3          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_SEL
 *
 *                   slow_no_motion_u8 : The value of slow no motion interrupt
 *                   enable
 *                   slow_no_motion_u8     |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SLOW_NO_MOTION 38

/* BMI055_ACCEL_SET_SLOW_NO_MOTION
 *                 - This API is used to set
 *                   the interrupt status of slow/no motion select and slow no motion
 *                   enable xyz interrupt in the register 0x18 bit from 0 to 3
 *
 *   ioctl-argument: channel_u8 : The value of slow/no motion select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_X
 *                   1          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Y
 *                   2          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Z
 *                   3          | BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_SEL
 *
 *                   slow_no_motion_u8 : The value of slow no motion interrupt enable
 *                   slow_no_motion_u8     |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SLOW_NO_MOTION 39

/* BMI055_ACCEL_GET_INTR_LOW_G
 *                 - This API is used to get
 *                   the interrupt enable of low_g interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_Low_g -> register 0x19 bit 0
 *                   NOTE: INTR2_Low_g -> register 0x1B bit 0
 *
 *   ioctl-argument: channel_u8 : The value of low interrupt selection channel
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_LOW_G
 *                   1          | BMA2x2_ACCEL_INTR2_LOW_G
 *
 *                   intr_low_g_u8 : the value of low_g interrupt
 *                   intr_low_u8           |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_LOW_G 40

/* BMI055_ACCEL_SET_INTR_LOW_G
 *                 - This API is used to set
 *                   the interrupt enable of low_g interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_Low_g -> register 0x19 bit 0
 *                   NOTE: INTR2_Low_g -> register 0x1B bit 0
 *
 *   ioctl-argument: channel_u8 : The value of low interrupt selection channel
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_LOW_G
 *                   1          | BMA2x2_ACCEL_INTR2_LOW_G
 *
 *                   intr_low_u8 : the value of low_g interrupt
 *                   intr_low_u8           |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_LOW_G 41

/* BMI055_ACCEL_GET_INTR_HIGH_G
 *                 - This API is used to get
 *                   the interrupt enable of high_g interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_high_g -> register 0x19 bit 1
 *                   NOTE: INTR2_high_g -> register 0x1B bit 1
 *
 *   ioctl-argument: channel_u8: The value of high_g interrupt selection
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_HIGH_G
 *                   1          | BMA2x2_ACCEL_INTR2_HIGH_G
 *
 *                   intr_high_g_u8 : the value of high_g interrupt
 *                   intr_high_g_u8        |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_HIGH_G 42

/* BMI055_ACCEL_SET_INTR_HIGH_G
 *                 - This API is used to set
 *                   the interrupt enable of high_g interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_high_g -> register 0x19 bit 1
 *                   NOTE: INTR2_high_g -> register 0x1B bit 1
 *
 *   ioctl-argument: channel_u8: The value of high_g interrupt selection
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_HIGH_G
 *                   1          | BMA2x2_ACCEL_INTR2_HIGH_G
 *
 *                   intr_high_g_u8 : the value of high_g interrupt
 *                   intr_high_g_u8        |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_HIGH_G 43

/* BMI055_ACCEL_GET_INTR_SLOPE
 *                 - This API is used to get
 *                   the interrupt enable of slope interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_slope -> register 0x19 bit 2
 *                   NOTE: INTR2_slope -> register 0x1B bit 2
 *
 *   ioctl-argument: channel_u8: the value of slope channel select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_SLOPE
 *                   1          | BMA2x2_ACCEL_INTR2_SLOPE
 *
 *                   intr_slope_u8 : The slope value enable value
 *                   intr_slope_u8         |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_SLOPE 44

/* BMI055_ACCEL_SET_INTR_SLOPE
 *                 - This API is used to set
 *                   the interrupt enable of slope interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_slope -> register 0x19 bit 2
 *                   NOTE: INTR2_slope -> register 0x1B bit 2
 *
 *   ioctl-argument: channel_u8: the value of slope channel select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_SLOPE
 *                   1          | BMA2x2_ACCEL_INTR2_SLOPE
 *
 *                   intr_slope_u8 : The slope value enable value
 *                   intr_slope_u8         |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_SLOPE 45

/* BMI055_ACCEL_GET_INTR_SLOW_NO_MOTION
 *                 - This API is used to get
 *                   the interrupt enable of slow/no motion interrupt in
 *                   the register 0x19 and 0x1B
 *                   NOTE: INTR1_slow_no_motion -> register 0x19 bit 3
 *                   NOTE: INTR2_slow_no_motion -> register 0x1B bit 3
 *
 *   ioctl-argument: channel_u8 : The value of slow/no motion selection
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_INTR1_SLOW_NO_MOTION
 *                   1          | BMA2x2_INTR2_SLOW_NO_MOTION
 *
 *                   intr_slow_no_motion_u8:  the slow_no_motion enable value
 *                   intr_slow_no_motion_u8 |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_SLOW_NO_MOTION 46

/* BMI055_ACCEL_SET_INTR_SLOW_NO_MOTION
 *                 - This API is used to set
 *                   the interrupt enable of slow/no motion interrupt in
 *                   the register 0x19 and 0x1B
 *                   NOTE: INTR1_slow_no_motion -> register 0x19 bit 3
 *                   NOTE: INTR2_slow_no_motion -> register 0x1B bit 3
 *
 *   ioctl-argument: channel_u8 : The value of slow/no motion selection
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_INTR1_SLOW_NO_MOTION
 *                   1          | BMA2x2_INTR2_SLOW_NO_MOTION
 *
 *                   intr_slow_no_motion_u8:  the slow_no_motion enable value
 *                   intr_slow_no_motion_u8 |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_SLOW_NO_MOTION 47

/* BMI055_ACCEL_GET_INTR_DOUBLE_TAP
 *                 - This API is used to get
 *                   the interrupt enable of double tap interrupt
 *                   in the register 0x19 and 0x1B
 *                   NOTE: INTR1_double -> register 0x19 bit 4
 *                   NOTE: INTR2_double -> register 0x1B bit 4
 *
 *   ioctl-argument: channel_u8: The value of double tap selection
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_DOUBLE_TAP
 *                   1          | BMA2x2_ACCEL_INTR2_DOUBLE_TAP
 *
 *                   intr_double_tap_u8: The double tap interrupt enable value
 *                   intr_double_tap_u8     |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_DOUBLE_TAP 48

/* BMI055_ACCEL_SET_INTR_DOUBLE_TAP
 *                 - This API is used to set
 *                   the interrupt enable of double tap interrupt
 *                   in the register 0x19 and 0x1B
 *                   NOTE: INTR1_double -> register 0x19 bit 4
 *                   NOTE: INTR2_double -> register 0x1B bit 4
 *
 *   ioctl-argument: channel_u8: The value of double tap selection
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_DOUBLE_TAP
 *                   1          | BMA2x2_ACCEL_INTR2_DOUBLE_TAP
 *
 *                   intr_double_tap_u8: The double tap interrupt enable value
 *                   intr_double_tap_u8     |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_DOUBLE_TAP 49

/* BMI055_ACCEL_GET_INTR_SINGLE_TAP
 *                 - This API is used to get
 *                   the interrupt enable of single tap
 *                   interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_single_tap -> register 0x19 bit 5
 *                   NOTE: INTR2_single_tap -> register 0x1B bit 5
 *
 *   ioctl-argument: channel_u8: The value of single tap interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_SINGLE_TAP
 *                   1          | BMA2x2_ACCEL_INTR2_SINGLE_TAP
 *
 *                   intr_single_tap_u8: The single tap interrupt enable value
 *                   intr_single_tap_u8     |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_SINGLE_TAP 50

/* BMI055_ACCEL_SET_INTR_SINGLE_TAP
 *                 - This API is used to set
 *                   the interrupt enable of single tap
 *                   interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_single_tap -> register 0x19 bit 5
 *                   NOTE: INTR2_single_tap -> register 0x1B bit 5
 *
 *   ioctl-argument: channel_u8: The value of single tap interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_SINGLE_TAP
 *                   1          | BMA2x2_ACCEL_INTR2_SINGLE_TAP
 *
 *                   intr_single_tap_u8: The single tap interrupt enable value
 *                   intr_single_tap_u8     |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_SINGLE_TAP 51

/* BMI055_ACCEL_GET_INTR_ORIENT
 *                 - This API is used to get
 *                   the interrupt status of orient interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_orient -> register 0x19 bit 6
 *                   NOTE: INTR2_orient -> register 0x1B bit 6
 *
 *   ioctl-argument: channel_u8: The value of orient interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_ORIENT
 *                   1          | BMA2x2_ACCEL_INTR2_ORIENT
 *
 *                   intr_orient_u8: The value of orient interrupt enable
 *                   intr_orient_u8         |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_ORIENT 52

/* BMI055_ACCEL_SET_INTR_ORIENT
 *                 - This API is used to set
 *                   the interrupt status of orient interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_orient -> register 0x19 bit 6
 *                   NOTE: INTR2_orient -> register 0x1B bit 6
 *
 *   ioctl-argument: channel_u8: The value of orient interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_ORIENT
 *                   1          | BMA2x2_ACCEL_INTR2_ORIENT
 *
 *                   intr_orient_u8: The value of orient interrupt enable
 *                   intr_orient_u8         |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_ORIENT 53

/* BMI055_ACCEL_GET_INTR_FLAT
 *                 - This API is used to get
 *                   the interrupt enable of flat interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_flat -> register 0x19 bit 7
 *                   NOTE: INTR2_flat -> register 0x1B bit 7
 *
 *   ioctl-argument: channel_u8: The value of flat interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_FLAT
 *                   1          | BMA2x2_ACCEL_INTR2_FLAT
 *
 *                   intr_flat_u8: The flat interrupt enable value
 *                   intr_flat_u8           |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_FLAT 54

/* BMI055_ACCEL_SET_INTR_FLAT
 *                 - This API is used to set
 *                   the interrupt enable of flat interrupt in the register 0x19 and 0x1B
 *                   NOTE: INTR1_flat -> register 0x19 bit 7
 *                   NOTE: INTR2_flat -> register 0x1B bit 7
 *
 *   ioctl-argument: channel_u8: The value of flat interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_FLAT
 *                   1          | BMA2x2_ACCEL_INTR2_FLAT
 *
 *                   intr_flat_u8: The flat interrupt enable value
 *                   intr_flat_u8           |   result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_FLAT 55

/* BMI055_ACCEL_GET_NEW_DATA
 *                 - This API is used to get
 *                   the interrupt status of new data in the register 0x19
 *                   NOTE: INTR1_data -> register 0x19 bit 0
 *                   NOTE: INTR2_data -> register 0x19 bit 7
 *
 *   ioctl-argument: channel_u8: The value of new data interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_NEWDATA
 *                   1          | BMA2x2_ACCEL_INTR2_NEWDATA
 *
 *                   intr_newdata_u8: The new data interrupt enable value
 *                   intr_newdata_u8          |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_NEW_DATA 56

/* BMI055_ACCEL_SET_NEW_DATA
 *                 - This API is used to set
 *                   the interrupt status of new data in the register 0x19
 *                   NOTE: INTR1_data -> register 0x19 bit 0
 *                   NOTE: INTR2_data -> register 0x19 bit 7
 *
 *   ioctl-argument: channel_u8: The value of new data interrupt select
 *                   channel_u8     |   result
 *                   ----------------- | ------------------
 *                   0          | BMA2x2_ACCEL_INTR1_NEWDATA
 *                   1          | BMA2x2_ACCEL_INTR2_NEWDATA
 *
 *                   intr_newdata_u8: The new data interrupt enable value
 *                   intr_newdata_u8          |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_NEW_DATA 57

/* BMI055_ACCEL_GET_INTR1_FIFO_WM
 *                 - This API is used to get the fifo watermark interrupt1 data
 *                   in the register 0x1A bit 1
 *
 *   ioctl-argument: intr1_fifo_wm_u8 : The value of interrupt1 FIFO watermark enable
 *                   intr1_fifo_wm_u8       |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR1_FIFO_WM 58

/* BMI055_ACCEL_SET_INTR1_FIFO_WM
 *                 - This API is used to set the fifo watermark interrupt1 data
 *                   in the register 0x1A bit 1
 *
 *   ioctl-argument: intr1_fifo_wm_u8 : The value of interrupt1 FIFO watermark enable
 *                   intr1_fifo_wm_u8       |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR1_FIFO_WM 59

/* BMI055_ACCEL_GET_INTR2_FIFO_WM
 *                 - This API is used to get the fifo watermark interrupt2 data
 *                   in the register 0x1A bit 6
 *
 *   ioctl-argument: intr2_fifo_wm_u8 : The value of interrupt1 FIFO watermark enable
 *                   intr2_fifo_wm_u8       |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR2_FIFO_WM 60

/* BMI055_ACCEL_SET_INTR2_FIFO_WM
 *                 - This API is used to set the fifo watermark interrupt2 data
 *                   in the register 0x1A bit 6
 *
 *   ioctl-argument: intr2_fifo_wm_u8 : The value of interrupt1 FIFO watermark enable
 *                   intr2_fifo_wm_u8       |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR2_FIFO_WM 61

/* BMI055_ACCEL_GET_INTR1_FIFO_FULL
 *                 - This API is used to get
 *                   the fifo full interrupt1 in the register 0x1A bit 2
 *
 *   ioctl-argument: intr1_fifo_full_u8 : The value of fifo full interrupt enable
 *                   intr1_fifo_full_u8     |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR1_FIFO_FULL 62

/* BMI055_ACCEL_SET_INTR1_FIFO_FULL
 *                 - This API is used to set
 *                   the fifo full interrupt1 in the register 0x1A bit 2
 *
 *   ioctl-argument: intr1_fifo_full_u8 : The value of fifo full interrupt enable
 *                   intr1_fifo_full_u8     |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR1_FIFO_FULL 63

/* BMI055_ACCEL_GET_INTR2_FIFO_FULL
 *                 - This API is used to get
 *                   the fifo full interrupt2 in the register 0x1A bit 5
 *
 *   ioctl-argument: intr2_fifo_full_u8 : Thee vale of fifo full enable
 *                   intr2_fifo_full_u8     |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR2_FIFO_FULL 64

/* BMI055_ACCEL_SET_INTR2_FIFO_FULL
 *                 - This API is used to set
 *                   the fifo full interrupt2 in the register 0x1A bit 5
 *
 *   ioctl-argument: intr2_fifo_full_u8 : Thee vale of fifo full enable
 *                   intr2_fifo_full_u8     |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR2_FIFO_FULL 65

/* BMI055_ACCEL_GET_SOURCE
 *                 - This API is used to get
 *                   the source data status of source data,
 *                   source slow no motion, source slope, source high
 *                   and source low in the register 0x1E bit from 0 to 5
 *
 *   ioctl-argument: channel_u8 : The value of source select
 *                   channel_u8     |    result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_SOURCE_LOW_G
 *                   1        | BMA2x2_ACCEL_SOURCE_HIGH_G
 *                   2        | BMA2x2_ACCEL_SOURCE_SLOPE
 *                   3        | BMA2x2_ACCEL_SOURCE_SLOW_NO_MOTION
 *                   4        | BMA2x2_ACCEL_SOURCE_TAP
 *                   5        | BMA2x2_ACCEL_SOURCE_DATA
 *
 *                   intr_source_u8: The source status enable value
 *                   intr_source_u8         |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SOURCE 66

/* BMI055_ACCEL_SET_SOURCE
 *                 - This API is used to set
 *                   the source data status of source data,
 *                   source slow no motion, source slope, source high
 *                   and source low in the register 0x1E bit from 0 to 5
 *
 *   ioctl-argument: channel_u8 : The value of source select
 *                   channel_u8     |    result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_SOURCE_LOW_G
 *                   1        | BMA2x2_ACCEL_SOURCE_HIGH_G
 *                   2        | BMA2x2_ACCEL_SOURCE_SLOPE
 *                   3        | BMA2x2_ACCEL_SOURCE_SLOW_NO_MOTION
 *                   4        | BMA2x2_ACCEL_SOURCE_TAP
 *                   5        | BMA2x2_ACCEL_SOURCE_DATA
 *
 *                   intr_source_u8: The source status enable value
 *                   intr_source_u8         |    result
 *                   ------------------------ | ------------------
 *                   0x00              | INTR_DISABLE
 *                   0x01              | INTR_ENABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SOURCE 67

/* BMI055_ACCEL_GET_INTR_OUTPUT_TYPE
 *                 - This API is used to get
 *                   the interrupt output type in the register 0x20.
 *
 *                   NOTE: INTR1 -> bit 1
 *
 *                   NOTE: INTR2 -> bit 3
 *
 *   ioctl-argument: channel_u8: The value of output type select
 *                   channel_u8     |    result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_INTR1_OUTPUT
 *                   1        | BMA2x2_ACCEL_INTR2_OUTPUT
 *
 *                   intr_output_type_u8: The value of output type select
 *                   intr_source_u8         |    result
 *                   ------------------------ | ------------------
 *                   0x01              | OPEN_DRAIN
 *                   0x00              | PUSS_PULL
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_OUTPUT_TYPE 68

/* BMI055_ACCEL_SET_INTR_OUTPUT_TYPE
 *                 - This API is used to set
 *                   the interrupt output type in the register 0x20.
 *
 *                   NOTE: INTR1 -> bit 1
 *
 *                   NOTE: INTR2 -> bit 3
 *
 *   ioctl-argument: channel_u8: The value of output type select
 *                   channel_u8   |    result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_INTR1_OUTPUT
 *                   1        | BMA2x2_ACCEL_INTR2_OUTPUT
 *
 *                   intr_output_type_u8: The value of output type select
 *                   intr_source_u8         |    result
 *                   ------------------------ | ------------------
 *                   0x01              | OPEN_DRAIN
 *                   0x00              | PUSS_PULL
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_OUTPUT_TYPE 69

/* BMI055_ACCEL_GET_INTR_LEVEL
 *                 - This API is used to get
 *                   Active Level status in the register 0x20
 *
 *                   NOTE: INTR1 -> bit 0
 *
 *                   NOTE: INTR2 -> bit 2
 *
 *   ioctl-argument: channel_u8: The value of Active Level select
 *                   channel_u8     |    result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_INTR1_LEVEL
 *                   1        | BMA2x2_ACCEL_INTR2_LEVEL
 *
 *                   intr_level_u8: The Active Level status enable value
 *                   intr_level_u8          |    result
 *                   ------------------------ | ------------------
 *                   0x01              | ACTIVE_HIGH
 *                   0x00              | ACTIVE_LOW
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_INTR_LEVEL 70

/* BMI055_ACCEL_SET_INTR_LEVEL
 *                 - This API is used to set
 *                   Active Level status in the register 0x20
 *
 *                   NOTE: INTR1 -> bit 0
 *
 *                   NOTE: INTR2 -> bit 2
 *
 *   ioctl-argument: channel_u8: The value of Active Level select
 *                   channel_u8     |    result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_INTR1_LEVEL
 *                   1        | BMA2x2_ACCEL_INTR2_LEVEL
 *
 *                   intr_level_u8: The Active Level status enable value
 *                   intr_level_u8          |    result
 *                   ------------------------ | ------------------
 *                   0x01              | ACTIVE_HIGH
 *                   0x00              | ACTIVE_LOW
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_INTR_LEVEL 71

/* BMI055_ACCEL_RST_INTR
 *                 - This API is used to set
 *                   the reset interrupt in the register 0x21 bit 7
 *
 *   ioctl-argument: rst_intr_u8: The value of reset interrupt
 *                   rst_intr_u8         |  result
 *                   ------------------------ | ------------------
 *                   0x01              | clear any latch interrupt
 *                   0x00              | keep latch interrupt active
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_RST_INTR 72

/* BMI055_ACCEL_GET_LATCH_INTR
 *                 - This API is used to get
 *                   the latch duration in the register 0x21 bit from 0 to 3
 *
 *   ioctl-argument: latch_intr_u8: The value of latch duration
 *                   latch_intr_u8 |  result
 *                   -----------------| ------------------
 *                   0x00     | BMA2x2_LATCH_DURN_NON_LATCH
 *                   0x01     | BMA2x2_LATCH_DURN_250MS
 *                   0x02     | BMA2x2_LATCH_DURN_500MS
 *                   0x03     | BMA2x2_LATCH_DURN_1S
 *                   0x04     | BMA2x2_LATCH_DURN_2S
 *                   0x05     | BMA2x2_LATCH_DURN_4S
 *                   0x06     | BMA2x2_LATCH_DURN_8S
 *                   0x07     | BMA2x2_LATCH_DURN_LATCH
 *                   0x08     | BMA2x2_LATCH_DURN_NON_LATCH1
 *                   0x09     | BMA2x2_LATCH_DURN_250US
 *                   0x0A     | BMA2x2_LATCH_DURN_500US
 *                   0x0B     | BMA2x2_LATCH_DURN_1MS
 *                   0x0C     | BMA2x2_LATCH_DURN_12_5MS
 *                   0x0D     | BMA2x2_LATCH_DURN_25MS
 *                   0x0E     | BMA2x2_LATCH_DURN_50MS
 *                   0x0F     | BMA2x2_LATCH_DURN_LATCH1
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_LATCH_INTR 73

/* BMI055_ACCEL_SET_LATCH_INTR
 *                 - This API is used to set
 *                   the latch duration in the register 0x21 bit from 0 to 3
 *
 *   ioctl-argument: latch_intr_u8: The value of latch duration
 *                   latch_intr_u8 |  result
 *                   -----------------| ------------------
 *                   0x00     | BMA2x2_LATCH_DURN_NON_LATCH
 *                   0x01     | BMA2x2_LATCH_DURN_250MS
 *                   0x02     | BMA2x2_LATCH_DURN_500MS
 *                   0x03     | BMA2x2_LATCH_DURN_1S
 *                   0x04     | BMA2x2_LATCH_DURN_2S
 *                   0x05     | BMA2x2_LATCH_DURN_4S
 *                   0x06     | BMA2x2_LATCH_DURN_8S
 *                   0x07     | BMA2x2_LATCH_DURN_LATCH
 *                   0x08     | BMA2x2_LATCH_DURN_NON_LATCH1
 *                   0x09     | BMA2x2_LATCH_DURN_250US
 *                   0x0A     | BMA2x2_LATCH_DURN_500US
 *                   0x0B     | BMA2x2_LATCH_DURN_1MS
 *                   0x0C     | BMA2x2_LATCH_DURN_12_5MS
 *                   0x0D     | BMA2x2_LATCH_DURN_25MS
 *                   0x0E     | BMA2x2_LATCH_DURN_50MS
 *                   0x0F     | BMA2x2_LATCH_DURN_LATCH1
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_LATCH_INTR 74

/* BMI055_ACCEL_GET_DURN
 *                 - This API is used to get the duration of
 *                   Low, High, Slope and slow no motion interrupts in the registers
 *
 *                   NOTE: LOW_DURN		-> register 0x22 bit form 0 to 7
 *
 *                   NOTE: HIGH_DURN		-> register 0x25 bit form 0 to 7
 *
 *                   NOTE: SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *
 *                   NOTE: SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *   ioctl-argument: channel_u8: The value of duration select
 *                   channel_u8   | result
 *                   -----------------| ------------------
 *                   0    | BMA2x2_ACCEL_LOW_DURN
 *                   1    | BMA2x2_ACCEL_HIGH_DURN
 *                   2    | BMA2x2_ACCEL_SLOPE_DURN
 *                   3    | BMA2x2_ACCEL_SLOW_NO_MOTION_DURN
 *
 *                   durn_u8: The value of duration
 *                   @note :
 *                   Duration           |    result
 *                   -----------------------| ------------------
 *                   BMA2x2_ACCEL_LOW_DURN  | Low-g interrupt trigger
 *                   -              | delay according to([durn_u8 +1]*2)ms
 *                   -              | range from 2ms to 512ms. default is 20ms
 *                   BMA2x2_ACCEL_HIGH_DURN | high-g interrupt trigger
 *                   -              | delay according to([durn_u8 +1]*2)ms
 *                   -              | range from 2ms to 512ms. default is 32ms
 *                   BMA2x2_ACCEL_SLOPE_DURN| slope interrupt trigger
 *                   -              | if[durn_u8<1:0>+1] consecutive data points
 *                   -              | are above the slope interrupt threshold
 *                   SLO_NO_MOT_DURN        | Refer data sheet for clear information
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_DURN 75

/* BMI055_ACCEL_SET_DURN
 *                 - This API is used to set the duration of
 *                   Low, High, Slope and slow no motion interrupts in the registers
 *
 *                   NOTE: LOW_DURN		-> register 0x22 bit form 0 to 7
 *
 *                   NOTE: HIGH_DURN		-> register 0x25 bit form 0 to 7
 *
 *                   NOTE: SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *
 *                   NOTE: SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *   ioctl-argument: channel_u8: The value of duration select
 *                   channel_u8   | result
 *                   -----------------| ------------------
 *                   0    | BMA2x2_ACCEL_LOW_DURN
 *                   1    | BMA2x2_ACCEL_HIGH_DURN
 *                   2    | BMA2x2_ACCEL_SLOPE_DURN
 *                   3    | BMA2x2_ACCEL_SLOW_NO_MOTION_DURN
 *
 *                   durn_u8: The value of duration
 *                   @note :
 *                   Duration           |    result
 *                   -----------------------| ------------------
 *                   BMA2x2_ACCEL_LOW_DURN  | Low-g interrupt trigger
 *                   -              | delay according to([durn_u8 +1]*2)ms
 *                   -              | range from 2ms to 512ms. default is 20ms
 *                   BMA2x2_ACCEL_HIGH_DURN | high-g interrupt trigger
 *                   -              | delay according to([durn_u8 +1]*2)ms
 *                   -              | range from 2ms to 512ms. default is 32ms
 *                   BMA2x2_ACCEL_SLOPE_DURN| slope interrupt trigger
 *                   -              | if[durn_u8<1:0>+1] consecutive data points
 *                   -              | are above the slope interrupt threshold
 *                   SLO_NO_MOT_DURN        | Refer data sheet for clear information
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_DURN 76

/* BMI055_ACCEL_GET_THRES
 *                 - This API is used to get the threshold of
 *                   Low, High, Slope and slow no motion interrupts in the registers
 *
 *                   NOTE: LOW_THRES		-> register 0x23 bit form 0 to 7
 *
 *                   NOTE: HIGH_THRES		-> register 0x26 bit form 0 to 7
 *
 *                   NOTE: SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *
 *                   NOTE: SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *   ioctl-argument: channel_u8: The value of threshold selection
 *                   channel_u8   | result
 *                   -----------------| ------------------
 *                   0    | BMA2x2_ACCEL_LOW_THRES
 *                   1    | BMA2x2_ACCEL_HIGH_THRES
 *                   2    | BMA2x2_ACCEL_SLOPE_THRES
 *                   3    | BMA2x2_ACCEL_SLOW_NO_MOTION_THRES
 *
 *                   thres_u8: The threshold value of selected interrupts
 *                   @note : LOW-G THRESHOLD
 *                   Threshold                    |    result
 *                   ---------------------------------| ------------------
 *                   BMA2x2_ACCEL_LOW_THRES           | Low-threshold interrupt trigger
 *                   | according to(thres_u8 * 7.81) mg
 *                   | range from 0g to 1.992g
 *                   | default is 375mg
 *                   @note : HIGH-G THRESHOLD
 *                   @note Threshold of high-g interrupt according to accel g range
 *                   g-range           |      High-g threshold
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 7.81) mg
 *                   4g               |    (thres_u8 * 15.63) mg
 *                   8g               |    (thres_u8 * 31.25) mg
 *                   16g              |    (thres_u8 * 62.5) mg
 *                   @note : SLOPE THRESHOLD
 *                   @note Threshold of slope interrupt according to accel g range
 *                   g-range           |      Slope threshold
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 3.19) mg
 *                   4g               |    (thres_u8 * 7.81) mg
 *                   8g               |    (thres_u8 * 15.63) mg
 *                   16g              |    (thres_u8 * 31.25) mg
 *                   @note : SLOW NO MOTION THRESHOLD
 *                   @note Threshold of slow no motion interrupt according to accel g range
 *                   g-range           |   slow no motion threshold
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 3.19) mg
 *                   4g               |    (thres_u8 * 7.81) mg
 *                   8g               |    (thres_u8 * 15.63) mg
 *                   16g              |    (thres_u8 * 31.25) mg
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_THRES 77

/* BMI055_ACCEL_SET_THRES
 *                 - This API is used to set the threshold of
 *                   Low, High, Slope and slow no motion interrupts in the registers
 *
 *                   NOTE: LOW_THRES		-> register 0x23 bit form 0 to 7
 *
 *                   NOTE: HIGH_THRES		-> register 0x26 bit form 0 to 7
 *
 *                   NOTE: SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *
 *                   NOTE: SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *   ioctl-argument: channel_u8: The value of threshold selection
 *                   channel_u8   | result
 *                   -----------------| ------------------
 *                   0    | BMA2x2_ACCEL_LOW_THRES
 *                   1    | BMA2x2_ACCEL_HIGH_THRES
 *                   2    | BMA2x2_ACCEL_SLOPE_THRES
 *                   3    | BMA2x2_ACCEL_SLOW_NO_MOTION_THRES
 *
 *                   thres_u8: The threshold value of selected interrupts
 *                   @note : LOW-G THRESHOLD
 *                   Threshold                    |    result
 *                   ---------------------------------| ------------------
 *                   BMA2x2_ACCEL_LOW_THRES           | Low-threshold interrupt trigger
 *                   | according to(thres_u8 * 7.81) mg
 *                   | range from 0g to 1.992g
 *                   | default is 375mg
 *                   @note : HIGH-G THRESHOLD
 *                   @note Threshold of high-g interrupt according to accel g range
 *                   g-range           |      High-g threshold
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 7.81) mg
 *                   4g               |    (thres_u8 * 15.63) mg
 *                   8g               |    (thres_u8 * 31.25) mg
 *                   16g              |    (thres_u8 * 62.5) mg
 *                   @note : SLOPE THRESHOLD
 *                   @note Threshold of slope interrupt according to accel g range
 *                   g-range           |      Slope threshold
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 3.19) mg
 *                   4g               |    (thres_u8 * 7.81) mg
 *                   8g               |    (thres_u8 * 15.63) mg
 *                   16g              |    (thres_u8 * 31.25) mg
 *                   @note : SLOW NO MOTION THRESHOLD
 *                   @note Threshold of slow no motion interrupt according to accel g range
 *                   g-range           |   slow no motion threshold
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 3.19) mg
 *                   4g               |    (thres_u8 * 7.81) mg
 *                   8g               |    (thres_u8 * 15.63) mg
 *                   16g              |    (thres_u8 * 31.25) mg
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_THRES 78

/* BMI055_ACCEL_GET_LOW_HIGH_G_HYST
 *                 - This API is used to get
 *                   the low high hysteresis in the registers 0x24
 *
 *                   NOTE: LOW_G_HYST  -> bit form 0 to 1
 *
 *                   NOTE: HIGH_G_HYST  -> bit from 6 to 7
 *
 *   ioctl-argument: channel_u8: The value of hysteresis selection
 *                   channel_u8   | result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_LOW_G_HYST
 *                   1        | BMA2x2_ACCEL_HIGH_G_HYST
 *
 *                   hyst_u8: The hysteresis data
 *                   @note LOW HYSTERESIS
 *                   @note Hysteresis of low-g interrupt according to (hyst_u8 * 125)mg
 *                   @note HIGH HYSTERESIS
 *                   @note High hysteresis depends on the accel range selection
 *                   g-range           |    High Hysteresis
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 125) mg
 *                   4g               |    (thres_u8 * 250) mg
 *                   8g               |    (thres_u8 * 500) mg
 *                   16g              |    (thres_u8 * 1000) mg
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_LOW_HIGH_G_HYST 79

/* BMI055_ACCEL_SET_LOW_HIGH_G_HYST
 *                 - This API is used to set
 *                   the low high hysteresis in the registers 0x24
 *
 *                   NOTE: LOW_G_HYST  -> bit form 0 to 1
 *
 *                   NOTE: HIGH_G_HYST  -> bit from 6 to 7
 *
 *   ioctl-argument: channel_u8: The value of hysteresis selection
 *                   channel_u8   | result
 *                   -----------------| ------------------
 *                   0        | BMA2x2_ACCEL_LOW_G_HYST
 *                   1        | BMA2x2_ACCEL_HIGH_G_HYST
 *
 *                   hyst_u8: The hysteresis data
 *                   @note LOW HYSTERESIS
 *                   @note Hysteresis of low-g interrupt according to (hyst_u8 * 125)mg
 *                   @note HIGH HYSTERESIS
 *                   @note High hysteresis depends on the accel range selection
 *                   g-range           |    High Hysteresis
 *                   --------------------|----------------------------
 *                   2g               |    (thres_u8 * 125) mg
 *                   4g               |    (thres_u8 * 250) mg
 *                   8g               |    (thres_u8 * 500) mg
 *                   16g              |    (thres_u8 * 1000) mg
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_LOW_HIGH_G_HYST 80

/* BMI055_ACCEL_GET_LOW_G_MODE
 *                 - This API is used to get
 *                   low_g  mode in the registers 0x24 bit 2
 *
 *   ioctl-argument: low_g_mode_u8: The value of Low_G mode
 *                   low_g_mode_u8   |   g-result
 *                   --------------------|----------------------------
 *                   0x00             | LOW_G_SINGLE_AXIS_MODE
 *                   0x01             | LOW_G_SUMMING_MODE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_LOW_G_MODE 81

/* BMI055_ACCEL_SET_LOW_G_MODE
 *                 - This API is used to set
 *                   low_g  mode in the registers 0x24 bit 2
 *
 *   ioctl-argument: low_g_mode_u8: The value of Low_G mode
 *                   low_g_mode_u8   |    result
 *                   --------------------|----------------------------
 *                   0x00             | LOW_G_SINGLE_AXIS_MODE
 *                   0x01             | LOW_G_SUMMING_MODE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_LOW_G_MODE 82

/* BMI055_ACCEL_GET_TAP_DURN
 *                 - This API is used to get
 *                   the tap duration in the register 0x2A bit form 0 to 2
 *
 *   ioctl-argument: tap_durn_u8: The value of tap duration
 *                   tap_durn_u8     |    result
 *                   --------------------|----------------------------
 *                   0x00             | TAP_DURN_50_MS
 *                   0x01             | TAP_DURN_100_MS
 *                   0x02             | TAP_DURN_150_MS
 *                   0x03             | TAP_DURN_200_MS
 *                   0x04             | TAP_DURN_250_MS
 *                   0x05             | TAP_DURN_375_MS
 *                   0x06             | TAP_DURN_500_MS
 *                   0x07             | TAP_DURN_700_MS
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_TAP_DURN 83

/* BMI055_ACCEL_SET_TAP_DURN
 *                 - This API is used to set
 *                   the tap duration in the register 0x2A bit form 0 to 2
 *
 *   ioctl-argument: tap_durn_u8: The value of tap duration
 *                   tap_durn_u8     |    result
 *                   --------------------|----------------------------
 *                   0x00             | TAP_DURN_50_MS
 *                   0x01             | TAP_DURN_100_MS
 *                   0x02             | TAP_DURN_150_MS
 *                   0x03             | TAP_DURN_200_MS
 *                   0x04             | TAP_DURN_250_MS
 *                   0x05             | TAP_DURN_375_MS
 *                   0x06             | TAP_DURN_500_MS
 *                   0x07             | TAP_DURN_700_MS
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_TAP_DURN 84

/* BMI055_ACCEL_GET_TAP_SHOCK
 *                 - This API is used to get
 *                   the tap shock form the register 0x2A bit 6
 *
 *   ioctl-argument: tap_shock_u8: The value of tap shock
 *                   tap_shock_u8    |    result
 *                   --------------------|----------------------
 *                   0x00             | TAP_SHOCK_50_MS
 *                   0x01             | TAP_SHOCK_75_MS
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_TAP_SHOCK 85

/* BMI055_ACCEL_SET_TAP_SHOCK
 *                 - This API is used to set
 *                   the tap shock form the register 0x2A bit 6
 *
 *   ioctl-argument: tap_shock_u8: The value of tap shock
 *                   tap_shock_u8    |    result
 *                   --------------------|----------------------
 *                   0x00             | TAP_SHOCK_50_MS
 *                   0x01             | TAP_SHOCK_75_MS
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_TAP_SHOCK 86

/* BMI055_ACCEL_GET_TAP_QUIET
 *                 - This API is used to get
 *                   the tap quiet in the register 0x2A bit 7
 *
 *   ioctl-argument: tap_quiet_u8 : The value of tap quiet
 *                   tap_quiet_u8    |    result
 *                   --------------------|----------------------
 *                   0x00             | TAP_QUIET_30_MS
 *                   0x01             | TAP_QUIET_20_MS
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_TAP_QUIET 87

/* BMI055_ACCEL_SET_TAP_QUIET
 *                 - This API is used to set
 *                   the tap quiet in the register 0x2A bit 7
 *
 *   ioctl-argument: tap_quiet_u8 : The value of tap quiet
 *                   tap_quiet_u8    |    result
 *                   --------------------|----------------------
 *                   0x00             | TAP_QUIET_30_MS
 *                   0x01             | TAP_QUIET_20_MS
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_TAP_QUIET 88

/* BMI055_ACCEL_GET_TAP_THRES
 *                 - This API is used to get
 *                   the tap threshold in the register 0x2B bit from 0 to 4
 *
 *   ioctl-argument: tap_thres_u8 : The value of tap threshold
 *                   @note Tap threshold of single and double tap corresponding
 *                   to accel range
 *                   range            |    Tap threshold
 *                   --------------------|----------------------
 *                   2g               | (tap_thres_u8 * 62.5)mg
 *                   4g               | (tap_thres_u8 * 125)mg
 *                   8g               | (tap_thres_u8 * 250)mg
 *                   16g              | (tap_thres_u8 * 500)mg
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_TAP_THRES 89

/* BMI055_ACCEL_SET_TAP_THRES
 *                 - This API is used to set
 *                   the tap threshold in the register 0x2B bit from 0 to 4
 *
 *   ioctl-argument: tap_thres_u8 : The value of tap threshold
 *                   @note Tap threshold of single and double tap corresponding
 *                   to accel range
 *                   range            |    Tap threshold
 *                   --------------------|----------------------
 *                   2g               | (tap_thres_u8 * 62.5)mg
 *                   4g               | (tap_thres_u8 * 125)mg
 *                   8g               | (tap_thres_u8 * 250)mg
 *                   16g              | (tap_thres_u8 * 500)mg
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_TAP_THRES 90

/* BMI055_ACCEL_GET_TAP_SAMPLE
 *                 - This API is used to get
 *                   the tap sample in the register 0x2B bit 6 and 7
 *
 *   ioctl-argument: tap_sample_u8 : The value of tap sample
 *                   tap_sample_u8  |    result
 *                   --------------------|----------------------
 *                   0x00             | 2 samples
 *                   0x01             | 4 samples
 *                   0x02             | 8 samples
 *                   0x03             | 16 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_TAP_SAMPLE 91

/* BMI055_ACCEL_SET_TAP_SAMPLE
 *                 - This API is used to set
 *                   the tap sample in the register 0x2B bit 6 and 7
 *
 *   ioctl-argument: tap_sample_u8 : The value of tap sample
 *                   tap_sample_u8  |    result
 *                   --------------------|----------------------
 *                   0x00             | 2 samples
 *                   0x01             | 4 samples
 *                   0x02             | 8 samples
 *                   0x03             | 16 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_TAP_SAMPLE 92

/* BMI055_ACCEL_GET_ORIENT_MODE
 *                 - This API is used to get
 *                   the orient mode in the register 0x2C bit 0 and 1
 *
 *   ioctl-argument: orient_mode_u8 : The value of orient mode
 *                   orient_mode_u8 |    result
 *                   --------------------|------------------
 *                   0x00             | symmetrical
 *                   0x01             | high asymmetrical
 *                   0x02             | low asymmetrical
 *                   0x03             | symmetrical
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_ORIENT_MODE 93

/* BMI055_ACCEL_SET_ORIENT_MODE
 *                 - This API is used to set
 *                   the orient mode in the register 0x2C bit 0 and 1
 *
 *   ioctl-argument: orient_mode_u8 : The value of orient mode
 *                   orient_mode_u8 |    result
 *                   --------------------|------------------
 *                   0x00             | symmetrical
 *                   0x01             | high asymmetrical
 *                   0x02             | low asymmetrical
 *                   0x03             | symmetrical
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_ORIENT_MODE 94

/* BMI055_ACCEL_GET_ORIENT_BLOCK
 *                 - This API is used to get
 *                   the orient block in the register 0x2C bit 2 and 3
 *
 *   ioctl-argument: orient_block_u8 : The value of orient block
 *                   orient_mode_u8 |    result
 *                   --------------------|------------------
 *                   0x00             | no blocking
 *                   0x01             | theta blocking or
 *                   | acceleration slope in any axis > 1.5g
 *                   0x02             | theta blocking or
 *                   | acceleration slope in any axis > 0.2g
 *                   | acceleration in any axis > 1.5g
 *                   0x03             | theta blocking or
 *                   | acceleration slope in any axis > 0.4g
 *                   | acceleration in any axis > 1.5g
 *                   | value of orient is not stable for at lease 100ms
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_ORIENT_BLOCK 95

/* BMI055_ACCEL_SET_ORIENT_BLOCK
 *                 - This API is used to set
 *                   the orient block in the register 0x2C bit 2 and 3
 *
 *   ioctl-argument: orient_block_u8 : The value of orient block
 *                   orient_mode_u8 |    result
 *                   --------------------|------------------
 *                   0x00             | no blocking
 *                   0x01             | theta blocking or
 *                   | acceleration slope in any axis > 1.5g
 *                   0x02             | theta blocking or
 *                   | acceleration slope in any axis > 0.2g
 *                   | acceleration in any axis > 1.5g
 *                   0x03             | theta blocking or
 *                   | acceleration slope in any axis > 0.4g
 *                   | acceleration in any axis > 1.5g
 *                   | value of orient is not stable for at lease 100ms
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_ORIENT_BLOCK 96

/* BMI055_ACCEL_GET_ORIENT_HYST
 *                 - This API is used to get
 *                   the orient hysteresis in the register 0x2C bit 4 to 6
 *
 *   ioctl-argument: orient_hyst_u8 : The value of orient hysteresis
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_ORIENT_HYST 97

/* BMI055_ACCEL_SET_ORIENT_HYST
 *                 - This API is used to set
 *                   the orient hysteresis in the register 0x2C bit 4 to 6
 *
 *   ioctl-argument: orient_hyst_u8 : The value of orient hysteresis
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_ORIENT_HYST 98

/* BMI055_ACCEL_GET_THETA
 *                 - This API is used to get
 *                   the theta value of orient and flat interrupts
 *
 *                   NOTE: ORIENT_THETA -> register 0x2D bit 0 to 5
 *
 *                   NOTE: FLAT_THETA   -> register 0x2E bit 0 to 5
 *
 *   ioctl-argument: channel_u8: The value of theta selection
 *                   channel_u8     |    result
 *                   --------------------|------------------
 *                   0x00             | BMA2x2_ACCEL_ORIENT_THETA
 *                   0x01             | BMA2x2_ACCEL_FLAT_THETA
 *                   @note
 *                   @note FLAT_THETA : Defines a blocking angle between 0 deg to 44.8 deg
 *                   @note ORIENT_THETA : Defines threshold for detection of flat position
 *                   in range from 0 deg to 44.8 deg
 *
 *                   theta_u8: The value of theta
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_THETA 99

/* BMI055_ACCEL_SET_THETA
 *                 - This API is used to set
 *                   the theta value of orient and flat interrupts
 *
 *                   NOTE: ORIENT_THETA -> register 0x2D bit 0 to 5
 *
 *                   NOTE: FLAT_THETA   -> register 0x2E bit 0 to 5
 *
 *   ioctl-argument: channel_u8: The value of theta selection
 *                   channel_u8     |    result
 *                   --------------------|------------------
 *                   0x00             | BMA2x2_ACCEL_ORIENT_THETA
 *                   0x01             | BMA2x2_ACCEL_FLAT_THETA
 *                   @note
 *                   @note FLAT_THETA : Defines a blocking angle between 0 deg to 44.8 deg
 *                   @note ORIENT_THETA : Defines threshold for detection of flat position
 *                   in range from 0 deg to 44.8 deg
 *
 *                   theta_u8: The value of theta
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_THETA 100

/* BMI055_ACCEL_GET_ORIENT_ENABLE
 *                 - This API is used to get
 *                   the interrupt enable of orient ud_enable in the register 0x2D bit 6
 *
 *   ioctl-argument: orient_enable_u8 : The value of orient ud_enable
 *                   orient_enable_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | Generates Interrupt
 *                   0x01                   | Do not generate interrupt
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_ORIENT_ENABLE 101

/* BMI055_ACCEL_SET_ORIENT_ENABLE
 *                 - This API is used to set
 *                   the interrupt enable of orient ud_enable in the register 0x2D bit 6
 *
 *   ioctl-argument: orient_enable_u8 : The value of orient ud_enable
 *                   orient_enable_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | Generates Interrupt
 *                   0x01                   | Do not generate interrupt
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_ORIENT_ENABLE 102

/* BMI055_ACCEL_GET_FLAT_HYST
 *                 - This API is used to get
 *                   the interrupt enable of flat hysteresis("flat_hy)
 *                   in the register 0x2F bit 0 to 2
 *
 *   ioctl-argument: flat_hyst_u8 : The value of flat hysteresis
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FLAT_HYST 103

/* BMI055_ACCEL_SET_FLAT_HYST
 *                 - This API is used to set
 *                   the interrupt enable of flat hysteresis("flat_hy)
 *                   in the register 0x2F bit 0 to 2
 *
 *   ioctl-argument: flat_hyst_u8 : The value of flat hysteresis
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_FLAT_HYST 104

/* BMI055_ACCEL_GET_FLAT_HOLD_TIME
 *                 - This API is used to get
 *                   the interrupt enable of flat hold time(flat_hold_time)
 *                   in the register 0x2F bit 4 and 5
 *
 *   ioctl-argument: flat_hold_time_u8 : The value of flat hold time
 *                   flat_hold_time_u8    |    result
 *                   ------------------------- |------------------
 *                   0x00                   | 0ms
 *                   0x01                   | 512ms
 *                   0x02                   | 1024ms
 *                   0x03                   | 2048ms
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FLAT_HOLD_TIME 105

/* BMI055_ACCEL_SET_FLAT_HOLD_TIME
 *                 - This API is used to set
 *                   the interrupt enable of flat hold time(flat_hold_time)
 *                   in the register 0x2F bit 4 and 5
 *
 *   ioctl-argument: flat_hold_time_u8 : The value of flat hold time
 *                   flat_hold_time_u8    |    result
 *                   ------------------------- |------------------
 *                   0x00                   | 0ms
 *                   0x01                   | 512ms
 *                   0x02                   | 1024ms
 *                   0x03                   | 2048ms
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_FLAT_HOLD_TIME 106

/* BMI055_ACCEL_GET_FIFO_WML_TRIG
 *                 - This API is used to get
 *                   the fifo water mark level trigger in the register 0x30 bit from 0 to 5
 *
 *   ioctl-argument: fifo_wml_trig: The value of fifo watermark trigger level
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FIFO_WML_TRIG 107

/* BMI055_ACCEL_SET_FIFO_WML_TRIG
 *                 - This API is used to set
 *                   the fifo water mark level trigger in the register 0x30 bit from 0 to 5
 *
 *   ioctl-argument: fifo_wml_trig: The value of fifo watermark trigger level
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_FIFO_WML_TRIG 108

/* BMI055_ACCEL_GET_SELFTEST_AXIS
 *                 - This API is for to get
 *                   the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *   ioctl-argument: selftest_axis_u8 : The value of selftest axis
 *                   selftest_axis_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | self test disable
 *                   0x01                   | x-axis
 *                   0x02                   | y-axis
 *                   0x03                   | z-axis
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SELFTEST_AXIS 109

/* BMI055_ACCEL_SET_SELFTEST_AXIS
 *                 - This API is for to set
 *                   the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *   ioctl-argument: selftest_axis_u8 : The value of selftest axis
 *                   selftest_axis_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | self test disable
 *                   0x01                   | x-axis
 *                   0x02                   | y-axis
 *                   0x03                   | z-axis
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SELFTEST_AXIS 110

/* BMI055_ACCEL_GET_SELFTEST_SIGN
 *                 - This API is for to get
 *                   the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *   ioctl-argument: selftest_sign_u8 : The value of self test sign
 *                   selftest_sign_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | negative sign
 *                   0x01                   | positive sign
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SELFTEST_SIGN 111

/* BMI055_ACCEL_SET_SELFTEST_SIGN
 *                 - This API is for to set
 *                   the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *   ioctl-argument: selftest_sign_u8 : The value of self test sign
 *                   selftest_sign_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | negative sign
 *                   0x01                   | positive sign
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SELFTEST_SIGN 112

/* BMI055_ACCEL_GET_NVMPROG_MODE
 *                 - This API is used to get
 *                   the nvm program mode(nvm_prog_mode)in the register 0x33 bit 0
 *
 *   ioctl-argument: nvmprog_mode_u8 : The value of nvm program mode
 *                   nvmprog_mode_u8      |    result
 *                   ------------------------- |------------------
 *                   0x00                   | Disable program mode
 *                   0x01                   | Enable program mode
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_NVMPROG_MODE 113

/* BMI055_ACCEL_SET_NVMPROG_MODE
 *                 - This API is used to set
 *                   the nvm program mode(nvm_prog_mode)in the register 0x33 bit 0
 *
 *   ioctl-argument: nvmprog_mode_u8 : The value of nvm program mode
 *                   nvmprog_mode_u8      |    result
 *                   ------------------------- |------------------
 *                   0x00                   | Disable program mode
 *                   0x01                   | Enable program mode
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_NVMPROG_MODE 114

/* BMI055_ACCEL_SET_NVPROG_TRIG
 *                 - This API is used to set
 *                   the value of nvm program trig in the register 0x33 bit 1
 *
 *   ioctl-argument: nvprog_trig_u8: The value of nvm program trig
 *                   nvprog_trig_u8       |    result
 *                   ------------------------- |------------------
 *                   0x00                   | Do not trigger nvm program
 *                   0x01                   | Trigger nvm program
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_NVPROG_TRIG 115

/* BMI055_ACCEL_GET_NVMPROG_READY
 *                 - This API is used to get
 *                   the nvm program ready in the register bit 2
 *
 *   ioctl-argument: nvprog_ready_u8: The value of nvm program ready
 *                   nvprog_ready_u8      |    result
 *                   ------------------------- |------------------
 *                   0x00                   | nvm write/update operation is in progress
 *                   0x01                   | nvm is ready to accept a new write
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_NVMPROG_READY 116

/* BMI055_ACCEL_GET_NVMPROG_REMAIN
 *                 - This API is used to set
 *                   the nvm program ready in the register bit 2
 *
 *   ioctl-argument: nvprog_remain_u8: The value of nvm program ready
 *                   nvprog_remain_u8     |    result
 *                   ------------------------- |------------------
 *                   0x00                   | nvm write/update operation is in progress
 *                   0x01                   | nvm is ready to accept a new write
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_NVMPROG_REMAIN 117

/* BMI055_ACCEL_GET_SPI3
 *                 - This API is used to get the enable status of spi3
 *                   in the register 0x34 bit 0
 *
 *   ioctl-argument: spi3_u8 : The value of SPI 3 or 4 wire enable
 *                   spi3_u8              |    result
 *                   ------------------------- |------------------
 *                   0x00                   |     spi4
 *                   0x01                   |     spi3
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SPI3 118

/* BMI055_ACCEL_SET_SPI3
 *                 - This API is used to set the enable status of spi3
 *                   in the register 0x34 bit 0
 *
 *   ioctl-argument: spi3_u8 : The value of SPI 3 or 4 wire enable
 *                   spi3_u8              |    result
 *                   ------------------------- |------------------
 *                   0x00                   |     spi4
 *                   0x01                   |     spi3
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SPI3 119

/* BMI055_ACCEL_GET_I2C_WDT
 *                 - This API is used to get the i2c
 *                   watch dog timer period and I2C interface mode is selected
 *                   in the register 0x34 bit 1 and 2
 *
 *   ioctl-argument: channel_u8: The i2c option selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_I2C_SELECT
 *                   1                   |   BMA2x2_ACCEL_I2C_ENABLE
 *
 *                   i2c_wdt_u8: watch dog timer period
 *                   and I2C interface mode is selected
 *                   BMA2x2_ACCEL_I2C_SELECT|    result
 *                   ------------------------- |------------------
 *                   0x00                   | Disable the watchdog at SDI pin
 *                   0x01                   | Enable watchdog
 *                   BMA2x2_I2C_ENABLE      |    result
 *                   ------------------------- |------------------
 *                   0x00                   | 1ms
 *                   0x01                   | 50ms
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_I2C_WDT 120

/* BMI055_ACCEL_SET_I2C_WDT
 *                 - This API is used to set the i2c
 *                   watch dog timer period and I2C interface mode is selected
 *                   in the register 0x34 bit 1 and 2
 *
 *   ioctl-argument: channel_u8: The i2c option selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_I2C_SELECT
 *                   1                   |   BMA2x2_ACCEL_I2C_ENABLE
 *
 *                   i2c_wdt_u8: watch dog timer period
 *                   and I2C interface mode is selected
 *                   BMA2x2_ACCEL_I2C_SELECT|    result
 *                   ------------------------- |------------------
 *                   0x00                   | Disable the watchdog at SDI pin
 *                   0x01                   | Enable watchdog
 *                   BMA2x2_I2C_ENABLE      |    result
 *                   ------------------------- |------------------
 *                   0x00                   | 1ms
 *                   0x01                   | 50ms
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_I2C_WDT 121

/* BMI055_ACCEL_GET_SLOW_COMP
 *                 - This API is used to get
 *                   slow compensation(hp_x_enable, hp_y_enable and hp_z_enable) enable
 *                   in the register 0x36 bit 0 to 2
 *
 *                   NOTE: SLOW_COMP_X -> bit 0
 *
 *                   NOTE: SLOW_COMP_Y -> bit 1
 *
 *                   NOTE: SLOW_COMP_Z -> bit 2
 *
 *   ioctl-argument: channel_u8: The value of slow compensation selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_SLOW_COMP_X
 *                   1                   |   BMA2x2_ACCEL_SLOW_COMP_Y
 *                   2                   |   BMA2x2_ACCEL_SLOW_COMP_Z
 *
 *                   slow_comp_u8: The value of slow compensation enable
 *                   slow_comp_u8         |    result
 *                   ------------------------- |------------------
 *                   0x00               |    Disable
 *                   0x01                |    Enable
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_SLOW_COMP 122

/* BMI055_ACCEL_SET_SLOW_COMP
 *                 - This API is used to set
 *                   slow compensation(hp_x_enable, hp_y_enable and hp_z_enable) enable
 *                   in the register 0x36 bit 0 to 2
 *
 *                   NOTE: SLOW_COMP_X -> bit 0
 *
 *                   NOTE: SLOW_COMP_Y -> bit 1
 *
 *                   NOTE: SLOW_COMP_Z -> bit 2
 *
 *   ioctl-argument: channel_u8: The value of slow compensation selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_SLOW_COMP_X
 *                   1                   |   BMA2x2_ACCEL_SLOW_COMP_Y
 *                   2                   |   BMA2x2_ACCEL_SLOW_COMP_Z
 *
 *                   slow_comp_u8: The value of slow compensation enable
 *                   slow_comp_u8         |    result
 *                   ------------------------- |------------------
 *                   0x00               |    Disable
 *                   0x01                |    Enable
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_SLOW_COMP 123

/* BMI055_ACCEL_GET_CAL_RDY
 *                 - This API is used to get
 *                   the status of fast offset compensation(cal_rdy) in the register 0x36
 *                   bit 4(Read Only Possible)
 *
 *   ioctl-argument: cal_rdy_u8: The value of cal_ready
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_CAL_RDY 124

/* BMI055_ACCEL_SET_CAL_TRIGGER
 *                 - This API is used to set
 *                   the status of fast offset compensation(cal_rdy) in the register 0x36
 *                   bit 4(Read Only Possible)
 *
 *   ioctl-argument: cal_trigger_u8: The value of cal_ready
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_CAL_TRIGGER 125

/* BMI055_ACCEL_SET_OFFSET_RST
 *                 - This API is used to set
 *                   the offset reset(offset_reset) in the register 0x36
 *                   bit 7(Write only possible)
 *
 *   ioctl-argument: offset_rst_u8: The offset reset value
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_OFFSET_RST 126

/* BMI055_ACCEL_GET_OFFSET_TARGET
 *                 - This API is used to get
 *                   the status of offset target axis(offset_target_x, offset_target_y and
 *                   offset_target_z) and cut_off in the register 0x37
 *
 *                   NOTE: CUT_OFF -> bit 0
 *
 *                   NOTE: OFFSET_TRIGGER_X -> bit 1 and 2
 *
 *                   NOTE: OFFSET_TRIGGER_Y -> bit 3 and 4
 *
 *                   NOTE: OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *   ioctl-argument: channel_u8: The value of offset axis selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_CUT_OFF
 *                   1                   |   BMA2x2_ACCEL_OFFSET_TRIGGER_X
 *                   2                   |   BMA2x2_ACCEL_OFFSET_TRIGGER_Y
 *                   2                   |   BMA2x2_ACCEL_OFFSET_TRIGGER_Z
 *
 *                   offset_u8: The offset target value
 *                   CUT_OFF                |    result
 *                   ------------------------- |------------------
 *                   0                   |   1Hz
 *                   1                   |   10Hz
 *                   OFFSET_TRIGGER         |    result
 *                   ------------------------- |------------------
 *                   0x00                |   0g
 *                   0x01                |   +1g
 *                   0x02                |   -1g
 *                   0x03                |   0g
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_OFFSET_TARGET 127

/* BMI055_ACCEL_SET_OFFSET_TARGET
 *                 - This API is used to set
 *                   the status of offset target axis(offset_target_x, offset_target_y and
 *                   offset_target_z) and cut_off in the register 0x37
 *
 *                   NOTE: CUT_OFF -> bit 0
 *
 *                   NOTE: OFFSET_TRIGGER_X -> bit 1 and 2
 *
 *                   NOTE: OFFSET_TRIGGER_Y -> bit 3 and 4
 *
 *                   NOTE: OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *   ioctl-argument: channel_u8: The value of offset axis selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_CUT_OFF
 *                   1                   |   BMA2x2_ACCEL_OFFSET_TRIGGER_X
 *                   2                   |   BMA2x2_ACCEL_OFFSET_TRIGGER_Y
 *                   2                   |   BMA2x2_ACCEL_OFFSET_TRIGGER_Z
 *
 *                   offset_u8: The offset target value
 *                   CUT_OFF                |    result
 *                   ------------------------- |------------------
 *                   0                   |   1Hz
 *                   1                   |   10Hz
 *                   OFFSET_TRIGGER         |    result
 *                   ------------------------- |------------------
 *                   0x00                |   0g
 *                   0x01                |   +1g
 *                   0x02                |   -1g
 *                   0x03                |   0g
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_OFFSET_TARGET 128

/* BMI055_ACCEL_GET_OFFSET
 *                 - This API is used to get the status of offset
 *                   (offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *
 *                   NOTE: offset_x -> register 0x38 bit 0 to 7
 *
 *                   NOTE: offset_y -> register 0x39 bit 0 to 7
 *
 *                   NOTE: offset_z -> register 0x3A bit 0 to 7
 *
 *   ioctl-argument: channel_u8: The value of offset selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_X_AXIS
 *                   1                   |   BMA2x2_ACCEL_Y_AXIS
 *                   2                   |   BMA2x2_ACCEL_Z_AXIS
 *
 *                   offset_u8: The value of offset
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_OFFSET 129

/* BMI055_ACCEL_SET_OFFSET
 *                 - This API is used to set the status of offset
 *                   (offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *
 *                   NOTE: offset_x -> register 0x38 bit 0 to 7
 *
 *                   NOTE: offset_y -> register 0x39 bit 0 to 7
 *
 *                   NOTE: offset_z -> register 0x3A bit 0 to 7
 *
 *   ioctl-argument: channel_u8: The value of offset selection
 *                   channel_u8           |    result
 *                   ------------------------- |------------------
 *                   0                   |   BMA2x2_ACCEL_X_AXIS
 *                   1                   |   BMA2x2_ACCEL_Y_AXIS
 *                   2                   |   BMA2x2_ACCEL_Z_AXIS
 *
 *                   offset_u8: The value of offset
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_OFFSET 130

/* BMI055_ACCEL_GET_FIFO_MODE
 *                 - This API is used to get
 *                   the status of fifo (fifo_mode) in the register 0x3E bit 6 and 7
 *
 *   ioctl-argument: fifo_mode_u8 : The value of fifo mode
 *                   fifo_mode_u8         |    result
 *                   ------------------------- |------------------
 *                   0x00                |   BYPASS
 *                   0x01                |   FIFO
 *                   0x02                |   STREAM
 *                   0x03                |   RESERVED
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FIFO_MODE 131

/* BMI055_ACCEL_SET_FIFO_MODE
 *                 - This API is used to set
 *                   the status of fifo (fifo_mode) in the register 0x3E bit 6 and 7
 *
 *   ioctl-argument: fifo_mode_u8 : The value of fifo mode
 *                   fifo_mode_u8         |    result
 *                   ------------------------- |------------------
 *                   0x00                |   BYPASS
 *                   0x01                |   FIFO
 *                   0x02                |   STREAM
 *                   0x03                |   RESERVED
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_FIFO_MODE 132

/* BMI055_ACCEL_GET_FIFO_DATA_SELECT
 *                 - This API is used to get
 *                   the axis enable of fifo data select in the register 0x3E bit 0 and 1
 *
 *   ioctl-argument: fifo_data_select_u8 : The value of FIFO axis data select
 *                   fifo_data_select_u8    |    result
 *                   ------------------------- |------------------
 *                   0x00                |   XYZ
 *                   0x01                |   Y
 *                   0x02                |   X
 *                   0x03                |   Z
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_GET_FIFO_DATA_SELECT 133

/* BMI055_ACCEL_SET_FIFO_DATA_SELECT
 *                 - This API is used to set
 *                   the axis enable of fifo data select in the register 0x3E bit 0 and 1
 *
 *   ioctl-argument: fifo_data_select_u8 : The value of FIFO axis data select
 *                   fifo_data_select_u8    |    result
 *                   ------------------------- |------------------
 *                   0x00                |   XYZ
 *                   0x01                |   Y
 *                   0x02                |   X
 *                   0x03                |   Z
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_SET_FIFO_DATA_SELECT 134

/* BMI055_ACCEL_READ_FIFO_DATA
 *                 - This API reads the FIFO data from the register 0x3F
 *                   and store the data in the user defined buffer mapped to the member
 *                   of structure "fifo_configuration"
 *
 *                   NOTE: Before calling this API user must map the following FIFO settings
 *                   required to read the FIFO data to the structure "fifo_configuration"
 *                   - Data buffer to store the FIFO data is mapped to
 *                   the structure member "fifo_data"
 *                   - Number of bytes to be read from FIFO is mapped to
 *                   the structure member "fifo_length"
 *
 *                   NOTE: The number of bytes to be read from the FIFO is specified in the
 *                   member "fifo_length" of the structure "fifo_configuration"
 *
 *   ioctl-argument: [in,out] fifo_conf : Structure containing the FIFO configurations
 *                   is passed as input and FIFO data of specified length is obtained as output
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_FIFO_DATA 135

/* BMI055_ACCEL_EXTRACT_ACCEL
 *                 - This API extracts the accel data from the FIFO frames
 *
 *                   NOTE: The bmi055_extract_accel() API should be called only after reading
 *                   the FIFO data by calling the bmi055_read_fifo_data() API
 *
 *   ioctl-argument: [in,out] accel_frame      : Instance of the union where accel data
 *                   in FIFO is parsed and stored
 *
 *                   [in,out] accel_frame_count: Number of Accel frames requested by user
 *                   is got as input and number of
 *                   accel frames parsed and stored is
 *                   returned as output to user
 *
 *                   [in, out] fifo_conf       : FIFO configuration structure.
 *                   It provides the following as input
 *                   - user defined buffer
 *                   - length of FIFO data read
 *                   It returns the accel_byte_start_index
 *                   (index of accel bytes parsed from FIFO)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_EXTRACT_ACCEL 136

/* BMI055_ACCEL_READ_TEMP
 *                 - This API is used to read the temp
 *                   from register 0x08
 *
 *   ioctl-argument: temp_s8: The value of temperature
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_TEMP 137

/* BMI055_ACCEL_READ_ACCEL_XYZT
 *                 - This API reads accelerometer data X,Y,Z values and
 *                   temperature data from location 02h to 08h
 *
 *   ioctl-argument: accel : The value of accel xyz and temperature data
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_XYZT 138

/* BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_XYZT
 *                 - This API reads accelerometer data X,Y,Z values and
 *                   temperature data from location 0x02 to 0x08
 *
 *   ioctl-argument: accel : The value of accel xyz and temperature data
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_XYZT 139

/**************************************************************************
 * Gyroscope functions following:
 **************************************************************************/

/* BMI055_GYRO_GET_DATA_X
 *                 - Reads Rate data X in the registers 0x02 to 0x03
 *
 *   ioctl-argument: v_data_x_s16: The value of gyro x axis data
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_DATA_X 200

/* BMI055_GYRO_GET_DATA_Y
 *                 - Reads Rate data Y in the registers 0x04 to 0x05
 *
 *   ioctl-argument: v_data_y_s16: The value of gyro y axis data
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_DATA_Y 201

/* BMI055_GYRO_GET_DATA_Z
 *                 - Reads Rate data Z in the registers 0x06 to 0x07
 *
 *   ioctl-argument: v_data_z_s16: The value of gyro z axis data
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_DATA_Z 202

/* BMI055_GYRO_GET_DATA_XYZ
 *                 - Reads data X,Y and Z from register location 0x02 to 0x07
 *
 *   ioctl-argument: data: The value of gyro xyz axis data
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_DATA_XYZ 203

/* BMI055_GYRO_GET_DATA_XYZI
 *                 - Reads data X,Y,Z and Interrupts
 *                   from register location 0x02 to 0x07
 *
 *   ioctl-argument: data: The value of gyro xyz axis data and interrupt status
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_DATA_XYZI 204

/* BMI055_GYRO_GET_TEMP
 *                 - Reads Temperature from register location 0x08
 *
 *   ioctl-argument: v_temp_s8: The value of temperature
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_TEMP 205

/* BMI055_GYRO_READ_REGISTER
 *                 - This API reads the data from
 *                   the given register
 *
 *   ioctl-argument: v_addr_u8 -> Address of the register
 *
 *                   v_data_u8 -> The data from the register
 *
 *                   v_len_u8 -> no of bytes to read
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_READ_REGISTER 206

/* BMI055_GYRO_BURST_READ
 *                 - This API reads the data from
 *                   the given register
 *
 *   ioctl-argument: v_addr_u8 -> Address of the register
 *
 *                   v_data_u8 -> The data from the register
 *
 *                   v_len_u32 -> no of bytes to read
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_BURST_READ 207

/* BMI055_GYRO_WRITE_REGISTER
 *                 - This API write the data to
 *                   the given register
 *
 *   ioctl-argument: v_addr_u8 -> Address of the register
 *
 *                   v_data_u8 -> The data from the register
 *
 *                   v_len_u8 -> no of bytes to read
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_WRITE_REGISTER 208

/* BMI055_GYRO_GET_INTR_STAT_REG_ZERO
 *                 - This api used to reads interrupt status of
 *                   any motion and high rate in the register 0x09
 *
 *                   NOTE:  any motion bit	->	2
 *
 *                   NOTE: high rate bit	->	1
 *
 *   ioctl-argument: v_stat0_data_u8 : The interrupt status of
 *                   any motion and high rate
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_STAT_REG_ZERO 209

/* BMI055_GYRO_GET_INTR_STAT_REG_ONE
 *                 - This api used to reads the interrupt status of
 *                   data, auto_offset, fast_offset and fifo_int in the register 0x0A
 *
 *                   NOTE: data bit			->	7
 *
 *                   NOTE: auto_offset bit	->	6
 *
 *                   NOTE: fast_offset bit	->	5
 *
 *                   NOTE: fifo_int bit		->	4
 *
 *   ioctl-argument:
 *                   v_stat1_data_u8 : The interrupt status of
 *                   data, auto_offset, fast_offset and fifo_int
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_STAT_REG_ONE 210

/* BMI055_GYRO_GET_INTR_STAT_REG_TWO
 *                 - This api used to reads the interrupt status of
 *
 *                   NOTE: any motion sign, any motion first_z, any motion
 *                   first_x and any motion first_y in the register 0x0B
 *
 *                   NOTE: any motion sign bit		->	3
 *
 *                   NOTE: any motion first_z bit	->	2
 *
 *                   NOTE: any motion first_x bit	->	1
 *
 *                   NOTE: any motion first_y bit	->	0
 *
 *   ioctl-argument:
 *                   v_stat2_data_u8 : Pointer holding the the interrupt status of
 *                   any motion sign, any motion first_z,
 *                   any motion first_x and any motion first_y
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_STAT_REG_TWO 211

/* BMI055_GYRO_GET_INTR_STAT_REG_THREE
 *                 - This api used to reads the interrupt status of
 *                   high_rate sign, high_rate first_z, high_rate first_x
 *                   and high_rate first_y in the register 0x0C
 *
 *                   NOTE: high_rate sign bit		->	3
 *
 *                   NOTE: high_rate first_z bit	->	2
 *
 *                   NOTE: high_rate first_x bit	->	1
 *
 *                   NOTE: high_rate first_y bit	->	0
 *
 *                   NOTE: high_rate first_y bit	->	0
 *
 *   ioctl-argument:
 *                   v_stat3_data_u8 : The interrupt status of
 *                   high_rate sign, high_rate first_z,
 *                   high_rate first_x and high_rate first_y
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_STAT_REG_THREE 212

/* BMI055_GYRO_GET_RANGE_REG
 *                 - This API is used to get
 *                   the range in the register 0x0F bits from 0 to 2
 *
 *   ioctl-argument: v_range_u8 : The value of gyro range
 *                   value    |   range
 *                   ----------|-----------
 *                   0x00   | BMI055_RANGE_2000
 *                   0x01   | BMI055_RANGE_1000
 *                   0x02   | BMI055_RANGE_500
 *                   0x03   | BMI055_RANGE_250
 *                   0x04   | BMI055_RANGE_125
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_RANGE_REG 213

/* BMI055_GYRO_SET_RANGE_REG
 *                 - This API is used to set
 *                   the range in the register 0x0F bits from 0 to 2
 *
 *   ioctl-argument: v_range_u8 : The value of gyro range
 *                   value    |   range
 *                   ----------|-----------
 *                   0x00   | BMI055_RANGE_2000
 *                   0x01   | BMI055_RANGE_1000
 *                   0x02   | BMI055_RANGE_500
 *                   0x03   | BMI055_RANGE_250
 *                   0x04   | BMI055_RANGE_125
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_RANGE_REG 214

/* BMI055_GYRO_GET_BW
 *                 - This API is used to get the gyro bandwidth
 *                   in the register 0x10 bits from 0 to 3
 *
 *   ioctl-argument: v_bw_u8: The value of gyro bandwidth
 *                   value   |  bandwidth
 *                   ---------|---------------
 *                   0x00   |  BMI055_BW_500_HZ
 *                   0x01   |  BMI055_BW_230_HZ
 *                   0x02   |  BMI055_BW_116_HZ
 *                   0x03   |  BMI055_BW_47_HZ
 *                   0x04   |  BMI055_BW_23_HZ
 *                   0x05   |  BMI055_BW_12_HZ
 *                   0x06   |  BMI055_BW_64_HZ
 *                   0x07   |  BMI055_BW_32_HZ
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_BW 215

/* BMI055_GYRO_SET_BW
 *                 - This API is used to set the gyro bandwidth
 *                   in the register 0x10 bits from 0 to 3
 *
 *   ioctl-argument: v_bw_u8: The value of gyro bandwidth
 *                   value   |  bandwidth
 *                   ---------|---------------
 *                   0x00   |  BMI055_BW_500_HZ
 *                   0x01   |  BMI055_BW_230_HZ
 *                   0x02   |  BMI055_BW_116_HZ
 *                   0x03   |  BMI055_BW_47_HZ
 *                   0x04   |  BMI055_BW_23_HZ
 *                   0x05   |  BMI055_BW_12_HZ
 *                   0x06   |  BMI055_BW_64_HZ
 *                   0x07   |  BMI055_BW_32_HZ
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_BW 216

/* BMI055_GYRO_GET_PMU_EXT_TRI_SELECT
 *                 - This API used to get the status of
 *                   External Trigger selection in the register 0x12h bits from 4 to 5
 *
 *   ioctl-argument: v_pwu_ext_tri_select_u8 : The value of External Trigger selection
 *                   v_pwu_ext_tri_select_u8	 |	Trigger source
 *                   --------------------------|-------------------------
 *                   0x00                 |      No
 *                   0x01                 |   INT1 pin
 *                   0x02                 |   INT2 pin
 *                   0x03                 |  SDO pin(SPI3 mode)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_PMU_EXT_TRI_SELECT 217

/* BMI055_GYRO_SET_PMU_EXT_TRI_SELECT
 *                 - This API used to set the status of
 *                   External Trigger selection in the register 0x12h bits from 4 to 5
 *
 *   ioctl-argument: v_pwu_ext_tri_select_u8 : The value of External Trigger selection
 *                   v_pwu_ext_tri_select_u8	 |	Trigger source
 *                   --------------------------|-------------------------
 *                   0x00                 |      No
 *                   0x01                 |   INT1 pin
 *                   0x02                 |   INT2 pin
 *                   0x03                 |  SDO pin(SPI3 mode)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_PMU_EXT_TRI_SELECT 218

/* BMI055_GYRO_GET_HIGH_BW
 *                 - This API is used to get data high bandwidth
 *                   in the register 0x13 bit 7
 *
 *   ioctl-argument: v_high_bw_u8 : The value of high bandwidth
 *                   value   |  Description
 *                   ---------|--------------
 *                   1     | unfiltered
 *                   0     | filtered
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_HIGH_BW 219

/* BMI055_GYRO_SET_HIGH_BW
 *                 - This API is used to set data high bandwidth
 *                   in the register 0x13 bit 7
 *
 *   ioctl-argument: v_high_bw_u8 : The value of high bandwidth
 *                   value   |  Description
 *                   ---------|--------------
 *                   1     | unfiltered
 *                   0     | filtered
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_HIGH_BW 220

/* BMI055_GYRO_GET_SHADOW_DIS
 *                 - This API is used to get the shadow dis
 *                   in the register 0x13 bit 6
 *
 *   ioctl-argument: v_shadow_dis_u8 : The value of shadow dis
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_SHADOW_DIS 221

/* BMI055_GYRO_SET_SHADOW_DIS
 *                 - This API is used to set the shadow dis
 *                   in the register 0x13 bit 6
 *
 *   ioctl-argument: v_shadow_dis_u8 : The value of shadow dis
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SHADOW_DIS 222

/* BMI055_GYRO_SET_SOFT_RST
 *                 - This API is used to set the shadow dis
 *                   in the register 0x13 bit 6
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SOFT_RST 223

/* BMI055_GYRO_GET_DATA_ENABLE
 *                 - This API is used to get the data(data_enable)
 *                   interrupt enable bits of the sensor in the registers 0x15 bit 7
 *
 *   ioctl-argument: v_data_enable_u8 : The value of data enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_DATA_ENABLE 224

/* BMI055_GYRO_SET_DATA_ENABLE
 *                 - This API is used to set the data(data_enable)
 *                   interrupt enable bits of the sensor in the registers 0x15 bit 7
 *
 *   ioctl-argument: v_data_enable_u8 : The value of data enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_DATA_ENABLE 225

/* BMI055_GYRO_GET_FIFO_ENABLE
 *                 - This API is used to get the fifo(fifo_enable)
 *                   interrupt enable bits of the sensor in the registers 0x15 bit 6
 *
 *   ioctl-argument: v_fifo_enable_u8 : The value of  fifo enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_ENABLE 226

/* BMI055_GYRO_SET_FIFO_ENABLE
 *                 - This API is used to set the fifo(fifo_enable)
 *                   interrupt enable bits of the sensor in the registers 0x15 bit 6
 *
 *   ioctl-argument: v_fifo_enable_u8 : The value of  fifo enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FIFO_ENABLE 227

/* BMI055_GYRO_GET_AUTO_OFFSET_ENABLE
 *                 - This API is used to get
 *                   the auto offset(auto_offset_enable) interrupt enable bits of
 *                   the sensor in the registers 0x15 bit 3
 *
 *   ioctl-argument: v_offset_enable_u8 : The value of offset enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_AUTO_OFFSET_ENABLE 228

/* BMI055_GYRO_SET_AUTO_OFFSET_ENABLE
 *                 - This API is used to set
 *                   the auto offset(auto_offset_enable) interrupt enable bits of
 *                   the sensor in the registers 0x15 bit 3
 *
 *   ioctl-argument: v_offset_enable_u8 : The value of offset enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_AUTO_OFFSET_ENABLE 229

/* BMI055_GYRO_GET_INTR_OUTPUT_TYPE
 *                 - This API is used to get
 *                   the output type status in the register 0x16.
 *
 *                   NOTE: INT1 -> bit 1
 *
 *                   NOTE: INT2 -> bit 3
 *
 *   ioctl-argument: v_param_u8: The value of output type selection number
 *                   v_param_u8| output type
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1
 *                   1       |   BMI055_INTR2
 *
 *                   v_intr_output_type_u8: The value of output type
 *                   value    |  output
 *                   -----------|-------------
 *                   1       | open drain
 *                   0       | push pull
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_OUTPUT_TYPE 230

/* BMI055_GYRO_SET_INTR_OUTPUT_TYPE
 *                 - This API is used to set
 *                   the output type status in the register 0x16.
 *
 *                   NOTE: INT1 -> bit 1
 *
 *                   NOTE: INT2 -> bit 3
 *
 *   ioctl-argument: v_param_u8: The value of output type selection number
 *                   v_param_u8| output type
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1
 *                   1       |   BMI055_INTR2
 *
 *                   v_intr_output_type_u8: The value of output type
 *                   value    |  output
 *                   -----------|-------------
 *                   1       | open drain
 *                   0       | push pull
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR_OUTPUT_TYPE 231

/* BMI055_GYRO_GET_INTR_LEVEL
 *                 - This API is used to get
 *                   Active Level status in the register 0x16
 *
 *                   NOTE: INT1 -> bit 0
 *
 *                   NOTE: INT2 -> bit 2
 *
 *   ioctl-argument: v_param_u8: The value of Active Level selection number
 *                   v_param_u8| Active Level
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1
 *                   1       |   BMI055_INTR2
 *
 *                   v_intr_level_u8: The  value of Active Level status value
 *                   value    |  Active Level
 *                   -----------|-------------
 *                   1       | Active HIGH
 *                   0       | Active LOW
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_LEVEL 232

/* BMI055_GYRO_SET_INTR_LEVEL
 *                 - This API is used to set
 *                   Active Level status in the register 0x16
 *
 *                   NOTE: INT1 -> bit 0
 *
 *                   NOTE: INT2 -> bit 2
 *
 *   ioctl-argument: v_param_u8: The value of Active Level selection number
 *                   v_param_u8| Active Level
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1
 *                   1       |   BMI055_INTR2
 *
 *                   v_intr_level_u8: The  value of Active Level status value
 *                   value    |  Active Level
 *                   -----------|-------------
 *                   1       | Active HIGH
 *                   0       | Active LOW
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR_LEVEL 233

/* BMI055_GYRO_GET_INTR1_HIGHRATE
 *                 - This API is used to get
 *                   the high rate(int1_high) interrupt1 enable bits of
 *                   the sensor in the registers 0x17 bit 3
 *
 *   ioctl-argument: v_intr1_u8 : The value of interrupt1 high_rate enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR1_HIGHRATE 234

/* BMI055_GYRO_SET_INTR1_HIGHRATE
 *                 - This API is used to set
 *                   the high rate(int1_high) interrupt1 enable bits of
 *                   the sensor in the registers 0x17 bit 3
 *
 *   ioctl-argument: v_intr1_u8 : The value of interrupt1 high_rate enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR1_HIGHRATE 235

/* BMI055_GYRO_GET_INTR1_ANY_MOTION
 *                 - This API is used to get
 *                   the any motion(int1_any) interrupt1 enable bits of
 *                   the sensor in the registers 0x17 bit 1
 *
 *   ioctl-argument: v_int1r_any_motion_u8 : The value of any motion interrupt1
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR1_ANY_MOTION 236

/* BMI055_GYRO_SET_INTR1_ANY_MOTION
 *                 - This API is used to set
 *                   the any motion(int1_any) interrupt1 enable bits of
 *                   the sensor in the registers 0x17 bit 1
 *
 *   ioctl-argument: v_int1r_any_motion_u8 : The value of any motion interrupt1
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR1_ANY_MOTION 237

/* BMI055_GYRO_GET_INTR_DATA
 *                 - This API is used to get
 *                   the data interrupt1 and interrupt2(int1_data and int2_data)
 *                   in the register 0x18
 *
 *                   NOTE: INT1 -> bit 0
 *
 *                   NOTE: INT2 -> bit 7
 *
 *   ioctl-argument: v_axis_u8: data interrupt selection
 *                   v_axis_u8 | Data interrupt
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1_DATA
 *                   1       |   BMI055_INTR2_DATA
 *
 *                   v_intr_data_u8: The value of data interrupt1 or interrupt2
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_DATA 238

/* BMI055_GYRO_SET_INTR_DATA
 *                 - This API is used to set
 *                   the data interrupt1 and interrupt2(int1_data and int2_data)
 *                   in the register 0x18
 *
 *                   NOTE: INT1 -> bit 0
 *
 *                   NOTE: INT2 -> bit 7
 *
 *   ioctl-argument: v_axis_u8: data interrupt selection
 *                   v_axis_u8 | Data interrupt
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1_DATA
 *                   1       |   BMI055_INTR2_DATA
 *
 *                   v_intr_data_u8: The value of data interrupt1 or interrupt2
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR_DATA 239

/* BMI055_GYRO_GET_INTR2_OFFSET
 *                 - This API is used to get
 *                   the fast offset(intr2_fast_offset) and auto offset(intr2_auto_offset)
 *                   of interrupt2 in the register 0x18
 *
 *                   NOTE: int2_fast_offset -> bit 6
 *
 *                   NOTE: int2_auto_offset -> bit 4
 *
 *   ioctl-argument: v_axis_u8: The value of fast/auto offset interrupts selection
 *                   v_axis_u8 | Data interrupt
 *                   ------------|--------------
 *                   1       |   BMI055_FAST_OFFSET
 *                   2       |   BMI055_AUTO_OFFSET
 *
 *                   v_intr2_offset_u8: The value of fast/auto offset enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR2_OFFSET 240

/* BMI055_GYRO_SET_INTR2_OFFSET
 *                 - This API is used to set
 *                   the fast offset(intr2_fast_offset) and auto offset(intr2_auto_offset)
 *                   of interrupt2 in the register 0x18
 *
 *                   NOTE: int2_fast_offset -> bit 6
 *
 *                   NOTE: int2_auto_offset -> bit 4
 *
 *   ioctl-argument: v_axis_u8: The value of fast/auto offset interrupts selection
 *                   v_axis_u8 | Data interrupt
 *                   ------------|--------------
 *                   1       |   BMI055_FAST_OFFSET
 *                   2       |   BMI055_AUTO_OFFSET
 *
 *                   v_intr2_offset_u8: The value of fast/auto offset enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR2_OFFSET 241

/* BMI055_GYRO_GET_INTR_OFFSET
 *                 - This API is used to get
 *                   the fast offset(int1_fast_offset) and auto offset(int1_auto_offset)
 *                   of interrupt1 in the register 0x18
 *
 *                   NOTE: int1_fast_offset -> bit 1
 *
 *                   NOTE: int1_auto_offset -> bit 3
 *
 *   ioctl-argument: v_axis_u8: The value of fast/auto offset interrupts selection
 *                   v_axis_u8 | Data interrupt
 *                   ------------|--------------
 *                   1       |   BMI055_FAST_OFFSET
 *                   2       |   BMI055_AUTO_OFFSET
 *
 *                   v_intr1_offset_u8: The value of fast/auto offset
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR_OFFSET 242

/* BMI055_GYRO_SET_INTR1_OFFSET
 *                 - This API is used to set
 *                   the fast offset(int1_fast_offset) and auto offset(int1_auto_offset)
 *                   of interrupt1 in the register 0x18
 *
 *                   NOTE: int1_fast_offset -> bit 1
 *
 *                   NOTE: int1_auto_offset -> bit 3
 *
 *   ioctl-argument: v_axis_u8: The value of fast/auto offset interrupts selection
 *                   v_axis_u8 | Data interrupt
 *                   ------------|--------------
 *                   1       |   BMI055_FAST_OFFSET
 *                   2       |   BMI055_AUTO_OFFSET
 *
 *                   v_intr1_offset_u8: The value of fast/auto offset
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR1_OFFSET 243

/* BMI055_GYRO_GET_INTR2_FIFO
 *                 - This API is used to get
 *                   the fifo(int2_fifo) interrupt2 enable bits of
 *                   the sensor in the registers 0x18 bit 5
 *
 *   ioctl-argument: v_intr_fifo_u8 : The interrupt2 fifo value
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR2_FIFO 244

/* BMI055_GYRO_GET_INTR1_FIFO
 *                 - This API is used to get
 *                   the fifo(int1_fifo) interrupt1 enable bits of
 *                   the sensor in the registers 0x18 bit 5
 *
 *   ioctl-argument: v_intr_fifo_u8 : The interrupt1 fifo value
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR1_FIFO 245

/* BMI055_GYRO_SET_INTR_FIFO
 *                 - This API is used to set the value of
 *                   the fifo interrupt1 and interrupt2(int1_fifo and int2_fifo)
 *                   in the register 0x18
 *
 *                   NOTE: int1_fifo -> bit 2
 *
 *                   NOTE: int2_fifo -> bit 5
 *
 *   ioctl-argument: v_axis_u8: The value of fifo interrupts selection
 *                   v_axis_u8 | fifo interrupt
 *                   ------------|--------------
 *                   0       |   BMI055_INTR1
 *                   1       |   BMI055_INTR2
 *
 *                   v_intr_fifo_u8: the value of int1_fifo/int2_fifo enable/disable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR_FIFO 246

/* BMI055_GYRO_GET_INTR2_HIGHRATE
 *                 - This API is used to get
 *                   the high rate(int2_high_rate) interrupt2 enable bits of
 *                   the sensor in the registers 0x19 bit 3
 *
 *   ioctl-argument: v_intr2_highrate_u8 : The interrupt2 high_rate value
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR2_HIGHRATE 247

/* BMI055_GYRO_SET_INTR2_HIGHRATE
 *                 - This API is used to set
 *                   the high rate(int2_high_rate) interrupt2 enable bits of
 *                   the sensor in the registers 0x19 bit 3
 *
 *   ioctl-argument: v_intr2_highrate_u8 : The interrupt2 high_rate value
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR2_HIGHRATE 248

/* BMI055_GYRO_GET_INTR2_ANY_MOTION
 *                 - This API is used to get
 *                   the any motion(int2_any_motion) interrupt2 enable bits of
 *                   the sensor in the registers 0x19 bit 1
 *
 *   ioctl-argument: v_intr2_any_motion_u8 : The value of interrupt2 any_motion
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_INTR2_ANY_MOTION 249

/* BMI055_GYRO_SET_INTR2_ANY_MOTION
 *                 - This API is used to set
 *                   the any motion(int2_any_motion) interrupt2 enable bits of
 *                   the sensor in the registers 0x19 bit 1
 *
 *   ioctl-argument: v_intr2_any_motion_u8 : The value of interrupt2 any_motion
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_INTR2_ANY_MOTION 250

/* BMI055_GYRO_GET_OFFSET_UNFILT
 *                 - This API is used to get
 *                   the slow offset and fast offset unfilt data in the register 0x1A and 1B
 *
 *                   NOTE: slow_offset_unfilt -> 0x1A bit 5
 *
 *                   NOTE: fast_offset_unfilt -> 0x1B bit 7
 *
 *   ioctl-argument: v_param_u8: The value of fast/slow offset unfilt data selection
 *                   v_param_u8 | offset selection
 *                   ------------|--------------
 *                   0       |   BMI055_SLOW_OFFSET
 *                   1       |   BMI055_FAST_OFFSET
 *
 *                   v_offset_unfilt_u8: The value of fast/slow offset unfilt data
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_OFFSET_UNFILT 251

/* BMI055_GYRO_SET_OFFSET_UNFILT
 *                 - This API is used to set
 *                   the slow offset and fast offset unfilt data in the register 0x1A and 1B
 *
 *                   NOTE: slow_offset_unfilt -> 0x1A bit 5
 *
 *                   NOTE: fast_offset_unfilt -> 0x1B bit 7
 *
 *   ioctl-argument: v_param_u8: The value of fast/slow offset unfilt data selection
 *                   v_param_u8 | offset selection
 *                   ------------|--------------
 *                   0       |   BMI055_SLOW_OFFSET
 *                   1       |   BMI055_FAST_OFFSET
 *
 *                   v_offset_unfilt_u8: The value of fast/slow offset unfilt data
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_OFFSET_UNFILT 252

/* BMI055_GYRO_GET_UNFILT_DATA
 *                 - This API is used to get
 *                   the any motion  and high rate unfilt data in the register 0x1A
 *
 *                   NOTE: any_unfilt_data -> bit 1
 *
 *                   NOTE: high_unfilt_data -> bit 3
 *
 *   ioctl-argument: v_param_u8: The value of any/high offset unfilt data selection
 *                   v_param_u8 | offset selection
 *                   ------------|--------------
 *                   1       |   BMI055_HIGHRATE_UNFILT_DATA
 *                   3       |   BMI055_ANY_MOTION_UNFILT_DATA
 *
 *                   v_unfilt_data_u8: The value of any/high unfilt data
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_UNFILT_DATA 253

/* BMI055_GYRO_SET_UNFILT_DATA
 *                 - This API is used to set
 *                   the any motion  and high rate unfilt data in the register 0x1A
 *
 *                   NOTE: any_unfilt_data -> bit 1
 *
 *                   NOTE: high_unfilt_data -> bit 3
 *
 *   ioctl-argument: v_param_u8: The value of any/high offset unfilt data selection
 *                   v_param_u8 | offset selection
 *                   ------------|--------------
 *                   1       |   BMI055_HIGHRATE_UNFILT_DATA
 *                   3       |   BMI055_ANY_MOTION_UNFILT_DATA
 *
 *                   v_unfilt_data_u8: The value of any/high unfilt data
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_UNFILT_DATA 254

/* BMI055_GYRO_GET_ANY_MOTION_THRES
 *                 - This API is used to get Any motion Threshold
 *                   in the register 0x1B bit from 0 to 6
 *
 *   ioctl-argument: v_any_motion_thres_u8 : The value of any_motion Threshold
 *                   @note Any motion threshold can be calculate using
 *                   @note ((1+ v_any_motion_thres_u8)*16LSB)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_ANY_MOTION_THRES 255

/* BMI055_GYRO_SET_ANY_MOTION_THRES
 *                 - This API is used to set Any motion Threshold
 *                   in the register 0x1B bit from 0 to 6
 *
 *   ioctl-argument: v_any_motion_thres_u8 : The value of any_motion Threshold
 *                   @note Any motion threshold can be calculate using
 *                   @note ((1+ v_any_motion_thres_u8)*16LSB)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_ANY_MOTION_THRES 256

/* BMI055_GYRO_GET_AWAKE_DURN
 *                 - This API is used to get the awake Duration
 *                   in the register 0x1C bit 6 and 7
 *
 *   ioctl-argument: v_awake_durn_u8 : The value of awake Duration
 *                   value   |  Duration
 *                   ---------|-----------
 *                   0x00   | 8 samples
 *                   0x01   | 16 samples
 *                   0x02   | 32 samples
 *                   0x03   | 64 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_AWAKE_DURN 257

/* BMI055_GYRO_SET_AWAKE_DURN
 *                 - This API is used to set the awake Duration
 *                   in the register 0x1C bit 6 and 7
 *
 *   ioctl-argument: v_awake_durn_u8 : The value of awake Duration
 *                   value   |  Duration
 *                   ---------|-----------
 *                   0x00   | 8 samples
 *                   0x01   | 16 samples
 *                   0x02   | 32 samples
 *                   0x03   | 64 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_AWAKE_DURN 258

/* BMI055_GYRO_GET_ANY_MOTION_DURN_SAMPLE
 *                 - This API is used to get
 *                   the any motion Duration samples in the register 0x1C bit 4 and 5
 *
 *   ioctl-argument: v_durn_sample_u8 : The value of any motion duration samples
 *                   value   |  Samples
 *                   ---------|-----------
 *                   0x00   | 4 samples
 *                   0x01   | 8 samples
 *                   0x02   | 12 samples
 *                   0x03   | 16 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_ANY_MOTION_DURN_SAMPLE 259

/* BMI055_GYRO_SET_ANY_MOTION_DURN_SAMPLE
 *                 - This API is used to set
 *                   the any motion Duration samples in the register 0x1C bit 4 and 5
 *
 *   ioctl-argument: v_durn_sample_u8 : The value of any motion duration samples
 *                   value   |  Samples
 *                   ---------|-----------
 *                   0x00   | 4 samples
 *                   0x01   | 8 samples
 *                   0x02   | 12 samples
 *                   0x03   | 16 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_ANY_MOTION_DURN_SAMPLE 260

/* BMI055_GYRO_GET_ANY_MOTION_ENABLE_AXIS
 *                 - This API is used to get the status of
 *                   Any motion interrupt axis(X,Y,Z) enable channel
 *
 *                   NOTE: BMI055_X_AXIS -> bit 0
 *
 *                   NOTE: BMI055_Y_AXIS -> bit 1
 *
 *                   NOTE: BMI055_Z_AXIS -> bit 2
 *
 *   ioctl-argument: v_channel_u8 : The value of Any Enable channel number
 *                   v_channel_u8 | axis
 *                   --------------|--------------
 *                   0         | BMI055_X_AXIS
 *                   1         | BMI055_Y_AXIS
 *                   2         | BMI055_Z_AXIS
 *
 *                   v_any_motion_axis_u8: The value of Any motion axis enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_ANY_MOTION_ENABLE_AXIS 261

/* BMI055_GYRO_SET_ANY_MOTION_ENABLE_AXIS
 *                 - This API is used to set the status of
 *                   Any motion interrupt axis(X,Y,Z) enable channel
 *
 *                   NOTE: BMI055_X_AXIS -> bit 0
 *
 *                   NOTE: BMI055_Y_AXIS -> bit 1
 *
 *                   NOTE: BMI055_Z_AXIS -> bit 2
 *
 *   ioctl-argument: v_channel_u8 : The value of Any Enable channel number
 *                   v_channel_u8 | axis
 *                   --------------|--------------
 *                   0         | BMI055_X_AXIS
 *                   1         | BMI055_Y_AXIS
 *                   2         | BMI055_Z_AXIS
 *
 *                   v_any_motion_axis_u8: The value of Any motion axis enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_ANY_MOTION_ENABLE_AXIS 262

/* BMI055_GYRO_GET_FIFO_WM_ENABLE
 *                 - This API is used to get
 *                   the status of fifo water mark in the register 0x1E bit 7
 *
 *   ioctl-argument: v_fifo_wm_enable_u8 : The value of fifo water mark enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_WM_ENABLE 263

/* BMI055_GYRO_SET_FIFO_WM_ENABLE
 *                 - This API is used to set
 *                   the status of fifo water mark in the register 0x1E bit 7
 *
 *   ioctl-argument: v_fifo_wm_enable_u8 : The value of fifo water mark enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FIFO_WM_ENABLE 264

/* BMI055_GYRO_SET_RST_INTR
 *                 - This API is used to set the Interrupt Reset
 *                   in the register 0x21 bit 7
 *
 *   ioctl-argument: v_rst_int_u8: the value of reset interrupt
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_RST_INTR 265

/* BMI055_GYRO_SET_OFFSET_RST
 *                 - This API is used to set the offset Reset
 *                   in the register 0x21 bit 6
 *
 *   ioctl-argument: v_offset_rst_u8: the value of reset offset
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_OFFSET_RST 266

/* BMI055_GYRO_GET_LATCH_STAT
 *                 - This API is used to get the Latch Status
 *                   in the register 0x21 bit 4
 *
 *   ioctl-argument: v_latch_stat_u8 : The value of latch status
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_LATCH_STAT 267

/* BMI055_GYRO_SET_LATCH_STAT
 *                 - This API is used to set the Latch Status
 *                   in the register 0x21 bit 4
 *
 *   ioctl-argument: v_latch_stat_u8 : The value of latch status
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_LATCH_STAT 268

/* BMI055_GYRO_GET_LATCH_INTR
 *                 - This API is used to get the Latch interrupt
 *                   in the register 0x21 bit from 0 to 3
 *
 *   ioctl-argument: v_latch_intr_u8 : The value of latch interrupt
 *                   Latch Interrupt        |  Value
 *                   ----------------------------|-----------------
 *                   BMI055_NON_LATCH            | 0x00
 *                   BMI055_LATCH_250_MS         | 0x01
 *                   BMI055_LATCH_500_MS         | 0x02
 *                   BMI055_LATCH_1_SEC          | 0x03
 *                   BMI055_LATCH_2_SEC          | 0x04
 *                   BMI055_LATCH_4_SEC          | 0x05
 *                   BMI055_LATCH_8_SEC          | 0x06
 *                   BMI055_LATCH_LATCHED        | 0x07
 *                   BMI055_LATCH_NON_LATCHED    | 0x08
 *                   BMI055_LATCH_250_MICRO_SEC  | 0x09
 *                   BMI055_LATCH_500_MICRO_SEC  | 0x0A
 *                   BMI055_LATCH_1_MILLI_SEC    | 0x0B
 *                   BMI055_LATCH_12.5_MILLI_SEC | 0x0C
 *                   BMI055_LATCH_25_MILLI_SEC   | 0x0D
 *                   BMI055_LATCH_50_MILLI_SEC   | 0x0E
 *                   BMI055_LATCH_LATCHED        | 0x0F
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_LATCH_INTR 269

/* BMI055_GYRO_SET_LATCH_INTR
 *                 - This API is used to set the Latch interrupt
 *                   in the register 0x21 bit from 0 to 3
 *
 *   ioctl-argument: v_latch_intr_u8 : The value of latch interrupt
 *                   Latch Interrupt        |  Value
 *                   ----------------------------|-----------------
 *                   BMI055_NON_LATCH            | 0x00
 *                   BMI055_LATCH_250_MS         | 0x01
 *                   BMI055_LATCH_500_MS         | 0x02
 *                   BMI055_LATCH_1_SEC          | 0x03
 *                   BMI055_LATCH_2_SEC          | 0x04
 *                   BMI055_LATCH_4_SEC          | 0x05
 *                   BMI055_LATCH_8_SEC          | 0x06
 *                   BMI055_LATCH_LATCHED        | 0x07
 *                   BMI055_LATCH_NON_LATCHED    | 0x08
 *                   BMI055_LATCH_250_MICRO_SEC  | 0x09
 *                   BMI055_LATCH_500_MICRO_SEC  | 0x0A
 *                   BMI055_LATCH_1_MILLI_SEC    | 0x0B
 *                   BMI055_LATCH_12.5_MILLI_SEC | 0x0C
 *                   BMI055_LATCH_25_MILLI_SEC   | 0x0D
 *                   BMI055_LATCH_50_MILLI_SEC   | 0x0E
 *                   BMI055_LATCH_LATCHED        | 0x0F
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_LATCH_INTR 270

/* BMI055_GYRO_GET_HIGHRATE_HYST
 *                 - This API is used to get the status of High
 *                   Hysteresis of X,Y,Z axis in the registers 0x22,0x24 and 0x26
 *
 *                   NOTE: X_AXIS - 0x22 bit 6 and 7
 *
 *                   NOTE: Y_AXIS - 0x24 bit 6 and 7
 *
 *                   NOTE: Z_AXIS - 0x26 bit 6 and 7
 *
 *   ioctl-argument: v_channel_u8: The value of high Hysteresis channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_highrate_hyst_u8: The value of high Hysteresis
 *                   @note High hysteresis can be calculated by
 *                   @note High_hyst = ((255+256 *v_highrate_hyst_u8) * 4LSB)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_HIGHRATE_HYST 271

/* BMI055_GYRO_SET_HIGHRATE_HYST
 *                 - This API is used to set the status of High
 *                   Hysteresis of X,Y,Z axis in the registers 0x22,0x24 and 0x26
 *
 *                   NOTE: X_AXIS - 0x22 bit 6 and 7
 *
 *                   NOTE: Y_AXIS - 0x24 bit 6 and 7
 *
 *                   NOTE: Z_AXIS - 0x26 bit 6 and 7
 *
 *   ioctl-argument: v_channel_u8: The value of high Hysteresis channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_highrate_hyst_u8: The value of high Hysteresis
 *                   @note High hysteresis can be calculated by
 *                   @note High_hyst = ((255+256 *v_highrate_hyst_u8) * 4LSB)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_HIGHRATE_HYST 272

/* BMI055_GYRO_GET_HIGHRATE_THRES
 *                 - This API is used to get the value of High rate
 *                   Threshold of X,Y,Z axis in the registers 0x22, 0x24 and 0x26
 *
 *                   NOTE: X_AXIS - 0x22 bit from 1 to 5
 *
 *                   NOTE: Y_AXIS - 0x24 bit from 1 to 5
 *
 *                   NOTE: Z_AXIS - 0x26 bit from 1 to 5
 *
 *   ioctl-argument: v_channel_u8 : The value of high threshold channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_highrate_thres_u8: the high threshold value
 *                   @note High Threshold can be calculated by
 *                   @note High_thres = ((255+256 *v_highrate_thres_u8) * 4LSB)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_HIGHRATE_THRES 273

/* BMI055_GYRO_SET_HIGHRATE_THRES
 *                 - This API is used to set the value of High rate
 *                   Threshold of X,Y,Z axis in the registers 0x22, 0x24 and 0x26
 *
 *                   NOTE: X_AXIS - 0x22 bit from 1 to 5
 *
 *                   NOTE: Y_AXIS - 0x24 bit from 1 to 5
 *
 *                   NOTE: Z_AXIS - 0x26 bit from 1 to 5
 *
 *   ioctl-argument: v_channel_u8 : The value of high threshold channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_highrate_thres_u8: the high threshold value
 *                   @note High Threshold can be calculated by
 *                   @note High_thres = ((255+256 *v_highrate_thres_u8) * 4LSB)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_HIGHRATE_THRES 274

/* BMI055_GYRO_GET_HIGHRATE_ENABLE_AXIS
 *                 - This API is used to get the status of High Enable
 *                   Channel X,Y,Z in the registers 0x22, 0x24 and 0x26
 *
 *                   NOTE: X_AXIS - 0x22 bit 0
 *
 *                   NOTE: Y_AXIS - 0x24 bit 0
 *
 *                   NOTE: Z_AXIS - 0x26 bit 0
 *
 *   ioctl-argument: v_channel_u8 : The value of high enable channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_enable_u8: The value of high axis enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_HIGHRATE_ENABLE_AXIS 275

/* BMI055_GYRO_SET_HIGHRATE_ENABLE_AXIS
 *                 - This API is used to set the status of High Enable
 *                   Channel X,Y,Z in the registers 0x22, 0x24 and 0x26
 *
 *                   NOTE: X_AXIS - 0x22 bit 0
 *
 *                   NOTE: Y_AXIS - 0x24 bit 0
 *
 *                   NOTE: Z_AXIS - 0x26 bit 0
 *
 *   ioctl-argument: v_channel_u8 : The value of high enable channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_enable_u8: The value of high axis enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_HIGHRATE_ENABLE_AXIS 276

/* BMI055_GYRO_GET_HIGHRATE_DURN_AXIS
 *                 - This API is used to get the status
 *                   of High duration of X,Y,Z axis in
 *                   the registers 0x23, 0x25 and 0x27
 *
 *                   NOTE: X_AXIS - 0x23 bit form 0 to 7
 *
 *                   NOTE: Y_AXIS - 0x25 bit form 0 to 7
 *
 *                   NOTE: Z_AXIS - 0x27 bit form 0 to 7
 *
 *   ioctl-argument: v_channel_u8: The value of High Duration channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_highrate_durn_axis_u8: The value of high duration
 *                   @note High rate duration can be calculated by using the formula
 *                   @note High_durn = ((1+v_highrate_durn_axis_u8)*2.5ms)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_HIGHRATE_DURN_AXIS 277

/* BMI055_GYRO_SET_HIGHRATE_DURN_AXIS
 *                 - This API is used to set the value
 *                   of High duration of X,Y,Z axis in
 *                   the registers 0x23, 0x25 and 0x27
 *
 *                   NOTE: X_AXIS - 0x23 bit form 0 to 7
 *
 *                   NOTE: Y_AXIS - 0x25 bit form 0 to 7
 *
 *                   NOTE: Z_AXIS - 0x27 bit form 0 to 7
 *
 *   ioctl-argument: v_channel_u8: The value of High Duration channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_highrate_durn_axis_u8: The value of high duration
 *                   @note High rate duration can be calculated by using the formula
 *                   @note High_durn = ((1+v_highrate_durn_axis_u8)*2.5ms)
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_HIGHRATE_DURN_AXIS 278

/* BMI055_GYRO_GET_SLOW_OFFSET_THRES
 *                 - This API is used to get Slow Offset Threshold
 *                   status in the register 0x31 bit 6 and 7
 *
 *   ioctl-argument: v_offset_thres_u8 : The value of slow offset Threshold
 *                   value    |   threshold
 *                   ----------|-------------
 *                   0x00   | 0.1 degree/sec
 *                   0x01   | 0.2 degree/sec
 *                   0x02   | 0.5 degree/sec
 *                   0x03   | 1 degree/sec
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_SLOW_OFFSET_THRES 279

/* BMI055_GYRO_SET_SLOW_OFFSET_THRES
 *                 - This API is used to set Slow Offset Threshold
 *                   status in the register 0x31 bit 6 and 7
 *
 *   ioctl-argument: v_offset_thres_u8 : The value of slow offset Threshold
 *                   value    |   threshold
 *                   ----------|-------------
 *                   0x00   | 0.1 degree/sec
 *                   0x01   | 0.2 degree/sec
 *                   0x02   | 0.5 degree/sec
 *                   0x03   | 1 degree/sec
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SLOW_OFFSET_THRES 280

/* BMI055_GYRO_GET_SLOW_OFFSET_DURN
 *                 - This API is used to get Slow Offset duration
 *                   status in the register 0x31 bit 4,5 and 6
 *
 *   ioctl-argument: v_offset_durn_u8 : The value of Slow Offset duration
 *                   value    |  Duration
 *                   -----------|-----------
 *                   0x00    | 40ms
 *                   0x01    | 80ms
 *                   0x02    | 160ms
 *                   0x03    | 320ms
 *                   0x04    | 640ms
 *                   0x05    | 1280ms
 *                   0x06    | unused
 *                   0x07    | unused
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_SLOW_OFFSET_DURN 281

/* BMI055_GYRO_SET_SLOW_OFFSET_DURN
 *                 - This API is used to set Slow Offset duration
 *                   status in the register 0x31 bit 4,5 and 6
 *
 *   ioctl-argument: v_offset_durn_u8 : The value of Slow Offset duration
 *                   value    |  Duration
 *                   -----------|-----------
 *                   0x00    | 40ms
 *                   0x01    | 80ms
 *                   0x02    | 160ms
 *                   0x03    | 320ms
 *                   0x04    | 640ms
 *                   0x05    | 1280ms
 *                   0x06    | unused
 *                   0x07    | unused
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SLOW_OFFSET_DURN 282

/* BMI055_GYRO_GET_SLOW_OFFSET_ENABLE_AXIS
 *                 - This API is used to get Slow Offset Enable channel
 *                   X,Y,Z in the register 0x31
 *
 *                   NOTE: X_AXIS -> bit 0
 *
 *                   NOTE: Y_AXIS -> bit 1
 *
 *                   NOTE: Z_AXIS -> bit 2
 *
 *   ioctl-argument: v_channel_u8: The value of slow offset channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_slow_offset_u8: The slow offset value
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_SLOW_OFFSET_ENABLE_AXIS 283

/* BMI055_GYRO_SET_SLOW_OFFSET_ENABLE_AXIS
 *                 - This API is used to set Slow Offset Enable channel
 *                   X,Y,Z in the register 0x31
 *
 *                   NOTE: X_AXIS -> bit 0
 *
 *                   NOTE: Y_AXIS -> bit 1
 *
 *                   NOTE: Z_AXIS -> bit 2
 *
 *   ioctl-argument: v_channel_u8: The value of slow offset channel number
 *                   v_channel_u8  |    value
 *                   --------------|--------------
 *                   BMI055_X_AXIS |   0
 *                   BMI055_Y_AXIS |   1
 *                   BMI055_Z_AXIS |   2
 *
 *                   v_slow_offset_u8: The slow offset value
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SLOW_OFFSET_ENABLE_AXIS 284

/* BMI055_GYRO_GET_OFFSET_WORD_LENGTH
 *                 - This API is used to get
 *                   Fast Offset WordLength and Auto Offset WordLength in the register 0x32
 *
 *                   NOTE: fast_offset_wordlength -> bit 4 and 5
 *
 *                   NOTE: auto_offset_wordlength -> bit 6 and 7
 *
 *   ioctl-argument: v_channel_u8: The value of WordLengthchannel number
 *                   v_channel_u8          |    value
 *                   ----------------------|--------------
 *                   BMI055_AUTO_OFFSET_WL |   0
 *                   BMI055_FAST_OFFSET_WL |   1
 *
 *                   v_offset_word_length_u8: The value of offset word length
 *                   value    |  word length
 *                   ----------|--------------
 *                   0x00    | 32 samples
 *                   0x01    | 64 samples
 *                   0x02    | 128 samples
 *                   0x03    | 256 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_OFFSET_WORD_LENGTH 285

/* BMI055_GYRO_SET_OFFSET_WORD_LENGTH
 *                 - This API is used to set
 *                   Fast Offset WordLength and Auto Offset WordLength in the register 0x32
 *
 *                   NOTE: fast_offset_wordlength -> bit 4 and 5
 *
 *                   NOTE: auto_offset_wordlength -> bit 6 and 7
 *
 *   ioctl-argument: v_channel_u8: The value of WordLengthchannel number
 *                   v_channel_u8          |    value
 *                   ----------------------|--------------
 *                   BMI055_AUTO_OFFSET_WL |   0
 *                   BMI055_FAST_OFFSET_WL |   1
 *
 *                   v_offset_word_length_u8: The value of offset word length
 *                   value    |  word length
 *                   ----------|--------------
 *                   0x00    | 32 samples
 *                   0x01    | 64 samples
 *                   0x02    | 128 samples
 *                   0x03    | 256 samples
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_OFFSET_WORD_LENGTH 286

/* BMI055_GYRO_ENABLE_FAST_OFFSET
 *                 - This API is used to enable fast offset
 *                   in the register 0x32 bit 3 it is a write only register
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_ENABLE_FAST_OFFSET 287

/* BMI055_GYRO_GET_FAST_OFFSET_ENABLE_AXIS
 *                 - This API read the Fast offset enable
 *                   v_axis_u8(X,Y and Z) in the register 0x32
 *
 *                   NOTE: X_AXIS -> bit 0
 *
 *                   NOTE: Y_AXIS -> bit 1
 *
 *                   NOTE: Z_AXIS -> bit 2
 *
 *   ioctl-argument: v_fast_offset_u8: The value of fast offset enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FAST_OFFSET_ENABLE_AXIS 288

/* BMI055_GYRO_SET_FAST_OFFSET_ENABLE_AXIS
 *                 - This API set the Fast offset enable
 *                   v_axis_u8(X,Y and Z) in the register 0x32
 *
 *                   NOTE: X_AXIS -> bit 0
 *
 *                   NOTE: Y_AXIS -> bit 1
 *
 *                   NOTE: Z_AXIS -> bit 2
 *
 *   ioctl-argument: v_channel_u8: The value of fast offset channel select
 *                   v_channel_u8    |    value
 *                   ----------------|--------------
 *                   BMI055_X_AXIS   |   0
 *                   BMI055_Y_AXIS   |   1
 *                   BMI055_Z_AXIS   |   2
 *
 *                   v_fast_offset_u8: The value of fast offset enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FAST_OFFSET_ENABLE_AXIS 289

/* BMI055_GYRO_GET_NVM_REMAIN
 *                 - This API is used to get the status of nvm program
 *                   remain in the register 0x33 bit from 4 to 7
 *
 *   ioctl-argument: v_nvm_remain_u8: The value of nvm program
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  Do not trigger
 *                   0       |  Trigger
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_NVM_REMAIN 290

/* BMI055_GYRO_SET_NVM_LOAD
 *                 - This API is used to set the status of nvm program
 *                   remain in the register 0x33 bit from 4 to 7
 *
 *   ioctl-argument: v_nvm_load_u8: The value of nvm program
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  Do not trigger
 *                   0       |  Trigger
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_NVM_LOAD 291

/* BMI055_GYRO_GET_NVM_RDY
 *                 - This API is used to get the status of nvm
 *                   program in the register 0x33 bit 2
 *
 *   ioctl-argument: v_nvm_rdy_u8: The value of nvm program
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  NVM write is in progress
 *                   0       |  NVM is ready to accept a new write
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_NVM_RDY 292

/* BMI055_GYRO_SET_NVM_PROG_TRIG
 *                 - This API is used to set the status of nvm
 *                   ready in the register 0x33 bit 1
 *
 *   ioctl-argument: nvm_prog_trig: The value of nvm program
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  program seq in progress
 *                   0       |  program seq finished
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_NVM_PROG_TRIG 293

/* BMI055_GYRO_GET_NVM_PROG_MODE
 *                 - This API is used to get
 *                   the status of nvm program mode in the register 0x33 bit 0
 *
 *   ioctl-argument: nvm_prog_mode: The value of nvm program mode
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  Unlock
 *                   0       |  Lock nvm write
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_NVM_PROG_MODE 294

/* BMI055_GYRO_SET_NVM_PROG_MODE
 *                 - This API is used to set
 *                   the status of nvm program mode in the register 0x33 bit 0
 *
 *   ioctl-argument: nvm_prog_mode: The value of nvm program mode
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  Unlock
 *                   0       |  Lock nvm write
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_NVM_PROG_MODE 295

/* BMI055_GYRO_GET_I2C_WDT
 *                 - This API is used to get
 *                   the status of i2c wdt select and enable in the register 0x34
 *
 *                   NOTE: i2c_wdt_select -> bit 1
 *
 *                   NOTE: i2c_wdt_enable -> bit 2
 *
 *   ioctl-argument: v_channel_u8: The value of i2c wdt channel number
 *                   v_channel_u8            |    value
 *                   ------------------------|--------------
 *                   BMI055_I2C_WDT_ENABLE   |   1
 *                   BMI055_I2C_WDT_SELECT   |   0
 *
 *                   v_i2c_wdt_u8: The value of I2C enable and WDT select
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_I2C_WDT 296

/* BMI055_GYRO_SET_I2C_WDT
 *                 - This API is used to set
 *                   the status of i2c wdt select and enable in the register 0x34
 *
 *                   NOTE: i2c_wdt_select -> bit 1
 *
 *                   NOTE: i2c_wdt_enable -> bit 2
 *
 *   ioctl-argument: v_channel_u8: The value of i2c wdt channel number
 *                   v_channel_u8            |    value
 *                   ------------------------|--------------
 *                   BMI055_I2C_WDT_ENABLE   |   1
 *                   BMI055_I2C_WDT_SELECT   |   0
 *
 *                   v_i2c_wdt_u8: The value of I2C enable and WDT select
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_I2C_WDT 297

/* BMI055_GYRO_GET_SPI3
 *                 - This API is used to get the status of spi3
 *                   in the register 0x34 bit 0
 *
 *   ioctl-argument: v_spi3_u8 : The value of spi3 enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_SPI3 298

/* BMI055_GYRO_SET_SPI3
 *                 - This API is used to set the status of spi3
 *                   in the register 0x34 bit 0
 *
 *   ioctl-argument: v_spi3_u8 : The value of spi3 enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SPI3 299

/* BMI055_GYRO_GET_FIFO_TAG
 *                 - This API is used to get the status of FIFO tag
 *                   in the register 0x3D bit 7
 *
 *   ioctl-argument: v_fifo_tag_u8 : The value of fifo tag enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_TAG 300

/* BMI055_GYRO_SET_FIFO_TAG
 *                 - This API is used to set the status of FIFO tag
 *                   in the register 0x3D bit 7
 *
 *   ioctl-argument: v_fifo_tag_u8 : The value of fifo tag enable
 *                   value    |  Description
 *                   -----------|---------------
 *                   1       |  BMI055_ENABLE
 *                   0       |  BMI055_DISABLE
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FIFO_TAG 301

/* BMI055_GYRO_GET_FIFO_WM_LEVEL
 *                 - This API is used to get Water Mark Level
 *                   in the register 0x3D bit from 0 to 6
 *
 *   ioctl-argument: v_fifo_wm_level_u8 : The value of fifo water mark level
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_WM_LEVEL 302

/* BMI055_GYRO_SET_FIFO_WM_LEVEL
 *                 - This API is used to set Water Mark Level
 *                   in the register 0x3D bit from 0 to 6
 *
 *   ioctl-argument: v_fifo_wm_level_u8 : The value of fifo water mark level
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FIFO_WM_LEVEL 303

/* BMI055_GYRO_GET_OFFSET
 *                 - This API is used to get the value of offset
 *                   X, Y and Z in the registers 0x36, 0x37, 0x38, 0x39 and 0x3A
 *                   the offset is a 12bit value
 *
 *                   NOTE: X_AXIS ->
 *
 *                   NOTE: bit 0 and 1 is available in the register 0x3A bit 2 and 3
 *
 *                   NOTE: bit 2 and 3 is available in the register 0x36 bit 6 and 7
 *
 *                   NOTE: bit 4 to 11 is available in the register 0x37 bit 0 to 7
 *
 *                   NOTE: Y_AXIS ->
 *
 *                   NOTE: bit 0 is available in the register 0x3A bit 1
 *
 *                   NOTE: bit 1,2 and 3 is available in the register 0x36 bit 3,4 and 5
 *
 *                   NOTE: bit 4 to 11 is available in the register 0x38 bit 0 to 7
 *
 *                   NOTE: Z_AXIS ->
 *
 *                   NOTE: bit 0 is available in the register 0x3A bit 0
 *
 *                   NOTE: bit 1,2 and 3 is available in the register 0x36 bit 0,1 and 3
 *
 *                   NOTE: bit 4 to 11 is available in the register 0x39 bit 0 to 7
 *
 *   ioctl-argument: v_axis_u8 : The value of offset axis selection
 *                   v_axis_u8       |    value
 *                   ----------------|--------------
 *                   BMI055_X_AXIS   |   1
 *                   BMI055_Y_AXIS   |   0
 *                   BMI055_Z_AXIS   |   0
 *
 *                   v_offset_s16: The value of offset
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_OFFSET 304

/* BMI055_GYRO_SET_OFFSET
 *                 - This API is used to set the value of offset
 *                   X, Y and Z in the registers 0x36, 0x37, 0x38, 0x39 and 0x3A
 *                   the offset is a 12bit value
 *
 *                   NOTE: X_AXIS ->
 *
 *                   NOTE: bit 0 and 1 is available in the register 0x3A bit 2 and 3
 *
 *                   NOTE: bit 2 and 3 is available in the register 0x36 bit 6 and 7
 *
 *                   NOTE: bit 4 to 11 is available in the register 0x37 bit 0 to 7
 *
 *                   NOTE: Y_AXIS ->
 *
 *                   NOTE: bit 0 is available in the register 0x3A bit 1
 *
 *                   NOTE: bit 1,2 and 3 is available in the register 0x36 bit 3,4 and 5
 *
 *                   NOTE: bit 4 to 11 is available in the register 0x38 bit 0 to 7
 *
 *                   NOTE: Z_AXIS ->
 *
 *                   NOTE: bit 0 is available in the register 0x3A bit 0
 *
 *                   NOTE: bit 1,2 and 3 is available in the register 0x36 bit 0,1 and 3
 *
 *                   NOTE: bit 4 to 11 is available in the register 0x39 bit 0 to 7
 *
 *   ioctl-argument: v_axis_u8 : The value of offset axis selection
 *                   v_axis_u8       |    value
 *                   ----------------|--------------
 *                   BMI055_X_AXIS   |   1
 *                   BMI055_Y_AXIS   |   0
 *                   BMI055_Z_AXIS   |   0
 *
 *                   v_offset_s16: The value of offset
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_OFFSET 305

/* BMI055_GYRO_GET_GP
 *                 - This API is used to get the status of general
 *                   purpose register in the register 0x3A and 0x3B
 *
 *   ioctl-argument: v_param_u8: The value of general purpose register select
 *                   v_param_u8      |    value
 *                   ----------------|--------------
 *                   BMI055_GP0      |  0
 *                   BMI055_GP1      |  1
 *
 *                   v_gp_u8: The value of general purpose register
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_GP 306

/* BMI055_GYRO_SET_GP
 *                 - This API is used to set the status of general
 *                   purpose register in the register 0x3A and 0x3B
 *
 *   ioctl-argument: v_param_u8: The value of general purpose register select
 *                   v_param_u8      |    value
 *                   ----------------|--------------
 *                   BMI055_GP0      |  0
 *                   BMI055_GP1      |  1
 *
 *                   v_gp_u8: The value of general purpose register
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_GP 307

/* BMI055_GYRO_GET_FIFO_DATA_REG
 *                 - Reads FIFO data from location 0x3F
 *
 *   ioctl-argument: v_fifo_data_u8 : The data of fifo
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_DATA_REG 308

/* BMI055_GYRO_GET_FIFO_STAT_REG
 *                 - this api is used to read the fifo status
 *                   of frame_counter and overrun in the register 0x0E
 *
 *                   NOTE: frame_counter > bit from 0 to 6
 *
 *                   NOTE: overrun -> bit 7
 *
 *   ioctl-argument: v_fifo_stat_u8 : The value of fifo overrun and fifo counter
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_STAT_REG 309

/* BMI055_GYRO_GET_FIFO_FRAME_COUNT
 *                 - this API is used to get the fifo frame counter
 *                   in the register 0x0E bit 0 to 6
 *
 *   ioctl-argument: v_fifo_frame_count_u8: The value of fifo frame counter
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_FRAME_COUNT 310

/* BMI055_GYRO_GET_FIFO_OVERRUN
 *                 - this API is used to get the fifo over run
 *                   in the register 0x0E bit 7
 *
 *   ioctl-argument: v_fifo_overrun_u8: The value of fifo over run
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_OVERRUN 311

/* BMI055_GYRO_GET_FIFO_MODE
 *                 - This API is used to get the status of fifo mode
 *                   in the register 0x3E bit 6 and 7
 *
 *   ioctl-argument: v_fifo_mode_u8 : The value of fifo mode
 *                   mode      |    value
 *                   ----------------|--------------
 *                   BYPASS      |  0
 *                   FIFO        |  1
 *                   STREAM      |  2
 *                   RESERVED    |  3
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_MODE 312

/* BMI055_GYRO_SET_FIFO_MODE
 *                 - This API is used to set the status of fifo mode
 *                   in the register 0x3E bit 6 and 7
 *
 *   ioctl-argument: v_fifo_mode_u8 : The value of fifo mode
 *                   mode      |    value
 *                   ----------------|--------------
 *                   BYPASS      |  0
 *                   FIFO        |  1
 *                   STREAM      |  2
 *                   RESERVED    |  3
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FIFO_MODE 313

/* BMI055_GYRO_GET_FIFO_DATA_SELECT
 *                 - This API is used to get the status of fifo
 *                   data select in the register 0x3E bit 0 and 1
 *
 *   ioctl-argument: v_fifo_data_select_u8 : The value of fifo data selection
 *                   data selection         |    value
 *                   ---------------------------|--------------
 *                   X,Y and Z (DEFAULT)    |  0
 *                   X only                 |  1
 *                   Y only                 |  2
 *                   Z only                 |  3
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_FIFO_DATA_SELECT 314

/* BMI055_GYRO_SET_FIFO_DATA_SELECT
 *                 - This API is used to set the status of fifo
 *                   data select in the register 0x3E bit 0 and 1
 *
 *   ioctl-argument: v_fifo_data_select_u8 : The value of fifo data selection
 *                   data selection         |    value
 *                   ---------------------------|--------------
 *                   X,Y and Z (DEFAULT)    |  0
 *                   X only                 |  1
 *                   Y only                 |  2
 *                   Z only                 |  3
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_FIFO_DATA_SELECT 315

/* BMI055_GYRO_GET_POWER_MODE
 *                 - This API is used to get the operating modes of the
 *                   sensor in the registers 0x11 and 0x12
 *
 *   ioctl-argument: v_power_mode_u8 :The value of power mode
 *                   value     |   power mode
 *                   -----------|----------------
 *                   0      | BMI055_MODE_NORMAL
 *                   1      | BMI055_MODE_SUSPEND
 *                   2      | BMI055_MODE_DEEPSUSPEND
 *                   3      | BMI055_MODE_FASTPOWERUP
 *                   4      | BMI055_MODE_ADVANCEDPOWERSAVING
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_POWER_MODE 316

/* BMI055_GYRO_SET_POWER_MODE
 *                 - This API is used to set the operating modes of the
 *                   sensor in the registers 0x11 and 0x12
 *
 *   ioctl-argument: v_power_mode_u8 :The value of power mode
 *                   value     |   power mode
 *                   -----------|----------------
 *                   0      | BMI055_MODE_NORMAL
 *                   1      | BMI055_MODE_SUSPEND
 *                   2      | BMI055_MODE_DEEPSUSPEND
 *                   3      | BMI055_MODE_FASTPOWERUP
 *                   4      | BMI055_MODE_ADVANCEDPOWERSAVING
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_POWER_MODE 317

/* BMI055_GYRO_SELFTEST
 *                 - This API is used to to do selftest to sensor
 *                   sensor in the register 0x3C
 *
 *   ioctl-argument: v_result_u8: The value of self test
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SELFTEST 318

/* BMI055_GYRO_GET_AUTO_SLEEP_DURN
 *                 - This API is used to get the auto sleep duration
 *                   in the register 0x12 bit 0 to 2
 *
 *   ioctl-argument: v_durn_u8 : The value of gyro auto sleep duration
 *                   sleep duration     |   value
 *                   ----------------------------|----------
 *                   not allowed    |   0
 *                   4ms            |   1
 *                   5ms            |   2
 *                   8ms            |   3
 *                   10ms           |   4
 *                   15ms           |   5
 *                   20ms           |   6
 *                   40ms           |   7
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_AUTO_SLEEP_DURN 319

/* BMI055_GYRO_SET_AUTO_SLEEP_DURN
 *                 - This API is used to set the auto sleep duration
 *                   in the register 0x12 bit 0 to 2
 *
 *   ioctl-argument: v_durn_u8 : The value of gyro auto sleep duration
 *                   sleep duration     |   value
 *                   ----------------------------|----------
 *                   not allowed    |   0
 *                   4ms            |   1
 *                   5ms            |   2
 *                   8ms            |   3
 *                   10ms           |   4
 *                   15ms           |   5
 *                   20ms           |   6
 *                   40ms           |   7
 *
 *                   v_bw_u8 : The value of selected bandwidth
 *                   v_bw_u8               |   value
 *                   ----------------------------|----------
 *                   C_BMI055_NO_FILTER_U8X      |   0
 *                   C_BMI055_BW_230HZ_U8X       |   1
 *                   C_BMI055_BW_116HZ_u8X       |   2
 *                   C_BMI055_BW_47HZ_u8X        |   3
 *                   C_BMI055_BW_23HZ_u8X        |   4
 *                   C_BMI055_BW_12HZ_u8X        |   5
 *                   C_BMI055_BW_64HZ_u8X        |   6
 *                   C_BMI055_BW_32HZ_u8X        |   7
 *                   @note: sleep duration depends on selected power mode and bandwidth
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_AUTO_SLEEP_DURN 320

/* BMI055_GYRO_GET_SLEEP_DURN
 *                 - This API is used to get the sleep duration
 *                   in the register 0x11 bit 1 to 3
 *
 *   ioctl-argument: v_durn_u8 : The value of sleep duration
 *                   sleep duration     |   value
 *                   ----------------------------|----------
 *                   2ms            |   0
 *                   4ms            |   1
 *                   5ms            |   2
 *                   8ms            |   3
 *                   10ms           |   4
 *                   15ms           |   5
 *                   18ms           |   6
 *                   20ms           |   7
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_GET_SLEEP_DURN 321

/* BMI055_GYRO_SET_SLEEP_DURN
 *                 - This API is used to set the sleep duration
 *                   in the register 0x11 bit 1 to 3
 *
 *   ioctl-argument: v_durn_u8 : The value of sleep duration
 *                   sleep duration     |   value
 *                   ----------------------------|----------
 *                   2ms            |   0
 *                   4ms            |   1
 *                   5ms            |   2
 *                   8ms            |   3
 *                   10ms           |   4
 *                   15ms           |   5
 *                   18ms           |   6
 *                   20ms           |   7
 *
 *   returns: 0: Success; negated value on error
 */
#define BMI055_GYRO_SET_SLEEP_DURN 322

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* A reference to a structure of this type must be passed to the BMI055
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct bmi055_config_s
{

  /* The IRQ number must be provided for each BMI055 device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the BMI055 interrupt handler to the GPIO interrupt of the
   * concrete BMI055 instance.
   */

  int (*attach)(FAR struct bmi055_config_s *, xcpt_t, FAR void* arg);
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bmi055_register
 *
 * Description:
 *   Register the BMI055 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   i2c - An instance of the I2C interface to use to communicate with BMI055
 *   config - contains the irq and isr pointer. Currently not implemented, can be NULL
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmi055_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, FAR struct bmi055_config_s* config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_BMI055 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMI055_H */
