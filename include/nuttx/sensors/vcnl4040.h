/****************************************************************************
 * include/nuttx/sensors/vcnl4040.h
 *
 *   Copyright (C) 2017 Giorgio Groß. All rights reserved.
 *   Author: Giorgio Groß <giorgio.gross@robodev.eu>
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

#ifndef __INCLUDE_NUTTX_SENSORS_VCNL4040_H
#define __INCLUDE_NUTTX_SENSORS_VCNL4040_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <fixedmath.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_VCNL4040)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I2C Address */
#define VCNL_I2C_ADDR             0x60

/* Device id */
#define VCNL4040_DEV_ID         0x0186

/*  Device commands to issue (write to) or read from */
#define VCNL4040_CMD_ALS_CONF     0x00 /* ALS integration time, persistence, interrupt, and function enable / disable */
#define VCNL4040_CMD_ALS_THDH     0x01 /* ALS high interrupt threshold */
#define VCNL4040_CMD_ALS_THDL     0x02 /* ALS low interrupt threshold */
#define VCNL4040_CMD_ALS_DATA     0x09 /* ALS output data */
#define VCNL4040_CMD_WHITE_DATA   0x0A /* White output data */

#define VCNL4040_CMD_PS_CONF_A    0x03 /* PS duty ratio, integration time, persistence, and PS enable / disable */
#define VCNL4040_CMD_PS_CONF_B    0x04 /* PS smart persistence, active force mode, white channel enable / disable, PS mode selection, PS protection setting, and LED current selection */
#define VCNL4040_CMD_PS_CANC      0x05 /* PS cancellation level setting */
#define VCNL4040_CMD_PS_THDL      0x06 /* PS low interrupt threshold setting */
#define VCNL4040_CMD_PS_THDH      0x07 /* PS high interrupt threshold setting */
#define VCNL4040_CMD_PS_DATA      0x08 /* PS output data */

#define VCNL4040_CMD_INTFLGS      0x0B /* ALS, PS interrupt flags */
#define VCNL4040_CMD_DEV_ID       0x0C /* Device ID */

/* Config params - use this as params along with the corresponding commands defined above */
/* ALS_CONF */
#define VCNL4040_ALS_ITTIME_80        (0x00 << 6) /* ALS integration time 80ms */
#define VCNL4040_ALS_ITTIME_160       (0x01 << 6) /* ALS integration time 160ms */
#define VCNL4040_ALS_ITTIME_320       (0x02 << 6) /* ALS integration time 320ms */
#define VCNL4040_ALS_ITTIME_640       (0x03 << 6) /* ALS integration time 640ms */

#define VCNL4040_ALS_PERS_1           (0x00 << 2) /* ALS interrupt persistance setting */
#define VCNL4040_ALS_PERS_2           (0x01 << 2) /* ALS interrupt persistance setting */
#define VCNL4040_ALS_PERS_4           (0x02 << 2) /* ALS interrupt persistance setting */
#define VCNL4040_ALS_PERS_8           (0x03 << 2) /* ALS interrupt persistance setting */

#define VCNL4040_ALS_INT_EN           (0x01 << 1) /* ALS interrupt eanble */
#define VCNL4040_ALS_SD               (0x01) /* ALS shut down */
#define VCNL4040_ALS_EN               (0x00) /* ALS enable */

/* PS_CONF_A */
#define VCNL4040_PS_HD                (0x01 << 11) /* PS output 16bit (12 bit if not set) */
#define VCNL4040_PS_INT_DIS           (0x00 << 8) /* PS interrupt disable */
#define VCNL4040_PS_INT_CL            (0x01 << 8) /* PS interrupt when close */
#define VCNL4040_PS_INT_AW            (0x02 << 8) /* PS interrupt when away */
#define VCNL4040_PS_INT_CA            (0x03 << 8) /* PS interrupt when close or away */
#define VCNL4040_PS_DUTY_40           (0x00 << 6) /* PS on/off duty ratio 1/40 */
#define VCNL4040_PS_DUTY_80           (0x01 << 6) /* PS on/off duty ratio 1/80 */
#define VCNL4040_PS_DUTY_160          (0x02 << 6) /* PS on/off duty ratio 1/160 */
#define VCNL4040_PS_DUTY_320          (0x03 << 6) /* PS on/off duty ratio 1/320 */
#define VCNL4040_PS_PERS_1            (0x00 << 4) /* PS interrupt persistance setting */
#define VCNL4040_PS_PERS_2            (0x01 << 4) /* PS interrupt persistance setting */
#define VCNL4040_PS_PERS_3            (0x02 << 4) /* PS interrupt persistance setting */
#define VCNL4040_PS_PERS_4            (0x03 << 4) /* PS interrupt persistance setting */
#define VCNL4040_PS_ITTIME_1          (0x00 << 1) /* PS integration time 1T */
#define VCNL4040_PS_ITTIME_1P5        (0x01 << 1) /* PS integration time 1.5T */
#define VCNL4040_PS_ITTIME_2          (0x02 << 1) /* PS integration time 2T */
#define VCNL4040_PS_ITTIME_2P5        (0x03 << 1) /* PS integration time 2.5T */
#define VCNL4040_PS_ITTIME_3          (0x04 << 1) /* PS integration time 3T*/
#define VCNL4040_PS_ITTIME_3P5        (0x05 << 1) /* PS integration time 3.5T */
#define VCNL4040_PS_ITTIME_4          (0x06 << 1) /* PS integration time 4T */
#define VCNL4040_PS_ITTIME_8          (0x07 << 1) /* PS integration time 8T */
#define VCNL4040_PS_SD                (0x01) /* PS shut down */
#define VCNL4040_PS_EN                (0x00) /* PS enable */

/* PS_CONF_B */
#define VCNL4040_PS_WHITE_DIS         (0x01 << 15) /* PS white channel disable */
#define VCNL4040_PS_MS                (0x01 << 14) /* PS proximity detection logic output mode enable */
#define VCNL4040_PS_LED_I_50          (0x00 << 8) /* PS LED current 50mA */
#define VCNL4040_PS_LED_I_75          (0x01 << 8) /* PS LED current 75mA */
#define VCNL4040_PS_LED_I_100         (0x02 << 8) /* PS LED current 100mA */
#define VCNL4040_PS_LED_I_120         (0x03 << 8) /* PS LED current 120mA */
#define VCNL4040_PS_LED_I_140         (0x04 << 8) /* PS LED current 140mA */
#define VCNL4040_PS_LED_I_160         (0x05 << 8) /* PS LED current 160mA */
#define VCNL4040_PS_LED_I_180         (0x06 << 8) /* PS LED current 180mA */
#define VCNL4040_PS_LED_I_200         (0x07 << 8) /* PS LED current 200mA */
#define VCNL4040_PS_SMART_PERS        (0x01 << 4) /* PS smart persistance enable */
#define VCNL4040_PS_AF                (0x01 << 3) /* PS active force mode enable (only sense proximity when
                                                   * '1' is written to PS_TRIG) */
#define VCNL4040_PS_TRIG              (0x01 << 2) /* PS trigger one time cycle; returns to 0 automatically; 
                                                   * PS_AF and PS_TRIG are used for power saving proximity 
                                                   * sensing and curreently not supported */


/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef struct vcnl4040_config
{
    uint16_t als_conf;
    uint16_t als_thdl;
    uint16_t als_thdh;
    uint16_t ps_conf_a;
    uint16_t ps_conf_b;
    uint16_t ps_canc_level;
    uint16_t ps_thdl;
    uint16_t ps_thdh;
} vcnl4040_config_t;

typedef struct vcnl4040_data
{
    b16_t als_data;
    b16_t ps_data;
    uint16_t int_flags;
} vcnl4040_data_t;

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
 * Name: vcnl4040_register
 *
 * Description:
 *   Register the VCNL4040 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/vcnl4040"
 *   i2c - An instance of the I2C interface to use to communicate with VCNL4040
 *   addr - The I2C address of the VCNL4040.
 *   frequency - The I2C frequency
 *   config - A vcnl4040_config_t describing how to set up the VCNL4040
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int vcnl4040_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr, uint32_t frequency, vcnl4040_config_t mode);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_VCNL4040 */
#endif /* __INCLUDE_NUTTX_SENSORS_VCNL4040_H */
