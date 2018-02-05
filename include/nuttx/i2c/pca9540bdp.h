/****************************************************************************
 * include/nuttx/i2c/pca9540bdp.h
 *
 *   Copyright (C) 2018 Giorgio Groß. All rights reserved.
 *   Author: Giorgio Groß <giorgio.gross@robodev.eu>
 *
 * References:
 *   "PCA9540B 2-channel I2C-bus multiplexer product datasheet",
 *   31 October 2016, NXP
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

#ifndef __INCLUDE_I2C_PCA9540BDP_H
#define __INCLUDE_I2C_PCA9540BDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef CONFIG_PCA9540BDP_BASEADDR
#  define CONFIG_PCA9540BDP_BASEADDR          0x70
#endif

/* Bit Definitions to be passed to ioctl calls and i2c_lower_half */

#define PCA9540BDP_SEL_PORT0        0x0
#define PCA9540BDP_SEL_PORT1        0x1
#define PCA9540BDP_ENABLE           0x4
#define PCA9540BDP_DISABLE          0x0

/* IOCTL Commands ***********************************************************/
/* PCA9540BDPIOC_DISABLE
 *                    - disable all ports
 *   ioctl argument:  none
 *
 * PCA9540BDPIOC_SEL_PORT
 *                    - Select port
 *   ioctl argument:  One of PCA9540BDP_SEL_CH0 or PCA9540BDP_SEL_CH1
 *
 */

#define PCA9540BDPIOC_DISABLE     0x01
#define PCA9540BDPIOC_SEL_PORT    0x02

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: pca9540bdp_initialize
 *
 * Description:
 *   Register the PCA9540BDP character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/pca0"
 *   i2c - An instance of the I2C interface to use to communicate with PCA9540BDP
 *   addr - The I2C address of the PCA9540BDP.  The base I2C address of the PCA9540BDP
 *   is 0x70.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2cmultiplexer_dev_s* pca9540bdp_initialize(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_I2C_PCA9540BDP_H */
