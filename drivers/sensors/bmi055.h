/*******************************************************************************
 * drivers/sensors/bmi055.h
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
 ******************************************************************************/

#ifndef __INCLUDE_DRIVERS_SENSORS_BMI055_H
#define __INCLUDE_DRIVERS_SENSORS_BMI055_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/sensors/bmi055.h>

#if defined(CONFIG_SENSORS_BMI055)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI055_I2C_ADDR 0x1D
#define BMI055_DEVICE_ID 0xFFA0  /* BGW_CHIPID rgister */

/********************************************************************************************
 * Private Types
 ********************************************************************************************/

/* This structure represents the state of the BMI055 driver */

struct bmi055_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t open_count;
};

#endif /* CONFIG_SENSORS_BMI055 */
#endif /* __INCLUDE_DRIVERS_SENSORS_BMI055_H */
