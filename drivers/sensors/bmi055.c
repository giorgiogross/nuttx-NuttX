/****************************************************************************
 * drivers/sensors/bmi055.c
 * Character driver for the BOSCH BMI055 Acceleration and Gyroscope Sensor
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <stdlib.h>
#include <string.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi055.h>
#include <nuttx/sensors/bma2x2_lib.h>
#include <nuttx/sensors/bmg160_lib.h>
// #include "bmi055.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMI055)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BMI055_I2C_FREQUENCY
#  define CONFIG_BMI055_I2C_FREQUENCY 400000  /* 400KHz */
#endif


/****************************************************************************
 * Private
 ****************************************************************************/

struct bmi055_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */

  struct bma2x2_t accel_dev;
  struct bmg160_t gyro_dev;

  FAR struct bmi055_config_s *config; /* Pointer to the configuration of the
                                       * BMI055 sensor */
  /* struct work_s work; */           /* The work queue is responsible for
                                       * retrieving the data from the sensor
                                       * after the arrival of new data was
                                       * signalled in an interrupt */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static s8     bmi055_read_reg(FAR void* p, u8 device_addr, u8 register_address, u8* register_value, u8 data_length);
static s8     bmi055_write_reg(FAR void* p, u8 device_addr, u8 register_address, u8* register_value, u8 data_length);

/* bosch arch dependent functions */
static void delay_msec(uint32_t delay);

/* Character driver methods */

static int     bmi055_open(FAR struct file *filep);
static int     bmi055_close(FAR struct file *filep);
static ssize_t bmi055_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t bmi055_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     bmi055_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/* ISR */
static int bmi055_interrupt_handler(int irq, FAR void* arg, void* context);
// static void bmi055_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmi055fops =
{
  bmi055_open,
  bmi055_close,
  bmi055_read,
  bmi055_write,
  NULL,
  bmi055_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static s8 bmi055_read_reg(FAR void* p, u8 device_addr, u8 register_address, u8* register_value, u8 data_length)
{
  struct bmi055_dev_s *priv = (struct bmi055_dev_s *) p;

  struct i2c_config_s iconf;
  iconf.frequency = CONFIG_BMI055_I2C_FREQUENCY;
  iconf.address = device_addr;
  iconf.addrlen = 7;

  int ret = i2c_writeread(priv->i2c, &iconf, &register_address, 1,
                    register_value, data_length);
  sninfo("device_addr: 0x%02X register_address: 0x%02X data_length: %d register_value: 0x%02x ret: %d\n",
    device_addr, register_address, data_length, *register_value, ret);
  return ret;
}

static s8 bmi055_write_reg(FAR void* p, u8 device_addr, u8 register_address, u8* register_value, u8 data_length)
{
  struct bmi055_dev_s *priv = (struct bmi055_dev_s *) p;

  unsigned char buf[data_length + 1];
  buf[0] = register_address;
  int c_new;
  for (c_new = 0; c_new < data_length; c_new++)
    {
      buf[c_new + 1] = register_value[c_new];
    }

  struct i2c_config_s iconf;
  iconf.frequency = CONFIG_BMI055_I2C_FREQUENCY;
  iconf.address = device_addr;
  iconf.addrlen = 7;

  int ret = i2c_write(priv->i2c, &iconf, buf, 1 + data_length);
  sninfo("register_address: 0x%02X, data_length: %d, buffer: 0x%02x (0x%04x), ret: %d\n", register_address, data_length, *buf, *((uint16_t*)(&buf[1])), ret);
  return ret;
}

static void delay_msec(uint32_t delay)
{
  usleep(delay);
}

/****************************************************************************
 * Name: bmi055_open
 *
 * Description:
 *   This function is called whenever the BMI055 device is opened.
 *
 ****************************************************************************/

static int bmi055_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: bmi055_close
 *
 * Description:
 *   This routine is called when the BMI055 device is closed.
 *
 ****************************************************************************/

static int bmi055_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: bmi055_read
 ****************************************************************************/

static ssize_t bmi055_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  size_t nsamples;
  int ret, i;

  /* How many samples will fit in the buffer? Buffer will have to handle
     bma2x2_accel_data type from bma2x2_bosch.h util */

  nsamples = buflen / sizeof(struct bma2x2_accel_data);

  /* If the provided buffer is not large enough to return a single sample,
   * then return an error.
   */

  if (nsamples < 1)
    {
      snerr("ERROR: Bufer too small %lu < %u\n",
            buflen, sizeof(struct bma2x2_accel_data));
      return (ssize_t)-EINVAL;
    }

  /* Return all of the samples that will fit in the user-provided buffer */

  for (i = 0; i < nsamples; i++)
    {
      /* Get the next sample data */

      ret = bma2x2_read_accel_xyz((FAR struct bma2x2_accel_data *)buffer);
      if (ret < 0)
        {
          snerr("ERROR: bmi055_read_sensor_data failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Set up for the next sample */

      buffer += sizeof(struct bma2x2_accel_data);
    }

  return (ssize_t)(nsamples * sizeof(struct bma2x2_accel_data));
}

/****************************************************************************
 * Name: bmi055_write
 ****************************************************************************/

static ssize_t bmi055_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bmi055_interrupt_handler
 ****************************************************************************/

static int bmi055_interrupt_handler(int irq, FAR void* arg, void* context)
{
  /* This function should be called upon a rising edge on a bmi055
   * interrupt pin since it signals that new data has been measured.
   *
   * NOTE: not implemented yet
   */

  /* Task the worker with retrieving the latest sensor data. We should not do
   * this in a interrupt since it might take too long. Also we cannot lock the
   * SPI bus from within an interrupt.
   */

  // FAR struct bmi055_dev_s *priv = (struct bmi055_dev_s*) arg;
  // int ret;

  // DEBUGASSERT(priv->work.worker == NULL);
  // ret = work_queue(HPWORK, &priv->work, bmi055_worker, priv, 0);
  // if (ret < 0)
  //   {
  //     snerr("ERROR: Failed to queue work: %d\n", ret);
  //     return ret;
  //   }

  return OK;
}

/****************************************************************************
 * Name: bmi055_worker  // TODO interrupts are not implemented yet
 ****************************************************************************/

// static void bmi055_worker(FAR void *arg)
// {
//   FAR struct bmi055_dev_s *priv = (FAR struct bmi055_dev_s *)(arg);
//   DEBUGASSERT(priv != NULL);*/

  /* Read out the latest sensor data */

// }

/****************************************************************************
 * Name: bmi055_ioctl
 ****************************************************************************/

static int bmi055_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = 0;

  switch(cmd)
  {
    /**************************************************************************
     * Acceleration functions following:
     **************************************************************************/
    case BMI055_ACCEL_READ_ACCEL_X:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_x((s16 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_x failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_X:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_eight_resolution_x((s8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_eight_resolution_x failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_Y:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_y((s16 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_y failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_Y:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_eight_resolution_y((s8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_eight_resolution_y failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_Z:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_z((s16 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_z failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_Z:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_eight_resolution_z((s8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_eight_resolution_z failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_XYZ:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_xyz((struct bma2x2_accel_data *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_xyz failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_EIGHT_RESOLUTION_XYZ:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_eight_resolution_xyz((struct bma2x2_accel_eight_resolution *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_eight_resolution_xyz failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_TAP_STAT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_tap_stat((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_tap_stat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_ORIENT_STAT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_orient_stat((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_orient_stat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FIFO_STAT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_fifo_stat((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_fifo_stat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FIFO_FRAME_COUNT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_fifo_frame_count((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_fifo_frame_count failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FIFO_OVERRUN:
      {
        void* params = (void*) arg;
        if(bma2x2_get_fifo_overrun((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_fifo_overrun failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_STAT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_stat((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_stat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_RANGE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_range((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_range failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_RANGE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_range(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_range failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_BW:
      {
        void* params = (void*) arg;
        if(bma2x2_get_bw((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_BW:
      {
        void* params = (void*) arg;
        if(bma2x2_set_bw(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_POWER_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_power_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_power_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_POWER_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_power_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_power_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_MODE_VALUE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_mode_value(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_mode_value failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SLEEP_DURN:
      {
        void* params = (void*) arg;
        if(bma2x2_get_sleep_durn((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_sleep_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SLEEP_DURN:
      {
        void* params = (void*) arg;
        if(bma2x2_set_sleep_durn(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_sleep_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SLEEP_TIMER_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_sleep_timer_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_sleep_timer_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SLEEP_TIMER_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_sleep_timer_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_sleep_timer_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_HIGH_BW:
      {
        void* params = (void*) arg;
        if(bma2x2_get_high_bw((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_high_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_HIGH_BW:
      {
        void* params = (void*) arg;
        if(bma2x2_set_high_bw(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_high_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SHADOW_DIS:
      {
        void* params = (void*) arg;
        if(bma2x2_get_shadow_dis((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_shadow_dis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SHADOW_DIS:
      {
        void* params = (void*) arg;
        if(bma2x2_set_shadow_dis(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_shadow_dis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SOFT_RST:
      {
        if(bma2x2_soft_rst() < 0)
          {
            snerr("ERROR: bma2x2_soft_rst failed\n");
            ret = -ENOTTY;
          }
      }
    case BMI055_ACCEL_UPDATE_IMAGE:
      {
        if(bma2x2_update_image() < 0)
          {
            snerr("ERROR: bma2x2_update_image failed\n");
            ret = -ENOTTY;
          }
      }
    case BMI055_ACCEL_GET_INTR_ENABLE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_enable(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_ENABLE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_enable(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_FIFO_FULL:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_fifo_full((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_fifo_full failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_FIFO_FULL:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_fifo_full(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_fifo_full failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_FIFO_WM:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_fifo_wm((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_fifo_wm failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_FIFO_WM:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_fifo_wm(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_fifo_wm failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SLOW_NO_MOTION:
      {
        void* params = (void*) arg;
        if(bma2x2_get_slow_no_motion(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_slow_no_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SLOW_NO_MOTION:
      {
        void* params = (void*) arg;
        if(bma2x2_set_slow_no_motion(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_slow_no_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_LOW_G:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_low_g(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_low_g failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_LOW_G:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_low_g(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_low_g failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_HIGH_G:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_high_g(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_high_g failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_HIGH_G:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_high_g(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_high_g failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_SLOPE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_slope(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_slope failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_SLOPE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_slope(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_slope failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_SLOW_NO_MOTION:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_slow_no_motion(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_slow_no_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_SLOW_NO_MOTION:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_slow_no_motion(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_slow_no_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_DOUBLE_TAP:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_double_tap(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_double_tap failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_DOUBLE_TAP:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_double_tap(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_double_tap failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_SINGLE_TAP:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_single_tap(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_single_tap failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_SINGLE_TAP:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_single_tap(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_single_tap failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_ORIENT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_orient(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_orient failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_ORIENT:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_orient(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_orient failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_FLAT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_flat(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_flat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_FLAT:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_flat(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_flat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_NEW_DATA:
      {
        void* params = (void*) arg;
        if(bma2x2_get_new_data(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_new_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_NEW_DATA:
      {
        void* params = (void*) arg;
        if(bma2x2_set_new_data(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_new_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR1_FIFO_WM:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr1_fifo_wm((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr1_fifo_wm failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR1_FIFO_WM:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr1_fifo_wm(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_intr1_fifo_wm failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR2_FIFO_WM:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr2_fifo_wm((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr2_fifo_wm failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR2_FIFO_WM:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr2_fifo_wm(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_intr2_fifo_wm failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR1_FIFO_FULL:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr1_fifo_full((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr1_fifo_full failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR1_FIFO_FULL:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr1_fifo_full(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_intr1_fifo_full failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR2_FIFO_FULL:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr2_fifo_full((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_intr2_fifo_full failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR2_FIFO_FULL:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr2_fifo_full(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_intr2_fifo_full failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SOURCE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_source(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_source failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SOURCE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_source(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_source failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_OUTPUT_TYPE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_output_type(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_output_type failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_OUTPUT_TYPE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_output_type(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_output_type failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_INTR_LEVEL:
      {
        void* params = (void*) arg;
        if(bma2x2_get_intr_level(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_intr_level failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_INTR_LEVEL:
      {
        void* params = (void*) arg;
        if(bma2x2_set_intr_level(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_intr_level failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_RST_INTR:
      {
        void* params = (void*) arg;
        if(bma2x2_rst_intr(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_rst_intr failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_LATCH_INTR:
      {
        void* params = (void*) arg;
        if(bma2x2_get_latch_intr((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_latch_intr failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_LATCH_INTR:
      {
        void* params = (void*) arg;
        if(bma2x2_set_latch_intr(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_latch_intr failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_DURN:
      {
        void* params = (void*) arg;
        if(bma2x2_get_durn(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_DURN:
      {
        void* params = (void*) arg;
        if(bma2x2_set_durn(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_THRES:
      {
        void* params = (void*) arg;
        if(bma2x2_get_thres(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_THRES:
      {
        void* params = (void*) arg;
        if(bma2x2_set_thres(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_LOW_HIGH_G_HYST:
      {
        void* params = (void*) arg;
        if(bma2x2_get_low_high_g_hyst(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_low_high_g_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_LOW_HIGH_G_HYST:
      {
        void* params = (void*) arg;
        if(bma2x2_set_low_high_g_hyst(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_low_high_g_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_LOW_G_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_low_g_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_low_g_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_LOW_G_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_low_g_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_low_g_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_TAP_DURN:
      {
        void* params = (void*) arg;
        if(bma2x2_get_tap_durn((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_tap_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_TAP_DURN:
      {
        void* params = (void*) arg;
        if(bma2x2_set_tap_durn(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_tap_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_TAP_SHOCK:
      {
        void* params = (void*) arg;
        if(bma2x2_get_tap_shock((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_tap_shock failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_TAP_SHOCK:
      {
        void* params = (void*) arg;
        if(bma2x2_set_tap_shock(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_tap_shock failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_TAP_QUIET:
      {
        void* params = (void*) arg;
        if(bma2x2_get_tap_quiet((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_tap_quiet failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_TAP_QUIET:
      {
        void* params = (void*) arg;
        if(bma2x2_set_tap_quiet(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_tap_quiet failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_TAP_THRES:
      {
        void* params = (void*) arg;
        if(bma2x2_get_tap_thres((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_tap_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_TAP_THRES:
      {
        void* params = (void*) arg;
        if(bma2x2_set_tap_thres(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_tap_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_TAP_SAMPLE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_tap_sample((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_tap_sample failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_TAP_SAMPLE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_tap_sample(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_tap_sample failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_ORIENT_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_orient_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_orient_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_ORIENT_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_orient_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_orient_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_ORIENT_BLOCK:
      {
        void* params = (void*) arg;
        if(bma2x2_get_orient_block((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_orient_block failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_ORIENT_BLOCK:
      {
        void* params = (void*) arg;
        if(bma2x2_set_orient_block(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_orient_block failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_ORIENT_HYST:
      {
        void* params = (void*) arg;
        if(bma2x2_get_orient_hyst((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_orient_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_ORIENT_HYST:
      {
        void* params = (void*) arg;
        if(bma2x2_set_orient_hyst(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_orient_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_THETA:
      {
        void* params = (void*) arg;
        if(bma2x2_get_theta(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_theta failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_THETA:
      {
        void* params = (void*) arg;
        if(bma2x2_set_theta(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_theta failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_ORIENT_ENABLE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_orient_enable((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_orient_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_ORIENT_ENABLE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_orient_enable(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_orient_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FLAT_HYST:
      {
        void* params = (void*) arg;
        if(bma2x2_get_flat_hyst((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_flat_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_FLAT_HYST:
      {
        void* params = (void*) arg;
        if(bma2x2_set_flat_hyst(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_flat_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FLAT_HOLD_TIME:
      {
        void* params = (void*) arg;
        if(bma2x2_get_flat_hold_time((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_flat_hold_time failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_FLAT_HOLD_TIME:
      {
        void* params = (void*) arg;
        if(bma2x2_set_flat_hold_time(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_flat_hold_time failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FIFO_WML_TRIG:
      {
        void* params = (void*) arg;
        if(bma2x2_get_fifo_wml_trig((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_fifo_wml_trig failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_FIFO_WML_TRIG:
      {
        void* params = (void*) arg;
        if(bma2x2_set_fifo_wml_trig(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_fifo_wml_trig failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SELFTEST_AXIS:
      {
        void* params = (void*) arg;
        if(bma2x2_get_selftest_axis((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_selftest_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SELFTEST_AXIS:
      {
        void* params = (void*) arg;
        if(bma2x2_set_selftest_axis(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_selftest_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SELFTEST_SIGN:
      {
        void* params = (void*) arg;
        if(bma2x2_get_selftest_sign((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_selftest_sign failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SELFTEST_SIGN:
      {
        void* params = (void*) arg;
        if(bma2x2_set_selftest_sign(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_selftest_sign failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_NVMPROG_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_nvmprog_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_nvmprog_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_NVMPROG_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_nvmprog_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_nvmprog_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_NVPROG_TRIG:
      {
        void* params = (void*) arg;
        if(bma2x2_set_nvprog_trig(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_nvprog_trig failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_NVMPROG_READY:
      {
        void* params = (void*) arg;
        if(bma2x2_get_nvmprog_ready((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_nvmprog_ready failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_NVMPROG_REMAIN:
      {
        void* params = (void*) arg;
        if(bma2x2_get_nvmprog_remain((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_nvmprog_remain failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SPI3:
      {
        void* params = (void*) arg;
        if(bma2x2_get_spi3((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_spi3 failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SPI3:
      {
        void* params = (void*) arg;
        if(bma2x2_set_spi3(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_spi3 failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_I2C_WDT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_i2c_wdt(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_i2c_wdt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_I2C_WDT:
      {
        void* params = (void*) arg;
        if(bma2x2_set_i2c_wdt(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_i2c_wdt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_SLOW_COMP:
      {
        void* params = (void*) arg;
        if(bma2x2_get_slow_comp(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_slow_comp failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_SLOW_COMP:
      {
        void* params = (void*) arg;
        if(bma2x2_set_slow_comp(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_slow_comp failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_CAL_RDY:
      {
        void* params = (void*) arg;
        if(bma2x2_get_cal_rdy((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_cal_rdy failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_CAL_TRIGGER:
      {
        void* params = (void*) arg;
        if(bma2x2_set_cal_trigger(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_cal_trigger failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_OFFSET_RST:
      {
        void* params = (void*) arg;
        if(bma2x2_set_offset_rst(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_offset_rst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_OFFSET_TARGET:
      {
        void* params = (void*) arg;
        if(bma2x2_get_offset_target(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_offset_target failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_OFFSET_TARGET:
      {
        void* params = (void*) arg;
        if(bma2x2_set_offset_target(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_offset_target failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_OFFSET:
      {
        void* params = (void*) arg;
        if(bma2x2_get_offset(* (u8 *) (params), (s8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_get_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_OFFSET:
      {
        void* params = (void*) arg;
        if(bma2x2_set_offset(* (u8 *) (params), * (s8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bma2x2_set_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FIFO_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_get_fifo_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_fifo_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_FIFO_MODE:
      {
        void* params = (void*) arg;
        if(bma2x2_set_fifo_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_fifo_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_GET_FIFO_DATA_SELECT:
      {
        void* params = (void*) arg;
        if(bma2x2_get_fifo_data_select((u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_get_fifo_data_select failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_SET_FIFO_DATA_SELECT:
      {
        void* params = (void*) arg;
        if(bma2x2_set_fifo_data_select(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_set_fifo_data_select failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_FIFO_DATA:
      {
        void* params = (void*) arg;
        if(bma2x2_read_fifo_data((struct fifo_configuration *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_fifo_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_EXTRACT_ACCEL:
      {
        void* params = (void*) arg;
        if(bma2x2_extract_accel((union fifo_frame *) (params), (u8 *) (params + sizeof(union fifo_frame *)), (struct fifo_configuration *) (params + sizeof(union fifo_frame *) + sizeof(u8 *))) < 0)
          {
            snerr("ERROR: bma2x2_extract_accel failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_TEMP:
      {
        void* params = (void*) arg;
        if(bma2x2_read_temp((s8 *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_temp failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_ACCEL_READ_ACCEL_XYZT:
      {
        void* params = (void*) arg;
        if(bma2x2_read_accel_xyzt((struct bma2x2_accel_data_temp *) (params)) < 0)
          {
            snerr("ERROR: bma2x2_read_accel_xyzt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    /**************************************************************************
     * Gyroscope functions following:
     **************************************************************************/
    case BMI055_GYRO_GET_DATA_X:
      {
        void* params = (void*) arg;
        if(bmg160_get_data_X((s16 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_data_X failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_DATA_Y:
      {
        void* params = (void*) arg;
        if(bmg160_get_data_Y((s16 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_data_Y failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_DATA_Z:
      {
        void* params = (void*) arg;
        if(bmg160_get_data_Z((s16 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_data_Z failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_DATA_XYZ:
      {
        void* params = (void*) arg;
        if(bmg160_get_data_XYZ((struct bmg160_data_t *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_data_XYZ failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_DATA_XYZI:
      {
        void* params = (void*) arg;
        if(bmg160_get_data_XYZI((struct bmg160_data_t *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_data_XYZI failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_TEMP:
      {
        void* params = (void*) arg;
        if(bmg160_get_temp((s8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_temp failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_READ_REGISTER:
      {
        void* params = (void*) arg;
        if(bmg160_read_register(* (u8 *) (params), (u8 *) (params + sizeof(u8)), * (u8 *) (params + sizeof(u8) + sizeof(u8 *))) < 0)
          {
            snerr("ERROR: bmg160_read_register failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_BURST_READ:
      {
        void* params = (void*) arg;
        if(bmg160_burst_read(* (u8 *) (params), (u8 *) (params + sizeof(u8)), * (u32 *) (params + sizeof(u8) + sizeof(u8 *))) < 0)
          {
            snerr("ERROR: bmg160_burst_read failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_WRITE_REGISTER:
      {
        void* params = (void*) arg;
        if(bmg160_write_register(* (u8 *) (params), (u8 *) (params + sizeof(u8)), * (u8 *) (params + sizeof(u8) + sizeof(u8 *))) < 0)
          {
            snerr("ERROR: bmg160_write_register failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_STAT_REG_ZERO:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_stat_reg_zero((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr_stat_reg_zero failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_STAT_REG_ONE:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_stat_reg_one((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr_stat_reg_one failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_STAT_REG_TWO:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_stat_reg_two((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr_stat_reg_two failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_STAT_REG_THREE:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_stat_reg_three((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr_stat_reg_three failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_RANGE_REG:
      {
        void* params = (void*) arg;
        if(bmg160_get_range_reg((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_range_reg failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_RANGE_REG:
      {
        void* params = (void*) arg;
        if(bmg160_set_range_reg(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_range_reg failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_BW:
      {
        void* params = (void*) arg;
        if(bmg160_get_bw((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_BW:
      {
        void* params = (void*) arg;
        if(bmg160_set_bw(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_PMU_EXT_TRI_SELECT:
      {
        void* params = (void*) arg;
        if(bmg160_get_pmu_ext_tri_select((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_pmu_ext_tri_select failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_PMU_EXT_TRI_SELECT:
      {
        void* params = (void*) arg;
        if(bmg160_set_pmu_ext_tri_select(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_pmu_ext_tri_select failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_HIGH_BW:
      {
        void* params = (void*) arg;
        if(bmg160_get_high_bw((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_high_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_HIGH_BW:
      {
        void* params = (void*) arg;
        if(bmg160_set_high_bw(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_high_bw failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_SHADOW_DIS:
      {
        void* params = (void*) arg;
        if(bmg160_get_shadow_dis((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_shadow_dis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SHADOW_DIS:
      {
        void* params = (void*) arg;
        if(bmg160_set_shadow_dis(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_shadow_dis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SOFT_RST:
      {
        if(bmg160_set_soft_rst() < 0)
          {
            snerr("ERROR: bmg160_set_soft_rst failed\n");
            ret = -ENOTTY;
          }
      }
    case BMI055_GYRO_GET_DATA_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_get_data_enable((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_data_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_DATA_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_set_data_enable(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_data_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_enable((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FIFO_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_set_fifo_enable(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_fifo_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_AUTO_OFFSET_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_get_auto_offset_enable((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_auto_offset_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_AUTO_OFFSET_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_set_auto_offset_enable(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_auto_offset_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_OUTPUT_TYPE:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_output_type(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_intr_output_type failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR_OUTPUT_TYPE:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr_output_type(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_intr_output_type failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_LEVEL:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_level(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_intr_level failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR_LEVEL:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr_level(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_intr_level failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR1_HIGHRATE:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr1_highrate((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr1_highrate failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR1_HIGHRATE:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr1_highrate(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_intr1_highrate failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR1_ANY_MOTION:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr1_any_motion((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr1_any_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR1_ANY_MOTION:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr1_any_motion(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_intr1_any_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_DATA:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_data(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_intr_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR_DATA:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr_data(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_intr_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR2_OFFSET:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr2_offset(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_intr2_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR2_OFFSET:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr2_offset(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_intr2_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR_OFFSET:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr_offset(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_intr_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR1_OFFSET:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr1_offset(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_intr1_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR2_FIFO:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr2_fifo((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr2_fifo failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR1_FIFO:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr1_fifo((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr1_fifo failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR_FIFO:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr_fifo(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_intr_fifo failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR2_HIGHRATE:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr2_highrate((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr2_highrate failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR2_HIGHRATE:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr2_highrate(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_intr2_highrate failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_INTR2_ANY_MOTION:
      {
        void* params = (void*) arg;
        if(bmg160_get_intr2_any_motion((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_intr2_any_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_INTR2_ANY_MOTION:
      {
        void* params = (void*) arg;
        if(bmg160_set_intr2_any_motion(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_intr2_any_motion failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_OFFSET_UNFILT:
      {
        void* params = (void*) arg;
        if(bmg160_get_offset_unfilt(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_offset_unfilt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_OFFSET_UNFILT:
      {
        void* params = (void*) arg;
        if(bmg160_set_offset_unfilt(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_offset_unfilt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_UNFILT_DATA:
      {
        void* params = (void*) arg;
        if(bmg160_get_unfilt_data(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_unfilt_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_UNFILT_DATA:
      {
        void* params = (void*) arg;
        if(bmg160_set_unfilt_data(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_unfilt_data failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_ANY_MOTION_THRES:
      {
        void* params = (void*) arg;
        if(bmg160_get_any_motion_thres((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_any_motion_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_ANY_MOTION_THRES:
      {
        void* params = (void*) arg;
        if(bmg160_set_any_motion_thres(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_any_motion_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_AWAKE_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_get_awake_durn((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_awake_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_AWAKE_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_set_awake_durn(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_awake_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_ANY_MOTION_DURN_SAMPLE:
      {
        void* params = (void*) arg;
        if(bmg160_get_any_motion_durn_sample((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_any_motion_durn_sample failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_ANY_MOTION_DURN_SAMPLE:
      {
        void* params = (void*) arg;
        if(bmg160_set_any_motion_durn_sample(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_any_motion_durn_sample failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_ANY_MOTION_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_get_any_motion_enable_axis(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_any_motion_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_ANY_MOTION_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_set_any_motion_enable_axis(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_any_motion_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_WM_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_wm_enable((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_wm_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FIFO_WM_ENABLE:
      {
        void* params = (void*) arg;
        if(bmg160_set_fifo_wm_enable(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_fifo_wm_enable failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_RST_INTR:
      {
        void* params = (void*) arg;
        if(bmg160_set_rst_intr(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_rst_intr failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_OFFSET_RST:
      {
        void* params = (void*) arg;
        if(bmg160_set_offset_rst(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_offset_rst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_LATCH_STAT:
      {
        void* params = (void*) arg;
        if(bmg160_get_latch_stat((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_latch_stat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_LATCH_STAT:
      {
        void* params = (void*) arg;
        if(bmg160_set_latch_stat(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_latch_stat failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_LATCH_INTR:
      {
        void* params = (void*) arg;
        if(bmg160_get_latch_intr((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_latch_intr failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_LATCH_INTR:
      {
        void* params = (void*) arg;
        if(bmg160_set_latch_intr(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_latch_intr failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_HIGHRATE_HYST:
      {
        void* params = (void*) arg;
        if(bmg160_get_highrate_hyst(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_highrate_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_HIGHRATE_HYST:
      {
        void* params = (void*) arg;
        if(bmg160_set_highrate_hyst(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_highrate_hyst failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_HIGHRATE_THRES:
      {
        void* params = (void*) arg;
        if(bmg160_get_highrate_thres(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_highrate_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_HIGHRATE_THRES:
      {
        void* params = (void*) arg;
        if(bmg160_set_highrate_thres(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_highrate_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_HIGHRATE_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_get_highrate_enable_axis(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_highrate_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_HIGHRATE_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_set_highrate_enable_axis(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_highrate_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_HIGHRATE_DURN_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_get_highrate_durn_axis(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_highrate_durn_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_HIGHRATE_DURN_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_set_highrate_durn_axis(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_highrate_durn_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_SLOW_OFFSET_THRES:
      {
        void* params = (void*) arg;
        if(bmg160_get_slow_offset_thres((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_slow_offset_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SLOW_OFFSET_THRES:
      {
        void* params = (void*) arg;
        if(bmg160_set_slow_offset_thres(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_slow_offset_thres failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_SLOW_OFFSET_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_get_slow_offset_durn((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_slow_offset_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SLOW_OFFSET_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_set_slow_offset_durn(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_slow_offset_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_SLOW_OFFSET_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_get_slow_offset_enable_axis(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_slow_offset_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SLOW_OFFSET_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_set_slow_offset_enable_axis(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_slow_offset_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_OFFSET_WORD_LENGTH:
      {
        void* params = (void*) arg;
        if(bmg160_get_offset_word_length(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_offset_word_length failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_OFFSET_WORD_LENGTH:
      {
        void* params = (void*) arg;
        if(bmg160_set_offset_word_length(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_offset_word_length failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_ENABLE_FAST_OFFSET:
      {
        if(bmg160_enable_fast_offset() < 0)
          {
            snerr("ERROR: bmg160_enable_fast_offset failed\n");
            ret = -ENOTTY;
          }
      }
    case BMI055_GYRO_GET_FAST_OFFSET_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_get_fast_offset_enable_axis((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fast_offset_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FAST_OFFSET_ENABLE_AXIS:
      {
        void* params = (void*) arg;
        if(bmg160_set_fast_offset_enable_axis(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_fast_offset_enable_axis failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_NVM_REMAIN:
      {
        void* params = (void*) arg;
        if(bmg160_get_nvm_remain((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_nvm_remain failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_NVM_LOAD:
      {
        void* params = (void*) arg;
        if(bmg160_set_nvm_load(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_nvm_load failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_NVM_RDY:
      {
        void* params = (void*) arg;
        if(bmg160_get_nvm_rdy((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_nvm_rdy failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_NVM_PROG_TRIG:
      {
        void* params = (void*) arg;
        if(bmg160_set_nvm_prog_trig(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_nvm_prog_trig failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_NVM_PROG_MODE:
      {
        void* params = (void*) arg;
        if(bmg160_get_nvm_prog_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_nvm_prog_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_NVM_PROG_MODE:
      {
        void* params = (void*) arg;
        if(bmg160_set_nvm_prog_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_nvm_prog_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_I2C_WDT:
      {
        void* params = (void*) arg;
        if(bmg160_get_i2c_wdt(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_i2c_wdt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_I2C_WDT:
      {
        void* params = (void*) arg;
        if(bmg160_set_i2c_wdt(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_i2c_wdt failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_SPI3:
      {
        void* params = (void*) arg;
        if(bmg160_get_spi3((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_spi3 failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SPI3:
      {
        void* params = (void*) arg;
        if(bmg160_set_spi3(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_spi3 failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_TAG:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_tag((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_tag failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FIFO_TAG:
      {
        void* params = (void*) arg;
        if(bmg160_set_fifo_tag(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_fifo_tag failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_WM_LEVEL:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_wm_level((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_wm_level failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FIFO_WM_LEVEL:
      {
        void* params = (void*) arg;
        if(bmg160_set_fifo_wm_level(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_fifo_wm_level failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_OFFSET:
      {
        void* params = (void*) arg;
        if(bmg160_get_offset(* (u8 *) (params), (s16 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_OFFSET:
      {
        void* params = (void*) arg;
        if(bmg160_set_offset(* (u8 *) (params), * (s16 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_offset failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_GP:
      {
        void* params = (void*) arg;
        if(bmg160_get_gp(* (u8 *) (params), (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_get_gp failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_GP:
      {
        void* params = (void*) arg;
        if(bmg160_set_gp(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_gp failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_DATA_REG:
      {
        void* params = (void*) arg;
        if(bmg160_get_FIFO_data_reg((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_FIFO_data_reg failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_STAT_REG:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_stat_reg((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_stat_reg failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_FRAME_COUNT:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_frame_count((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_frame_count failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_OVERRUN:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_overrun((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_overrun failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_MODE:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FIFO_MODE:
      {
        void* params = (void*) arg;
        if(bmg160_set_fifo_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_fifo_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_FIFO_DATA_SELECT:
      {
        void* params = (void*) arg;
        if(bmg160_get_fifo_data_select((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_fifo_data_select failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_FIFO_DATA_SELECT:
      {
        void* params = (void*) arg;
        if(bmg160_set_fifo_data_select(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_fifo_data_select failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_POWER_MODE:
      {
        void* params = (void*) arg;
        if(bmg160_get_power_mode((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_power_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_POWER_MODE:
      {
        void* params = (void*) arg;
        if(bmg160_set_power_mode(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_power_mode failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SELFTEST:
      {
        void* params = (void*) arg;
        if(bmg160_selftest((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_selftest failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_AUTO_SLEEP_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_get_auto_sleep_durn((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_auto_sleep_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_AUTO_SLEEP_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_set_auto_sleep_durn(* (u8 *) (params), * (u8 *) (params + sizeof(u8))) < 0)
          {
            snerr("ERROR: bmg160_set_auto_sleep_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_GET_SLEEP_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_get_sleep_durn((u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_get_sleep_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }
    case BMI055_GYRO_SET_SLEEP_DURN:
      {
        void* params = (void*) arg;
        if(bmg160_set_sleep_durn(* (u8 *) (params)) < 0)
          {
            snerr("ERROR: bmg160_set_sleep_durn failed\n");
            ret = -ENOTTY;
          }
        break;
      }

    default:
      {
        ret = -ENOTTY;
      }
  }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi055_register
 *
 * Description:
 *   Register the BMI055 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   i2c - An instance of the I2C interface to use to communicate with BMI055
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmi055_register(FAR const char *devpath, FAR struct i2c_master_s *i2c, FAR struct bmi055_config_s* config)
{
  FAR struct bmi055_dev_s *priv;
  int ret;

  /* Initialize the BMI055 device structure */

  priv = (FAR struct bmi055_dev_s *)kmm_malloc(sizeof(struct bmi055_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->config      = config;
  // priv->work.worker = NULL;

  if(priv->config) {
    ret = priv->config->attach(priv->config, &bmi055_interrupt_handler, priv);
    if (ret < 0)
      {
        snerr("ERROR: Failed to attach interrupt\n");
        return ret;
      }
  }

  /* initialize acceleration functionality */

  priv->accel_dev.priv = priv;
  priv->accel_dev.resolution = BMA2x2_12_RESOLUTION;
  priv->accel_dev.dev_addr = BMI055_ACCEL_I2C_ADDR;
  priv->accel_dev.bus_write = &bmi055_write_reg;
  priv->accel_dev.bus_read = &bmi055_read_reg;
  priv->accel_dev.burst_read = NULL;  /* burst read not supported yet */
  priv->accel_dev.delay_msec = &delay_msec;

  ret = bma2x2_init(&priv->accel_dev);
  if(ret < 0)
    {
      snerr("ERROR: Failed to initialize acceleration device through bma2x2 util: %d\n", ret);
      kmm_free(priv);
    }

  priv->gyro_dev.priv = priv;
  priv->gyro_dev.dev_addr = BMI055_GYRO_I2C_ADDR;
  priv->gyro_dev.bus_write = &bmi055_write_reg;
  priv->gyro_dev.bus_read = &bmi055_read_reg;
  priv->gyro_dev.burst_read = NULL;  /* burst read not supported yet */
  priv->gyro_dev.delay_msec = &delay_msec;

  ret = bmg160_init(&priv->gyro_dev);
  if(ret < 0)
    {
      snerr("ERROR: Failed to initialize gyroscope device through bmg160 util: %d\n", ret);
      kmm_free(priv);
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_bmi055fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMI055 (Accel Chip ID: 0x%02x) (Gyro Chip ID: 0x%02x) registered at %s\n",
    priv->accel_dev.chip_id, priv->gyro_dev.chip_id, devpath);

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_BMI055 */
