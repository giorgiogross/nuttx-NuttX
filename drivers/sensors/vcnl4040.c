/****************************************************************************
 * drivers/sensors/vcnl4040.c
 * Character driver for the Vishay Ambient Light and Proximity Sensor
 * VCNL4040
 *
 *   Copyright (C) 2017 - 2018 Giorgio Groß. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <fixedmath.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/vcnl4040.h>

#if defined(CONFIG_I2C) && defined(CONFIG_VCNL4040)

/****************************************************************************
 * Pre-process Definitions
 ****************************************************************************/
#define I2C_NOSTARTSTOP_MSGS 2
#define I2C_NOSTARTSTOP_COMMAND_MSG_INDEX 0
#define I2C_NOSTARTSTOP_DATA_MSG_INDEX 1

#define VCNL4040_ALS_IT_UNIT_80   0.1
#define VCNL4040_ALS_IT_UNIT_160  0.05
#define VCNL4040_ALS_IT_UNIT_320  0.025
#define VCNL4040_ALS_IT_UNIT_640  0.0125

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vcnl4040_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint32_t frequency;           /* I2C frequency */
  vcnl4040_config_t config;     /* device configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     vcnl4040_read16(FAR struct vcnl4040_dev_s *priv, uint8_t command_code, FAR uint16_t *regvalue);
static int     vcnl4040_write16(FAR struct vcnl4040_dev_s *priv, uint8_t command_code, FAR uint16_t *regvalue);

/* Character driver methods */

static int     vcnl4040_open(FAR struct file *filep);
static int     vcnl4040_close(FAR struct file *filep);
static ssize_t vcnl4040_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t vcnl4040_write(FAR struct file *filep,
                 FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_vcnl4040_fops =
{
  vcnl4040_open,   /* open */
  vcnl4040_close,  /* close */
  vcnl4040_read,   /* read */
  vcnl4040_write,  /* write */
  NULL,            /* seek */
  NULL             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


static int vcnl4040_read_reg(FAR struct vcnl4040_dev_s *priv, uint8_t command_code, uint8_t* register_value, uint8_t data_length) {
  struct i2c_config_s iconf;
  iconf.frequency = priv->frequency;
  iconf.address = priv->addr;
  iconf.addrlen = 7;

  int ret = i2c_writeread(priv->i2c, &iconf, &command_code, 1,
                    register_value, data_length);
  sninfo("command_code: 0x%02X data_length: %d register_value: 0x%02x (0x%04x) ret: %d\n", command_code, data_length, *register_value, *((uint16_t*)register_value), ret);
  return ret;
}

static int vcnl4040_write_reg(FAR struct vcnl4040_dev_s *priv, uint8_t command_code, uint8_t* buffer, uint8_t data_length) {
  // struct i2c_msg_s msg;

  unsigned char buf[data_length + 1];
  buf[0] = command_code;
  int c_new;
  for (c_new = 0; c_new < data_length; c_new++)
    {
      buf[c_new + 1] = buffer[c_new];
    }

  struct i2c_config_s iconf;
  iconf.frequency = priv->frequency;
  iconf.address = priv->addr;
  iconf.addrlen = 7;

  int ret = i2c_write(priv->i2c, &iconf, buf, 1 + data_length);
  sninfo("command_code: 0x%02X, data_length: %d, buffer: 0x%02x (0x%04x), ret: %d\n", command_code, data_length, *buffer, *((uint16_t*)(&buf[1])), ret);
  return ret;
}

/****************************************************************************
 * Name: vcnl4040_read16
 *
 * Description:
 *   Read 16-bit register
 *
 ****************************************************************************/
static int vcnl4040_read16(FAR struct vcnl4040_dev_s *priv, uint8_t command_code, FAR uint16_t *regvalue)
{
  return vcnl4040_read_reg(priv, command_code, (uint8_t*)regvalue, sizeof(*regvalue));
}

/****************************************************************************
 * Name: vcnl4040_write16
 *
 * Description:
 *   Write to a 16-bit register
 *
 ****************************************************************************/
static int vcnl4040_write16(FAR struct vcnl4040_dev_s *priv, uint8_t command_code, FAR uint16_t *regvalue)
{
  return vcnl4040_write_reg(priv, command_code, (uint8_t*)regvalue, sizeof(*regvalue));
}

/****************************************************************************
 * Name: vcnl4040_open
 *
 * Description:
 *   This function is called whenever the VCNL4040 device is opened.
 *
 ****************************************************************************/

static int vcnl4040_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vcnl4040_close
 *
 * Description:
 *   This routine is called when the VCNL4040 device is closed.
 *
 ****************************************************************************/

static int vcnl4040_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vcnl4040_read
 ****************************************************************************/

static ssize_t vcnl4040_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  int ret = OK;
  FAR struct inode         *inode;
  FAR struct vcnl4040_dev_s *priv;
  uint16_t raw;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct vcnl4040_dev_s *)inode->i_private;

  /* Check if the user is reading the right size */

  if(buflen != sizeof(vcnl4040_data_t))
    {
      snerr("ERROR: Provided buffer invalid, should provide %d bytes\n", sizeof(vcnl4040_data_t));
      return -EINVAL;
    }

  /* Read 16 bit als value */

  if(!(priv->config.als_conf & VCNL4040_ALS_SD))
    {
      ret = vcnl4040_read16(priv, VCNL4040_CMD_ALS_DATA, &raw);
      if (ret != OK)
        {
          snerr("ERROR: Reading als failed!\n");
          return -ECOMM;  /* nothing read */
        }

      uint16_t als_ittime = priv->config.als_conf & VCNL4040_ALS_ITTIME_640;
      float lv = -1.0;
      switch (als_ittime)
        {
          case VCNL4040_ALS_ITTIME_80:
            lv = (float) raw * VCNL4040_ALS_IT_UNIT_80;
            break;
          case VCNL4040_ALS_ITTIME_160:
            lv = (float) raw * VCNL4040_ALS_IT_UNIT_160;
            break;
          case VCNL4040_ALS_ITTIME_320:
            lv = (float) raw * VCNL4040_ALS_IT_UNIT_320;
            break;
          case VCNL4040_ALS_ITTIME_640:
            lv = (float) raw * VCNL4040_ALS_IT_UNIT_640;
            break;
        }

      ((vcnl4040_data_t*)buffer)->als_data = ftob16(lv);
    } else {
      ((vcnl4040_data_t*)buffer)->als_data = ftob16(-1.0);
    }

  /* Read 16 bit ps value */

  if(!(priv->config.ps_conf_a & VCNL4040_ALS_SD))
    {
      ret = vcnl4040_read16(priv, VCNL4040_CMD_PS_DATA, &raw);
      if (ret != OK)
        {
          snerr("ERROR: Reading ps failed!\n");
          return -ECOMM;  /* nothing read */
        }

      ((vcnl4040_data_t*)buffer)->ps_data = ftob16((float) raw);
    } else {
      ((vcnl4040_data_t*)buffer)->ps_data = ftob16(-1.0);
    }

  /* Reset interrupts */

  ret = vcnl4040_read16(priv, VCNL4040_CMD_INTFLGS, &raw);
  if (ret != OK)
    {
      snerr("ERROR: Reading interrupt flags (and resetting them) failed!\n");
      return -ECOMM;  /* nothing read */
    }

  ((vcnl4040_data_t*)buffer)->int_flags = raw;

  return buflen;  /* read buflen bytes in total */
}

/****************************************************************************
 * Name: vcnl4040_write
 ****************************************************************************/

static ssize_t vcnl4040_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  /* ps active force mode or resetting the device configuration can be implemented
     here if needed */
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int vcnl4040_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr, uint32_t frequency, vcnl4040_config_t config)
{
  int ret;
  uint16_t id = 0;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Check format of config struct */

  if((config.als_conf & 0xff30) | (config.ps_conf_a & 0xf000) | (config.ps_conf_b & 0x38e1))
  {
    snerr("ERROR: Configuration invalid. Reserved bits are used.");
    return -EINVAL;
  }

  /* Initialize the VCNL4040 device structure */

  FAR struct vcnl4040_dev_s *priv = (FAR struct vcnl4040_dev_s *)kmm_malloc(sizeof(struct vcnl4040_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate VCNL instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->frequency = frequency;
  priv->config = config;

  /* write default configuration first as in some cases first access to sensor
     registers fail */

  ret = vcnl4040_write16(priv, VCNL4040_CMD_ALS_CONF, &config.als_conf);
  if (ret != OK)
   {
     snerr("ERROR: Failed to write dummy config!\n");
     kmm_free(priv);
     return ret;
   }

  /* check device id */

  ret = vcnl4040_read16(priv, VCNL4040_CMD_DEV_ID, &id);
  if (ret != OK)
    {
      snerr("ERROR: Failed to read device id from VCNL4040\n");
      kmm_free(priv);
      return ret;
    }
  DEBUGASSERT(id == VCNL4040_DEV_ID);

  if(!(config.als_conf & VCNL4040_ALS_SD))
    {
      /* Configure ALS */

      ret = vcnl4040_write16(priv, VCNL4040_CMD_ALS_CONF, &config.als_conf);
      if (ret != OK)
        {
          snerr("ERROR: Failed to initialize ALS for VCNL4040!\n");
          kmm_free(priv);
          return ret;
        }

      /* Write threshold values if interupts are enabled */

      if(config.als_conf & VCNL4040_ALS_INT_EN)
        {
          ret = vcnl4040_write16(priv, VCNL4040_CMD_ALS_THDH, &config.als_thdh);
          if (ret != OK)
            {
              snerr("ERROR: Failed to set ALS high threshold for VCNL4040!\n");
              kmm_free(priv);
              return ret;
            }

          ret = vcnl4040_write16(priv, VCNL4040_CMD_ALS_THDL, &config.als_thdl);
          if (ret != OK)
            {
              snerr("ERROR: Failed to set ALS low threshold for VCNL4040!\n");
              kmm_free(priv);
              return ret;
            }
        }
    }

  if(!(config.ps_conf_a & VCNL4040_PS_SD))
    {

      /* Configure PS */

      ret = vcnl4040_write16(priv, VCNL4040_CMD_PS_CONF_A, &config.ps_conf_a);
      if (ret != OK)
        {
          snerr("ERROR: Failed to initialize PS CONF A for VCNL4040!\n");
          kmm_free(priv);
          return ret;
        }

      ret = vcnl4040_write16(priv, VCNL4040_CMD_PS_CONF_B, &config.ps_conf_b);
      if (ret != OK)
        {
          snerr("ERROR: Failed to initialize PS CONF B for VCNL4040!\n");
          kmm_free(priv);
          return ret;
        }

      /* Write cancellation level setting */

      ret = vcnl4040_write16(priv, VCNL4040_CMD_PS_CANC, &config.ps_canc_level);
        if (ret != OK)
          {
            snerr("ERROR: Failed to set cancellation level setting for VCNL4040!\n");
            kmm_free(priv);
            return ret;
          }

      /* Write threshold values if interupts are enabled */

      if(config.ps_conf_a & VCNL4040_PS_INT_CA)
        {
          ret = vcnl4040_write16(priv, VCNL4040_CMD_PS_THDH, &config.ps_thdh);
          if (ret != OK)
            {
              snerr("ERROR: Failed to set PS high threshold for VCNL4040!\n");
              kmm_free(priv);
              return ret;
            }

          ret = vcnl4040_write16(priv, VCNL4040_CMD_PS_THDL, &config.ps_thdl);
          if (ret != OK)
            {
              snerr("ERROR: Failed to set PS low threshold for VCNL4040!\n");
              kmm_free(priv);
              return ret;
            }
        }
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_vcnl4040_fops, 0666, priv);
  if (ret != OK)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_VCNL4040 */
