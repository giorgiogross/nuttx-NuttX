/****************************************************************************
 * drivers/i2c/pca9540bdp.c
 * Driver for the PCA9540BDP i2c multiplexer
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/pca9540bdp.h>

#if defined(CONFIG_I2C) && defined(CONFIG_PCA9540BDP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_PCA9549BDP_I2C_FREQUENCY
#  define CONFIG_PCA9549BDP_I2C_FREQUENCY   400000  /* 400 khz */
#endif

#define PCA9540BDP_CH_BITMASK               0x03
#define PCA9540BDP_ENABLE_BITMASK           0x04

#define PCA9540BDP_CH_BIT                   0
#define PCA9540BDP_CH_NONE_BIT              1
#define PCA9540BDP_ENABLE_BIT               2

/****************************************************************************
 * Private
 ****************************************************************************/

struct pca9540bdp_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint16_t addr;
  uint8_t state;  /* control register state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static int     pca9540bdp_write_config(FAR struct pca9540bdp_dev_s *priv,
                             FAR uint8_t regvalue);
static int     pca9540bdp_read_config(FAR struct pca9540bdp_dev_s *priv,
                             FAR uint8_t *regvalue);

/* Character driver methods */

static int     pca9540bdp_open(FAR struct file *filep);
static int     pca9540bdp_close(FAR struct file *filep);
static ssize_t pca9540bdp_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t pca9540bdp_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     pca9540bdp_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pca9540bdpfops =
{
  pca9540bdp_open,
  pca9540bdp_close,
  pca9540bdp_read,
  pca9540bdp_write,
  NULL,
  pca9540bdp_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pca9540bdp_write_config(FAR struct pca9540bdp_dev_s *priv,
                             FAR uint8_t regvalue)
{
  struct i2c_config_s iconf;
  iconf.frequency = CONFIG_PCA9549BDP_I2C_FREQUENCY;
  iconf.address = priv->addr;
  iconf.addrlen = 7;
  int ret = i2c_write(priv->i2c, &iconf, &regvalue, 1);
  i2cinfo("address: 0x%02X register_value: 0x%02x  ret: %d\n", priv->addr, regvalue, ret);
  return ret;
}

static int pca9540bdp_read_config(FAR struct pca9540bdp_dev_s *priv,
                             FAR uint8_t *regvalue)
{
  struct i2c_config_s iconf;
  iconf.frequency = CONFIG_PCA9549BDP_I2C_FREQUENCY;
  iconf.address = priv->addr;
  iconf.addrlen = 7;
  int ret = i2c_read(priv->i2c, &iconf, regvalue, 1);

  i2cinfo("address: 0x%02X register_value: 0x%02x  ret: %d\n", priv->addr, *regvalue, ret);
  return ret;
}

/****************************************************************************
 * Name: pca9540bdp_open
 *
 * Description:
 *   This function is called whenever the PCA9549BDP device is opened.
 *
 ****************************************************************************/

static int pca9540bdp_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pca9540bdp_close
 *
 * Description:
 *   This routine is called when the PCA9549BDP device is closed.
 *
 ****************************************************************************/

static int pca9540bdp_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pca9540bdp_read
 ****************************************************************************/

static ssize_t pca9540bdp_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pca9540bdp_dev_s *priv  = inode->i_private;
  int                    ret   = OK;

  DEBUGASSERT(priv != NULL);

  if(buflen >= 1 && pca9540bdp_read_config(priv, (uint8_t *)buffer) == OK)
  {
    ret = 1;
  } else {
    ret = -EINVAL;
  }
  return ret;
}

/****************************************************************************
 * Name: pca9540bdp_write
 ****************************************************************************/

static ssize_t pca9540bdp_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: pca9540bdp_ioctl
 ****************************************************************************/

static int pca9540bdp_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pca9540bdp_dev_s *priv  = inode->i_private;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
      /* Write enable bit */

      case PCA9540BDPIOC_DISABLE:
        {
          priv->state = PCA9540BDP_DISABLE;
          break;
        }

      /* Write channel selection bit */

      case PCA9540BDPIOC_SEL_CH:
        {
          uint8_t val = (uint8_t) arg;

          if(val == PCA9540BDP_SEL_CH0)
          {
            priv->state = PCA9540BDP_ENABLE | PCA9540BDP_SEL_CH0;
          }
          else if(val == PCA9540BDP_SEL_CH1)
          {
            priv->state = PCA9540BDP_ENABLE | PCA9540BDP_SEL_CH1;
          }
          else
          {
            return -EINVAL;
          }
          break;
        }

      default:
        i2cinfo("Unrecognized cmd: %d\n", cmd);
        return -ENOTTY;
    }
  return pca9540bdp_write_config(priv, priv->state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pca9540bdp_register
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pca9540bdp_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                        uint8_t addr)
{
  FAR struct pca9540bdp_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the PCA9549BDP device structure */

  priv = (FAR struct pca9540bdp_dev_s *)kmm_malloc(sizeof(struct pca9540bdp_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->state      = 0x00;  // power on reset configuration

  /* Register the character driver */

  ret = register_driver(devpath, &g_pca9540bdpfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  i2cinfo("PCA9549BDP (addr=0x%02x) registered at %s\n", priv->addr, devpath);

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_PCA9549BDP */
