/****************************************************************************
 * drivers/i2c/i2c_lower_half.c
 *
 *   Copyright (C) 2018 Giorgio Groß. All rights reserved.
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
#include <nuttx/i2c/i2cmultiplexer.h>

#ifdef CONFIG_I2CMUX_LOWER_HALF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Tyes
 ****************************************************************************/

/* I2C lower half device struct */
struct i2clh_dev_s
{

  /* Publicly visible lower half state */

  FAR struct i2c_master_s vi2c; /* virtual I2C interface; allows casting as public
                                   i2c master */

  /* Private lower half data */

  FAR struct i2cmultiplexer_dev_s* i2cmux;  /* i2c multiplexer device */
  uint8_t port;  /* mux port to associate the vi2c instance with */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int i2clh_transfer (FAR struct i2c_master_s *dev,
                    FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int i2clh_reset (FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/


static const struct i2c_ops_s g_i2clh_ops =
{
  i2clh_transfer
#ifdef CONFIG_I2C_RESET
  , i2clh_reset
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int i2clh_transfer (FAR struct i2c_master_s *dev,
                    FAR struct i2c_msg_s *msgs, int count)
{

  /* pass the call to the proper i2c multiplexer */

  FAR struct i2clh_dev_s* priv = (struct i2clh_dev_s*) dev;
  return I2CMUX_TRANSFER_ON_PORT(priv->i2cmux, priv->port, msgs, count);
}

#ifdef CONFIG_I2C_RESET
static int i2clh_reset (FAR struct i2c_master_s *dev)
{

  /* pass the call to the proper i2c multiplexer */

  FAR struct i2clh_dev_s* priv = (struct i2clh_dev_s*) dev;
  return I2CMUX_RESET_ON_PORT(priv->i2cmux, priv->port);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2cmux_lower_half
 *
 * Description:
 *   Create and register a virtual I2C character driver.
 *
 *   Transfer and reset calls the the created virtual i2c master instance will
 *   be piped through the i2cmux lower half implementation. Each virtual i2c master
 *   instance is associated with an i2c multiplexer and a port on that multiplexer
 *   and will be registered as /dev/i2cN where N is the passed minor.
 *
 * Input Parameters:
 *   i2cmux - An instance of the i2c mutliplexer
 *   port   - The associated port on the multiplexer
 *   minor  - The I2C bus number.  This will be used as the I2C device minor
 *     number.  The virtual I2C character device will be registered as /dev/i2cN
 *     where N is the minor number
 *
 * Returned Value:
 *   Pointer to the virtual i2c master instance; NULL on failure.
 *
 ****************************************************************************/

int i2cmux_lower_half(FAR struct i2cmultiplexer_dev_s* i2cmux, FAR struct i2c_master_s** vi2c,
                        uint8_t port)
{
  FAR struct i2clh_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2cmux != NULL);

  /* Initialize the PCA9549BDP device structure */

  priv = (FAR struct i2clh_dev_s *)kmm_malloc(sizeof(struct i2clh_dev_s));
  if (priv == NULL)
    {
      i2cerr("ERROR: Failed to allocate i2c mux lower half instance\n");
      return -ENOMEM;
    }

  /* set vi2c ops to their implementations */

  priv->vi2c.ops = &g_i2clh_ops;
  *vi2c = &priv->vi2c;

  /* save state variables */

  priv->i2cmux     = i2cmux;
  priv->port       = port;

  return OK;
}
#endif /* CONFIG_I2CMUX_LOWER_HALF */
