/****************************************************************************
 * include/nuttx/i2c/i2cmultiplexer.h
 *
 *   Copyright(C) 2018 Giorgio Gross. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_I2C_I2CMULTIPLEXER_H
#define __INCLUDE_NUTTX_I2C_I2CMULTIPLEXER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdint.h>

#ifdef CONFIG_I2CMULTIPLEXER
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Access macros ************************************************************/

/****************************************************************************
 * Name: I2CMUX_TRANSFER_ON_PORT
 *
 * Description:
 *   Select the passed port on the i2c multiplexer and perform a sequence of I2C
 *   transfers, each transfer is started with a START and the final transfer is
 *   completed with a STOP. Each sequence will be an 'atomic'  operation in the
 *   sense that any other I2C actions will be serialized and pend until this
 *   sequence of transfers completes.
 *
 * Input Parameters:
 *   dev   - I2C multiplexer device-specific state data
 *   port  - Port number on the multiplexer
 *   msgs  - A pointer to a set of message descriptors
 *   count - The number of transfers to perform
 *
 * Returned Value:
 *   The number of transfers completed; a negated errno value on failure.
 *
 ****************************************************************************/

#define I2CMUX_TRANSFER_ON_PORT(d,p,m,c) ((d)->ops->transfer_on_port(d,p,m,c))

/*****************************************************************************
 * Name: I2CMUX_RESET_ON_PORT
 *
 * Description:
 *   Select the passed port on the i2c multiplexer and perform an I2C bus reset
 *   in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - I2C multiplexer device-specific state data
 *   port  - Port number on the mulitplexer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
#  define I2CMUX_RESET_ON_PORT(d,p) ((d)->ops->reset_on_port(d,p))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* I2C multiplexer interface methods */

struct i2cmultiplexer_dev_s;
struct i2cmultiplexer_ops_s
{
  CODE int (*transfer_on_port)(FAR struct i2cmultiplexer_dev_s* dev, uint8_t port,
                       FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
  CODE int (*reset_on_port)(FAR struct i2cmultiplexer_dev_s* dev, uint8_t port);
#endif
};

struct i2cmultiplexer_dev_s
{

  /* Lower half operations provided by the i2c multiplexer lower half */

  FAR const struct i2cmultiplexer_ops_s *ops;
};
#endif  /* CONFIG_I2CMULTIPLEXER */
#endif  /* __INCLUDE_NUTTX_I2C_I2CMULTIPLEXER_H */
