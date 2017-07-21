/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_spi.h
 * Header file for low level SPIRIT SPI driver.
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives loosely from similarly licensed SPI interface definitions from
 * STMicro:
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct spi_dev_s; /* Forward reference */

/******************************************************************************
 * Name: spirit_reg_read
 *
 * Description:
 *   Read single or multiple SPIRIT1 register
 *
 * Input parameters:
 *
 *   regaddr: Base register's address to be read
 *   buffer:  Pointer to the buffer of registers' values to be read
 *   buflen:  Number of registers and bytes to be read
 *
 * Returned Value:
 *   Spirit status bytes
 *
 ******************************************************************************/

struct spirit_status_s spirit_reg_read(FAR struct spi_dev_s *spi,
                                       uint8_t regaddr, FAR uint8_t *buffer,
                                       unsigned int buflen);

/******************************************************************************
 * Name: spirit_reg_write
 *
 * Description:
 *   Read single or multiple SPIRIT1 register
 *
 * Input parameters:
 *
 *   regaddr: Base register's address to write
 *   buffer:  Pointer to the buffer of registers' values to write
 *   buflen:  Number of registers and bytes to be read
 *
 * Returned Value:
 *   Spirit status bytes
 *
 ******************************************************************************/

struct spirit_status_s spirit_reg_write(FAR struct spi_dev_s *spi,
                                        uint8_t regaddr,
                                        FAR const uint8_t *buffer,
                                        unsigned int buflen);

/******************************************************************************
 * Name: spirit_command
 *
 * Description:
 *   Send a command
 *
 * Input parameters:
 *
 *   cmd:  Command code to be sent
 *
 * Returned Value:
 *   Spirit status bytes
 *
 ******************************************************************************/

struct spirit_status_s spirit_command(FAR struct spi_dev_s *spi, uint8_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_SPI_H */
