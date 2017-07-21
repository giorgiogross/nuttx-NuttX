/****************************************************************************
 * drivers/wireless/ieee802154/spirit1/spirit1.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <assert.h>
#include <debug.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/spirit1.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "spirit1.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

#ifndef CONFIG_IEEE802154_spirit1_SPIMODE
#  define CONFIG_IEEE802154_spirit1_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_IEEE802154_spirit1_FREQUENCY
#  define CONFIG_IEEE802154_spirit1_FREQUENCY 1000000
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* SPIRIT1 device instance
 *
 * Make sure that struct ieee802154_radio_s remains first.  If not it will break the
 * code
 */

struct spirit1_dev_s
{
  struct ieee802154_radio_s         ieee;      /* Public device instance */
  FAR struct spi_dev_s             *spi;       /* Saved SPI interface instance */
  FAR const struct spirit1_lower_s *lower;     /* Low-level MCU-specific support */
  struct work_s                     irqwork;   /* Interrupt continuation work queue support */
  uint8_t                           panid[2];  /* PAN identifier, FFFF = not set */
  uint16_t                          saddr;     /* Short address, FFFF = not set */
  uint8_t                           eaddr[8];  /* Extended address, FFFFFFFFFFFFFFFF = not set */
  uint8_t                           channel;   /* 11 to 26 for the 2.4 GHz band */
  uint8_t                           devmode;   /* Device mode: device, coord, pancoord */
  uint8_t                           paenabled; /* Enable usage of PA */
  uint8_t                           rxmode;    /* Reception mode: Main, no CRC, promiscuous */
  int32_t                           txpower;   /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s           cca;       /* Clear channel assessement method */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spirit1_irqworker
 *
 * Description:
 *   Actual thread to handle the irq outside of privaleged mode.
 *
 ****************************************************************************/

static void spirit1_irqworker(FAR void *arg)
{
  FAR struct spirit1_dev_s *dev = (FAR struct spirit1_dev_s *)arg;
  uint8_t status = 0;

  wlinfo("Status: 0x%02X\n", status);
  UNUSED(status);

  /* Process the Spirit1 interrupt */

  /* Re-enable the interrupt. */

  dev->lower->enable(dev->lower, true);
}

/****************************************************************************
 * Name: spirit1_interrupt
 *
 * Description:
 *   Actual interrupt handler ran inside privileged space.
 *
 ****************************************************************************/

static int spirit1_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct spirit1_dev_s *dev = (FAR struct spirit1_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&dev->irqwork));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  dev->lower->enable(dev->lower, false);

  return work_queue(HPWORK, &dev->irqwork, spirit1_irqworker,
                    (FAR void *)dev, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spirit1_init
 *
 * Description:
 *   Return an spirit1 device for use by other drivers.
 *
 ****************************************************************************/

FAR struct ieee802154_radio_s *spirit1_init(FAR struct spi_dev_s *spi,
                                            FAR const struct spirit1_lower_s *lower)
{
  FAR struct spirit1_dev_s *dev;

  /* Allocate a driver state structure instance */

  dev = (FAR struct spirit1_dev_s *)kmm_zalloc(sizeof(struct spirit1_dev_s));
  if (dev == NULL)
   {
      wlerr("ERROR: Failed to allocate device structure\n");
   }

  /* Attach the interface, lower driver, and devops */

  dev->spi   = spi;
  dev->lower = lower;
  //dev->ieee.ops = &spirit1_devops;

  /* Attach irq */

  if (lower->attach(lower, spirit1_interrupt, dev) != OK)
    {
      return NULL;
    }

  //sem_init(&dev->ieee.rxsem, 0, 0);
  //sem_init(&dev->ieee.txsem, 0, 0);

  /* Initialize device */

  //spirit1_initialize(dev);

  /* Put the Device to RX ON Mode */

  /* Enable Radio IRQ */

  lower->enable(lower, true);
  return &dev->ieee;
}
