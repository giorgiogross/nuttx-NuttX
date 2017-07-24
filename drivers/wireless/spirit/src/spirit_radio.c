/******************************************************************************
 * drivers/wireless/spirit/spirit_radio.c
 * This file provides all the low level API to manage Analog and Digital radio
 * part of SPIRIT.
 *
 *  Copyright(c) 2015 STMicroelectronics
 *  Author: VMA division - AMS
 *  Version 3.2.2 08-July-2015
 *
 *  Adapted for NuttX by:
 *  Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <math.h>
#include <assert.h>
#include <errno.h>

#include "spirit_types.h"
#include "spirit_radio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name: 
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Value:
 *
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_radio_initialize
 *
 * Description:
 *   Initializes the SPIRIT analog and digital radio part according to the
 *   specified parameters in the radioinit.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   radioinit - Pointer to a struct radio_init_s that contains the
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_initialize(FAR struct spirit_library_s *spirit,
                            FAR const struct radio_init_s *radioinit)
{
#warning Missing logic
  return -ENOSYS;
}

/******************************************************************************
 * Name: spirit_radio_set_palevel
 *
 * Description:
 *   Sets a specific PA_LEVEL register, with a value given in dBm.
 *
 *   NOTE: This function makes use of the @ref spirit_radio_dbm2reg fcn to
 *   interpolate the power value.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   ndx      - PA_LEVEL to set. This parameter shall be in the range [0:7].
 *   powerdbm - PA value to write expressed in dBm . Be sure that this values
 *              is in the correct range [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dBm.
 *
 * Returned Value:
 *   None.
 *
 ******************************************************************************/

int spirit_radio_set_palevel(FAR struct spirit_library_s *spirit,
                             uint8_t ndx, float powerdbm)
{
#warning Missing logic
}

/******************************************************************************
 * Name: spirit_radio_set_palevel_maxindex
 *
 * Description:
 *   Sets a specific PA_LEVEL_MAX_INDEX.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   ndx    - PA_LEVEL_MAX_INDEX to set. This parameter must be in the range
 *            [0:7].
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel_maxindex(FAR struct spirit_library_s *spirit,
                                      uint8_t ndx)
{
#warning Missing logic
}

/******************************************************************************
 * Name: spirit_radio_afcfreezeonsync
 *
 * Description:
 *   Enables or Disables the AFC freeze on sync word detection.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - new state for AFC freeze on sync word detection.
 *              This parameter can be: S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_afcfreezeonsync(FAR struct spirit_library_s *spirit,
                                 enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the AFC_2 register and configure the AFC Freeze on Sync field */

  ret = spirit_reg_read(spirit, AFC2_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= AFC2_AFC_FREEZE_ON_SYNC_MASK;
        }
      else
        {
          regval &= (~AFC2_AFC_FREEZE_ON_SYNC_MASK);
        }

      /* Sets the AFC_2 register */

      ret = spirit_reg_write(spirit, AFC2_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_persistentrx
 *
 * Description:
 *   Enables or Disables the persistent RX mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state of this mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_persistentrx(FAR struct spirit_library_s *spirit,
                              enum spirit_functional_state_e newstate)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the PROTOCOL0_BASE and mask the PROTOCOL0_PERS_RX_MASK bitfield */

  ret = spirit_reg_read(spirit, PROTOCOL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= PROTOCOL0_PERS_RX_MASK;
        }
      else
        {
          regval &= (~PROTOCOL0_PERS_RX_MASK);
        }

      /* Writes the new value in the PROTOCOL0_BASE register */

      ret = spirit_reg_write(spirit, PROTOCOL0_BASE, &regval, 1);
    }

  return ret;
}

