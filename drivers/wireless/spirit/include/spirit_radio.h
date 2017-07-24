/******************************************************************************
 * include/nuttx/wireless/spirit/spirit_radio.h
 * This file provides all the low level API to manage Analog and Digital radio
 * part of SPIRIT.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_RADIO_H
#define __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_RADIO_H

/* In order to configure the Radio main parameters, the user can fit struct
 * radio_init_s structure the and call the spirit_radio_initialize()
 * function passing its pointer* as an argument.
 *
 * Example:
 *
 * struct radio_init_s g_radio_init =
 * {
 *    433.4e6,                 # base frequency
 *    20e3,                    # channel space
 *    0,                       # Xtal offset in ppm
 *    0,                       # channel number
 *    FSK,                     # modulation select
 *    38400,                   # datarate
 *    20e3,                    # frequency deviation
 *    100.5e3                  # channel filter bandwidth
 * };
 *
 * ...
 *
 * spirit_radio_initialize(spirit, &g_radio_init);
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macros used in assertions */

#define IS_XTAL_FLAG(flag)          (((flag) == XTAL_FLAG_24_MHz) || \
                                     ((flag) == XTAL_FLAG_26_MHz))
#define IS_MODULATION_SELECTED(mod) (((mod)  == FSK)              || \
                                     ((mod)  == GFSK_BT05)        || \
                                     ((mod)  == GFSK_BT1)         || \
                                     ((mod)  == ASK_OOK)          || \
                                     ((mod)  == MSK))

/******************************************************************************
 * Public Types
 ******************************************************************************/

/* SPIRIT XTAL frequency enumeration */

enum xtal_flag_e
{
  XTAL_FLAG_24_MHz = 0x00,  /* 24 MHz Xtal selected */
  XTAL_FLAG_26_MHz = 0x01   /* 26 MHz Xtal selected */
};

/* SPIRIT Modulation enumeration */

enum modulation_select_e
{
  FSK              = 0x00,  /* 2-FSK modulation selected */
  GFSK_BT05        = 0x50,  /* GFSK modulation selected with BT=0.5 */
  GFSK_BT1         = 0x10,  /* GFSK modulation selected with BT=1 */
  ASK_OOK          = 0x20,  /* ASK or OOK modulation selected. ASK will
                             * use power ramping */
  MSK              = 0x30   /* MSK modulation selected */
};

/* SPIRIT Radio initialization structure definition */

struct radio_init_s
{
  uint32_t base_frequency;  /* Specifies the base carrier frequency (in
                             * Hz), i.e. the carrier frequency of channel
                             * #0. This parameter can be in one of the
                             * following ranges: High_Band: from 779 MHz to 
                             * 915 MHz Middle Band: from 387 MHz to 470 MHz
                             * Low Band: from 300 MHz to 348 MHz */
  uint32_t chspace;         /* Specifies the channel spacing expressed
                             * in Hz. The channel spacing is expressed as:
                             * NxFREQUENCY_STEPS, where FREQUENCY STEPS is
                             * F_Xo/2^15. This parameter can be in the
                             * range: [0, F_Xo/2^15*255] Hz */
  int16_t xtal_offset_ppm;  /* Specifies the offset frequency (in ppm)
                             * to compensate crystal inaccuracy expressed
                             * as signed value. */
  uint8_t chnum;            /* Specifies the channel number. This value
                             * is multiplied by the channel spacing and
                             * added to synthesizer base frequency to
                             * generate the actual RF carrier frequency */
  uint8_t modselect;        /* Specifies the modulation. This parameter can
                             * be any value from enum modulation_select_e */
  uint32_t datarate;        /* Specifies the datarate expressed in bps.
                             * This parameter can be in the range between
                             * 100 bps and 500 kbps */
  uint32_t freqdev;         /* Specifies the frequency deviation
                             * expressed in Hz. This parameter can be in
                             * the range: [F_Xo*8/2^18, F_Xo*7680/2^18] Hz */
  uint32_t bandwidth;       /* Specifies the channel filter bandwidth
                             * expressed in Hz. This parameter can be in
                             * the range between 1100 and 800100 Hz */
};

/******************************************************************************
 * Public Function Prototypes
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
                            FAR const struct radio_init_s *radioinit);

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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_palevel(FAR struct spirit_library_s *spirit,
                             uint8_t ndx, float powerdbm);

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
                                      uint8_t ndx);

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
                                 enum spirit_functional_state_e newstate);

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
                              enum spirit_functional_state_e newstate);

#endif /* __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_RADIO_H*/
