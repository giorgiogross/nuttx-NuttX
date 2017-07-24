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

#include <sys/types.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <errno.h>

#include "spirit_config.h"
#include "spirit_types.h"
#include "spirit_management.h"
#include "spirit_radio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define XTAL_FLAG(xtalFrequency) \
  (xtalFrequency >= 25e6) ? XTAL_FLAG_26_MHz : XTAL_FLAG_24_MHz
#define ROUND(a) \
  (((a - (uint32_t)a) > 0.5) ? (uint32_t)a + 1 : (uint32_t)a)

/* Returns the absolute value. */

#define S_ABS(a) ((a) > 0 ? (a) : -(a))

/******************************************************************************
 * Private Data
 ******************************************************************************/

/* It represents the available channel bandwidth times 10 for 26 Mhz xtal.
 * NOTE: The channel bandwidth for others xtal frequencies can be computed
 * since this table multiplying the current table by a factor
 * xtal_frequency/26e6.
 */

static const uint16_t g_vectn_bandwidth[90] =
{
  8001, 7951, 7684, 7368, 7051, 6709, 6423, 5867, 5414,
  4509, 4259, 4032, 3808, 3621, 3417, 3254, 2945, 2703,
  2247, 2124, 2015, 1900, 1807, 1706, 1624, 1471, 1350,
  1123, 1062, 1005,  950,  903,  853,  812,  735,  675,
   561,  530,  502,  474,  451,  426,  406,  367,  337,
   280,  265,  251,  237,  226,  213,  203,  184,  169,
   140,  133,  126,  119,  113,  106,  101,   92,   84,
    70,   66,   63,   59,   56,   53,   51,   46,   42,
    35,   33,   31,   30,   28,   27,   25,   23,   21,
    18,   17,   16,   15,   14,   13,   13,   12,   11
};

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
  float ifoff;
  int32_t offset;
  int16_t fcoffset;
  uint8_t anaoffset;
  uint8_t digregs[4];
  uint8_t anaregs[8];
  uint8_t drm;
  uint8_t dre;
  uint8_t fdevm;
  uint8_t fdeve;
  uint8_t bwm;
  uint8_t bwe;
  uint8_t regval;
  uint8_t value;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_FREQUENCY_BAND(radioinit->base_frequency));
  DEBUGASSERT(IS_MODULATION_SELECTED(radioinit->modselect));
  DEBUGASSERT(IS_DATARATE(radioinit->datarate));
  DEBUGASSERT(IS_FREQUENCY_OFFSET(offset, spirit->xtal_frequency));
  DEBUGASSERT(IS_CHANNEL_SPACE
                 (radioinit->chspace, spirit->xtal_frequency));
  DEBUGASSERT(IS_F_DEV(radioinit->freqdev, spirit->xtal_frequency));

  /* Workaround for Vtune */

  value = 0xa0;
  ret = spirit_reg_write(spirit, 0x9F, &value, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Calculates the offset respect to RF frequency and according to xtal_ppm
   * parameter: (xtal_ppm*FBase)/10^6
   */

  offset = (int32_t)(((float)radioinit->xtal_offset_ppm *
                     radioinit->base_frequency) / PPM_FACTOR);

  /* Disable the digital, ADC, SMPS reference clock divider if fXO > 24MHz or
   * fXO < 26MHz
   */

  ret = spirit_command(spirit, COMMAND_STANDBY);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      volatile uint8_t i;

      /* Delay for state transition */

      for (i = 0; i != 0xff; i++);

      /* Reads the MC_STATUS register */

      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_STANDBY);

  if (spirit->xtal_frequency < DOUBLE_XTAL_THR)
    {
      ret = spirit_radio_enable_digdivider(spirit, S_DISABLE);
      DEBUGASSERT(IS_CH_BW(radioinit->bandwidth, spirit->xtal_frequency));
    }
  else
    {
      ret = spirit_radio_enable_digdivider(spirit, S_ENABLE);
      DEBUGASSERT(IS_CH_BW(radioinit->bandwidth, (spirit->xtal_frequency >> 1)));
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Go to READY state */

  ret = spirit_command(spirit, COMMAND_READY);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      volatile uint8_t i;

      /* Delay for state transition */

      for (i = 0; i != 0xff; i++);

      /* Reads the MC_STATUS register */

      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
       }
    }
  while (spirit->u.state.MC_STATE != MC_STATE_READY);

  /* Calculates the FC_OFFSET parameter and cast as signed int: offset =
   * (Fxtal/2^18)*FC_OFFSET
   */

  fcoffset = (int16_t)(((float)offset * FBASE_DIVIDER) /
                       spirit->xtal_frequency);
  anaregs[2] = (uint8_t)((((uint16_t)fcoffset) >> 8) & 0x0f);
  anaregs[3] = (uint8_t)fcoffset;

  /* Calculates the channel space factor */

  anaregs[0] = ((uint32_t)radioinit->chspace << 9) /
                (spirit->xtal_frequency >> 6) + 1;

  spirit_management_initcommstate(spirit, radioinit->base_frequency);

  /* 2nd order DEM algorithm enabling */

  ret = spirit_reg_read(spirit, 0xa3, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~0x02;
  ret = spirit_reg_write(spirit, 0xa3, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Check the channel center frequency is in one of the possible range */

  DEBUGASSERT(IS_FREQUENCY_BAND((radioinit->base_frequency +
                                ((fcoffset * spirit->xtal_frequency) / FBASE_DIVIDER) +
                                radioinit->chspace * radioinit->chnum)));

  /* Calculates the datarate mantissa and exponent */

  ret = spirit_radio_convert_datarate(spirit, radioinit->datarate,
                                      &drm, &dre);
  if (ret < 0)
    {
      return ret;
    }

  digregs[0] = (uint8_t)(drm);
  digregs[1] = (uint8_t)(radioinit->modselect | dre);

  /* Read the fdev register to preserve the clock recovery algo bit */

  ret = spirit_reg_read(spirit, 0x1c, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Calculates the frequency deviation mantissa and exponent */

  ret = spirit_radio_convert_freqdev(spirit, radioinit->freqdev,
                                     &fdevm, &fdeve);
  if (ret < 0)
    {
      return ret;
    }

  digregs[2] = (uint8_t)((fdeve << 4) | (regval & 0x08) | fdevm);

  /* Calculates the channel filter mantissa and exponent */

  ret = spirit_radio_convert_chbandwidth(spirit, radioinit->bandwidth, &bwm, &bwe);
  if (ret < 0)
    {
      return ret;
    }

  digregs[3] = (uint8_t) ((bwm << 4) | bwe);

  ifoff      = (3.0 * 480140) / (spirit->xtal_frequency >> 12) - 64;
  anaoffset  = ROUND(ifoff);

  if (spirit->xtal_frequency < DOUBLE_XTAL_THR)
    {
      /* if offset digital is the same in case of single xtal */

      anaregs[1] = anaoffset;
    }
  else
    {
      ifoff = (3.0 * 480140) / (spirit->xtal_frequency >> 13) - 64;

      /* ... otherwise recompute it */

      anaregs[1] = ROUND(ifoff);
    }

#if 0
  if (spirit->xtal_frequency == 24000000)
    {
      anaoffset  = 0xb6;
      anaregs[1] = 0xb6;
    }

  if (spirit->xtal_frequency == 25000000)
    {
      anaoffset  = 0xac;
      anaregs[1] = 0xac;
    }

  if (spirit->xtal_frequency == 26000000)
    {
      anaoffset  = 0xa3;
      anaregs[1] = 0xa3;
    }

  if (spirit->xtal_frequency == 48000000)
    {
      anaoffset  = 0x3b;
      anaregs[1] = 0xb6;
    }

  if (spirit->xtal_frequency == 50000000)
    {
      anaoffset  = 0x36;
      anaregs[1] = 0xac;
    }

  if (spirit->xtal_frequency == 52000000)
    {
      anaoffset  = 0x31;
      anaregs[1] = 0xa3;
    }
#endif

  ret = spirit_reg_write(spirit, IF_OFFSET_ANA_BASE, &anaoffset, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Set Xtal configuration */

  if (spirit->xtal_frequency > DOUBLE_XTAL_THR)
    {
      enum xtal_flag_e xtlflag = XTAL_FLAG((spirit->xtal_frequency / 2));
      ret = spirit_radio_set_xtalflag(spirit, xtlflag);
    }
  else
    {
      enum xtal_flag_e xtlflag = XTAL_FLAG(spirit->xtal_frequency);
      ret = spirit_radio_set_xtalflag(spirit, xtlflag);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Sets the channel number in the corresponding register */

  ret = spirit_reg_write(spirit, CHNUM_BASE, &radioinit->chnum, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Configures the Analog Radio registers */

  ret = spirit_reg_write(spirit, CHSPACE_BASE, anaregs, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Configures the Digital Radio registers */

  ret = spirit_reg_write(spirit, MOD1_BASE, digregs, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the freeze option of the AFC on the SYNC word */

  ret = spirit_radio_afcfreezeonsync(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the IQC correction optimal value */

  anaregs[0] = 0x80;
  anaregs[1] = 0xe3;

  ret = spirit_reg_write(spirit, 0x99, anaregs, 2);
  if (ret < 0)
    {
      return ret;
    }

  return spirit_radio_set_basefrequency(spirit, radioinit->base_frequency);
}

/******************************************************************************
 * Name: spirit_radio_set_xtalflag
 *
 * Description:
 *   Sets the Xtal configuration in the ANA_FUNC_CONF0 register.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   xtalflag one of the possible value of the enum type xtal_flag_e.
 *         XTAL_FLAG_24_MHz:  in case of 24 MHz crystal
 *         XTAL_FLAG_26_MHz:  in case of 26 MHz crystal
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_set_xtalflag(FAR struct spirit_library_s *spirit,
                              enum xtal_flag_e xtalflag)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_XTAL_FLAG(xtalflag));

  /* Reads the ANA_FUNC_CONF_0 register */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (xtalflag == XTAL_FLAG_26_MHz)
        {
          regval |= SELECT_24_26_MHZ_MASK;
        }
      else
        {
          regval &= (~SELECT_24_26_MHZ_MASK);
        }

      /* Sets the 24_26MHz_SELECT field in the ANA_FUNC_CONF_0 register */

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_set_basefrequency
 *
 * Description:
 *   Sets the Synth word and the Band Select register according to desired
 *   base carrier frequency.  In this API the Xtal configuration is read out
 *   from the corresponding register. The user shall fix it before call this
 *   API.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fbase  - The base carrier frequency expressed in Hz as unsigned word.
 *
 * Returned Value:
 *   Error code: 0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_set_basefrequency(FAR struct spirit_library_s *spirit,
                                   uint32_t fbase)
{
#warning Missing logic
  return -ENOSYS;
}

/******************************************************************************
 * Name: spirit_radio_convert_datarate
 *
 * Description:
 *   Returns the mantissa and exponent, whose value used in the datarate
 *   formula will give the datarate value closer to the given datarate.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   datarate - datarate expressed in bps. This parameter ranging between 100 and 500000.
 *   pcm      - pointer to the returned mantissa value.
 *   pce      - pointer to the returned exponent value.
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_convert_datarate(FAR struct spirit_library_s *spirit,
                                  uint32_t datarate, FAR uint8_t *pcm,
                                  FAR uint8_t *pce)
{
  int16_t intermediate[3];
  uint16_t delta;
  uint8_t mantissa;
  uint8_t divider = 0;
  int8_t i = 15;
  uint8_t j;
  volatile bool find = false;

  /* Check the parameters */

  DEBUGASSERT(IS_DATARATE(datarate));

  divider = (uint8_t)spirit_radio_isenabled_digdivider(spirit);

  /* Search in the datarate array the exponent value */

  while (!find && i >= 0)
    {
      if (datarate >= (spirit->xtal_frequency >> (20 - i + divider)))
        {
          find = true;
        }
      else
        {
          i--;
        }
    }

  i < 0 ? i = 0 : i;
  *pce = i;

  /* Calculates the mantissa value according to the datarate formula */

  mantissa = (datarate * ((uint32_t)1 << (23 - i))) /
              (spirit->xtal_frequency >> (5 + divider)) - 256;

  /* Finds the mantissa value with less approximation */

  for (j = 0; j < 3; j++)
    {
      if ((mantissa + j - 1))
        {
          intermediate[j] = datarate - (((256 + mantissa + j - 1) *
                            (spirit->xtal_frequency >> (5 + divider))) >>
                            (23 - i));
        }
      else
        {
          intermediate[j] = 0x7fff;
        }
    }

  delta = 0xffff;
  for (j = 0; j < 3; j++)
    {
      if (S_ABS(intermediate[j]) < delta)
        {
          delta = S_ABS(intermediate[j]);
          *pcm = mantissa + j - 1;
        }
    }

  return OK;
}

/******************************************************************************
 * Name: spirit_radio_convert_freqdev
 *
 * Description:
 *   Returns the mantissa and exponent, whose value used in the frequency
 *   deviation formula will give a frequency deviation value most closer to
 *   the given frequency deviation.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   fdev   - Frequency deviation expressed in Hz. This parameter can be a
 *            value in the range [F_Xo*8/2^18, F_Xo*7680/2^18].
 *   pcm    - pointer to the returned mantissa value.
 *   pce    - pointer to the returned exponent value.
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_convert_freqdev(FAR struct spirit_library_s *spirit,
                                 uint32_t fdev, FAR uint8_t *pcm,
                                 FAR uint8_t *pce)
{
  uint32_t a;
  uint32_t bp;
  uint32_t b = 0;
  uint8_t i;
  float xtalDivtmp = (float)spirit->xtal_frequency / (((uint32_t) 1) << 18);

  /* Check the parameters */

  DEBUGASSERT(IS_F_DEV(fdev, spirit->xtal_frequency));

  for (i = 0; i < 10; i++)
    {
      a = (uint32_t) (xtalDivtmp * (uint32_t) (7.5 * (1 << i)));
      if (fdev < a)
        {
          break;
        }
    }

  *pce = i;

  for (i = 0; i < 8; i++)
    {
      bp = b;
      b = (uint32_t)(xtalDivtmp * (uint32_t)((8.0 + i) / 2 * (1 << (*pce))));
      if (fdev < b)
        {
          break;
        }
    }

  if ((fdev - bp) < (b - fdev))
    {
      i--;
    }

  *pcm = i;
  return OK;
}

/******************************************************************************
 * Name:
 *
 * Description:
 *   Returns the mantissa and exponent for a given bandwidth.  Even if it is
 *   possible to pass as parameter any value in the below mentioned range, the
 *   API will search the closer value according to a fixed table of channel
 *   bandwidth values (@ref s_vectnBandwidth), as defined in the datasheet,
 *   returning the corresponding mantissa and exponent value.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   bandwidth - bandwidth expressed in Hz. This parameter ranging between
 *               1100 and 800100.
 *   pcm       - pointer to the returned mantissa value.
 *   pce       - pointer to the returned exponent value.
 *
 * Returned Value:
 *   Error code:  0=no error, <0=error during calibration of VCO.
 *
 ******************************************************************************/

int spirit_radio_convert_chbandwidth(FAR struct spirit_library_s *spirit,
                                     uint32_t bandwidth, FAR uint8_t *pcm,
                                     FAR uint8_t *pce)
{
  uint32_t chfltfactor;
  int16_t intermediate[3];
  uint16_t delta;
  uint8_t divider = 1;
  int8_t tmp;
  int8_t i;
  int8_t j;

  /* Search in the channel filter bandwidth table the exponent value */

  if (spirit_radio_isenabled_digdivider(spirit) != S_DISABLE)
    {
      divider = 2;
    }
  else
    {
      divider = 1;
    }

  DEBUGASSERT(IS_CH_BW(bandwidth, spirit->xtal_frequency / divider));

  chfltfactor = (spirit->xtal_frequency / divider) / 100;

  for (i = 0;
       i < 90 && (bandwidth < (uint32_t)((g_vectn_bandwidth[i] *
                  chfltfactor) / 2600));
       i++);

  if (i != 0)
    {
      /* Finds the mantissa value with less approximation */

      tmp = i;
      for (j = 0; j < 3; j++)
        {
          if (((tmp + j - 1) >= 0) || ((tmp + j - 1) <= 89))
            {
              intermediate[j] = bandwidth -
                               (uint32_t)((g_vectn_bandwidth[tmp + j - 1] *
                                chfltfactor) / 2600);
            }
          else
            {
              intermediate[j] = 0x7fff;
            }
        }

      delta = 0xFFFF;

      for (j = 0; j < 3; j++)
        {
          if (S_ABS(intermediate[j]) < delta)
            {
              delta = S_ABS(intermediate[j]);
              i = tmp + j - 1;
            }
        }
    }

  *pce = (uint8_t)(i / 9);
  *pcm = (uint8_t)(i % 9);
  return OK;
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
  return -ENOSYS;
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
  return -ENOSYS;
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

/******************************************************************************
 * Name: spirit_radio_enable_digdivider
 *
 * Description:
 *   Enables or Disables the synthesizer reference divider.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for synthesizer reference divider.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_enable_digdivider(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the XO_RCO_TEST_BASE and mask the PD_CLKDIV bit field */

  ret = spirit_reg_read(spirit, XO_RCO_TEST_BASE, &regval, 1);
  if (ret > 0)
    {
      if (newstate == S_ENABLE)
        {
          regval &= 0xf7;
        }
      else
        {
          regval |= 0x08;
        }

      /* Write the new value to the XO_RCO_TEST_BASE register */

      ret = spirit_reg_write(spirit, XO_RCO_TEST_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_radio_isenabled_digdivider
 *
 * Description:
 *   Get the the synthesizer reference divider state.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.  Error conditions are not detected.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_radio_isenabled_digdivider(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  (void)spirit_reg_read(spirit, XO_RCO_TEST_BASE, &regval, 1);

  if (((regval >> 3) & 0x1) != 0)
    {
      return S_DISABLE;
    }
  else
    {
      return S_ENABLE;
    }
}
