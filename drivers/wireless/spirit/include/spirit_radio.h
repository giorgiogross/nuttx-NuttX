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

/* Radio_Band */

#define FBASE_DIVIDER               262144  /* 2^18 factor dividing fxo in fbase
                                             * formula */

#define HIGH_BAND_FACTOR            6       /* Band select factor for high band.
                                             * Factor B in the equation 2 */
#define MIDDLE_BAND_FACTOR          12      /* Band select factor for middle
                                             * band. Factor B in the equation 2 */
#define LOW_BAND_FACTOR             16      /* Band select factor for low band.
                                             * Factor B in the equation 2 */
#define VERY_LOW_BAND_FACTOR        32      /* Band select factor for very low
                                             * band. Factor B in the equation 2 */

#define HIGH_BAND_LOWER_LIMIT       778000000  /* Lower limit of the high
                                                * band: 779 MHz */
#define HIGH_BAND_UPPER_LIMIT       957100000  /* Upper limit of the high
                                                * band: 956 MHz */
#define MIDDLE_BAND_LOWER_LIMIT     386000000  /* Lower limit of the middle
                                                * band: 387 MHz */
#define MIDDLE_BAND_UPPER_LIMIT     471100000  /* Upper limit of the middle
                                                * band: 470 MHz */
#define LOW_BAND_LOWER_LIMIT        299000000  /* Lower limit of the low
                                                * band: 300 MHz */
#define LOW_BAND_UPPER_LIMIT        349100000  /* Upper limit of the low
                                                * band: 348 MHz */
#define VERY_LOW_BAND_LOWER_LIMIT   149000000  /* Lower limit of the very
                                                * low band: 150 MHz */
#define VERY_LOW_BAND_UPPER_LIMIT   175100000  /* Upper limit of the very
                                                * low band: 174 MHz */

#define IS_FREQUENCY_BAND_HIGH(frequency) \
  ((frequency) >= HIGH_BAND_LOWER_LIMIT && (frequency) <= HIGH_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_MIDDLE(frequency) \
  ((frequency) >= MIDDLE_BAND_LOWER_LIMIT && (frequency) <= MIDDLE_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_LOW(frequency) \
  ((frequency) >= LOW_BAND_LOWER_LIMIT && (frequency) <= LOW_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_VERY_LOW(frequency) \
  ((frequency) >= VERY_LOW_BAND_LOWER_LIMIT && (frequency)<=VERY_LOW_BAND_UPPER_LIMIT)

#define IS_FREQUENCY_BAND(frequency) \
  (IS_FREQUENCY_BAND_HIGH(frequency)|| IS_FREQUENCY_BAND_MIDDLE(frequency)|| \
   IS_FREQUENCY_BAND_LOW(frequency) || IS_FREQUENCY_BAND_VERY_LOW(frequency))

/* Radio_IF_Offset.
 *
 * This represents the IF_OFFSET_ANA inorder to have an intermediate frequency
 * of 480 kHz.
 */

#define IF_OFFSET_ANA(fxo)          (lroundf(480140.0 / (fxo) * 12288 - 64.0))

/* Radio_FC_Offset */

#define F_OFFSET_DIVIDER            262144     /* 2^18 factor dividing fxo
                                                * in foffset formula */
#define PPM_FACTOR                  1000000    /* 10^6 factor to use with
                                                * Xtal_offset_ppm */

#define F_OFFSET_LOWER_LIMIT(fxo)   ((-(int32_t)fxo) / F_OFFSET_DIVIDER * 2048)
#define F_OFFSET_UPPER_LIMIT(fxo)   ((int32_t)(fxo / F_OFFSET_DIVIDER * 2047))

#define IS_FREQUENCY_OFFSET(offset, fxo) \
  (offset >= F_OFFSET_LOWER_LIMIT(fxo) && offset <= F_OFFSET_UPPER_LIMIT(fxo))

/* Radio_Channel_Space */

#define CHSPACE_DIVIDER             32768      /* 2^15 factor dividing fxo in
                                                * channel space formula */

#define IS_CHANNEL_SPACE(chspace, fxo) \
  (chspace <= (fxo / 32768 * 255))

/* Radio_Datarate */

#define MINIMUM_DATARATE            100        /* Minimum datarate
                                                * supported by SPIRIT1 100 bps */
#define MAXIMUM_DATARATE            510000     /* Maximum datarate
                                                * supported by SPIRIT1 500 kbps */

#define IS_DATARATE(datarate) \
  (datarate >= MINIMUM_DATARATE && datarate <= MAXIMUM_DATARATE)

/* Radio_Frequency_Deviation */

#define F_DEV_MANTISSA_UPPER_LIMIT  7     /* Maximum value for the
                                                 * mantissa in frequency
                                                 * deviation formula */
#define F_DEV_EXPONENT_UPPER_LIMIT  9     /* Maximum value for the
                                                 * exponent in frequency
                                                 * deviation formula */

#define F_DEV_LOWER_LIMIT(fxo)      (fxo >> 16)
#define F_DEV_UPPER_LIMIT(fxo)      ((fxo * 15) >> 10)

#define IS_F_DEV(fdev,fxo) \
  (fdev >= F_DEV_LOWER_LIMIT(fxo) && fdev <= F_DEV_UPPER_LIMIT(fxo))

/* Radio_Channel_Bandwidth
 *
 *  CH_BW_LOWER_LIMIT - Minimum value of the channel filter bandwidth
 *  CH_BW_UPPER_LIMIT - Maximum value of the channel filter bandwidth
 */

#define CH_BW_LOWER_LIMIT(fxo)      1100 * (fxo / 1000000) / 26
#define CH_BW_UPPER_LIMIT(fxo)      800100*(fxo / 1000000) / 26

#define IS_CH_BW(bw,fxo) \
  ((bw) >= CH_BW_LOWER_LIMIT(fxo) && (bw) <= CH_BW_UPPER_LIMIT(fxo))

/* Radio_Power_Amplifier */

#define IS_PA_MAX_INDEX(index)      ((index) <= 7)
#define IS_PAPOWER_DBM(patable)     ((patable) >= (-31) && (patable) <= (12))
#define IS_PAPOWER(patable)         ((patable) <= 90)
#define IS_PA_STEP_WIDTH(width)     ((width) >= 1 && (width) <= 4)

/* Radio_Automatic_Frequency_Correction */

#define IS_AFC_FAST_GAIN(gain)      ((gain) <= 5)
#define IS_AFC_SLOW_GAIN(gain)      ((gain) <= 15)
#define IS_AFC_PD_LEAKAGE(leakage)  ((leakage) <= 31)

/* Radio_Automatic_Gain_Control */

#define AGC_MEASURE_TIME_UPPER_LIMIT_US(fxo) \
  (393216.0 / fxo)

#define IS_AGC_MEASURE_TIME_US(time, fxo) \
  (time <= AGC_MEASURE_TIME_UPPER_LIMIT_US(fxo))

#define IS_AGC_MEASURE_TIME(time)   (time<=15)

#define AGC_HOLD_TIME_UPPER_LIMIT_US(fxo) \
  (756.0 / fxo)

#define IS_AGC_HOLD_TIME_US(time,fxo) \
  (time <= AGC_HOLD_TIME_UPPER_LIMIT_US(fxo))

#define IS_AGC_HOLD_TIME(time)      (time <= 63)

#define IS_AGC_THRESHOLD(threshold) (threshold <=1 5)

/* Radio_Clock_Recovery */

#define IS_CLK_REC_P_GAIN(gain)     ((gain) <= 7)
#define IS_CLK_REC_I_GAIN(gain)     ((gain) <= 15)

/* Other acros used in assertions */

#define IS_XTAL_FLAG(flag) \
  (((flag) == XTAL_FLAG_24_MHz) || ((flag) == XTAL_FLAG_26_MHz))
#define IS_BAND_SELECTED(band) \
  (((band) == HIGH_BAND)        || ((band) == MIDDLE_BAND)      || \
   ((band) == LOW_BAND)         || ((band) == VERY_LOW_BAND))
#define IS_MODULATION_SELECTED(mod) \
  (((mod)  == FSK)              || ((mod)  == GFSK_BT05)        || \
   ((mod)  == GFSK_BT1)         || ((mod)  == ASK_OOK)          || \
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

/* SPIRIT Band enumeration */

enum spirit_bandselect_e
{
  HIGH_BAND        = 0x00,  /* High_Band selected: from 779 MHz to 915 MHz */
  MIDDLE_BAND      = 0x01,  /* Middle Band selected: from 387 MHz to 470 MHz */
  LOW_BAND         = 0x02,  /* Low Band selected: from 300 MHz to 348 MHz */
  VERY_LOW_BAND    = 0x03   /* Vary low Band selected: from 150 MHz to 174 MHz */
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
                             * NxFREQUENCY_STEPS, where frequency STEPS is
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
                              enum xtal_flag_e xtalflag);

/******************************************************************************
 * Name: spirit_radio_get_xtalflag
 *
 * Description:
 *   Returns the Xtal configuration in the ANA_FUNC_CONF0 register.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   XtalFrequency Settled Xtal configuration.
 *
 ******************************************************************************/

enum xtal_flag_e spirit_radio_get_xtalflag(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_radio_get_synthword
 *
 * Description:
 *   Returns the synth word.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   32-bit Synth word.  Errors are not reported.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_synthword(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_radio_set_synthword
 *
 * Description:
 *   Sets the SYNTH registers.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   synthword - The synth word to write in the SYNTH[3:0] registers.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_synthword(FAR struct spirit_library_s *spirit,
                               uint32_t synthword);

/******************************************************************************
 * Name: spirit_radio_set_band
 *
 * Description:
 *   Sets the operating band.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   band   - The band to set.  This parameter can be one of following values:
 *             HIGH_BAND      High_Band selected: from 779 MHz to 915 MHz
 *             MIDDLE_BAND:   Middle Band selected: from 387 MHz to 470 MHz
 *             LOW_BAND:      Low Band selected: from 300 MHz to 348 MHz
 *             VERY_LOW_BAND: Very low Band selected: from 150 MHz to 174 MHz
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_band(FAR struct spirit_library_s *spirit,
                          enum spirit_bandselect_e band);

/******************************************************************************
 * Name: spirit_radio_get_band
 *
 * Description:
 *   Returns the operating band.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   BandSelect Settled band.  This returned value may be one of the
 *   following values:
 *     HIGH_BAND      High_Band selected: from 779 MHz to 915 MHz
 *     MIDDLE_BAND:   Middle Band selected: from 387 MHz to 470 MHz
 *     LOW_BAND:      Low Band selected: from 300 MHz to 348 MHz
 *     VERY_LOW_BAND: Very low Band selected: from 150 MHz to 174 MHz
 *
 ******************************************************************************/

enum spirit_bandselect_e
  spirit_radio_get_band(FAR struct spirit_library_s *spirit);

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
                                   uint32_t fbase);

/******************************************************************************
 * Name: 
 *
 * Description:
 *   Returns the base carrier frequency.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Base carrier frequency expressed in Hz as unsigned word.
 *
 ******************************************************************************/

uint32_t spirit_radio_get_basefrequency(FAR struct spirit_library_s *spirit);

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
                                  FAR uint8_t *pce);

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
                                 FAR uint8_t *pce);

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
                                     FAR uint8_t *pce);

/******************************************************************************
 * Name: spirit_radio_dbm2reg
 *
 * Description:
 *   Returns the PA register value that corresponds to the passed dBm power.
 *
 *   NOTE: The power interpolation curves used by this function have been
 *   extracted by measurements done on the divisional evaluation boards.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   fbase    Frequency base expressed in Hz.
 *   powerdbm Desired power in dBm.
 *
 * Returned Value:
 *   Register value as byte.
 *
 ******************************************************************************/

uint8_t spirit_radio_dbm2reg(FAR struct spirit_library_s *spirit,
                             uint32_t fbase, float powerdbm);

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

/******************************************************************************
 * Name: spirit_radio_set_refdiv
 *
 * Description:
 *   Enables or Disables the synthesizer reference divider.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate new state for synthesizer reference divider.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_radio_set_refdiv(FAR struct spirit_library_s *spirit,
                            enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_radio_get_refdiv
 *
 * Description:
 *   Get the the synthesizer reference divider state.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.  Errors are not reported.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_radio_get_refdiv(FAR struct spirit_library_s *spirit);

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
                                   enum spirit_functional_state_e newstate);

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
  spirit_radio_isenabled_digdivider(FAR struct spirit_library_s *spirit);

#endif /* __INCLUDE_NUTT_WIRELESS_SPIRIT_SPIRIT_RADIO_H*/
