
/**
 * @file    cs43l22.h
 * @brief   CS43L22 Driver macros and structures.
 *
 * @addtogroup CS43L22
 * @{
 */

#ifndef _CS43L22_H_
#define _CS43L22_H_

#if HAL_USE_CS43L22 || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Chip I.D. and revision register */
#define CS43L22_REG_GET_ID 0x01
/* Chip I.D. and revision register - Chip identifier */
#define CS43L22_CHIP_ID_MASK ((uint8_t)0xf8)
/* Chip I.D. and revision register - CS43L22 identifier */
#define CS43L22_CHIP_ID ((uint8_t)0xe0)
/* Chip I.D. and revision register - Revision level - Mask */
#define CS43L22_CHIP_REV_MASK ((uint8_t)0x07)
/* Chip I.D. and revision register - Revision level - A1 */
#define CS43L22_CHIP_REV_A1 ((uint8_t)0x01)
/* Chip I.D. and revision register - Revision level - B1 */
#define CS43L22_CHIP_REV_B1 ((uint8_t)0x03)
/* Chip I.D. and revision register - Revision level - A0 */
#define CS43L22_CHIP_REV_A0 ((uint8_t)0x00)
/* Chip I.D. and revision register - Revision level - B0 */
#define CS43L22_CHIP_REV_B0 ((uint8_t)0x02)




/* Power control 2 - Speaker A - Enabled if switch high */
#define CS43L22_PWR_SWHI ((uint8_t)0x01)
/* Power control 2 - Speaker A - Disabled */
#define CS43L22_PWR_OFF ((uint8_t)0x03)
/* Power control 2 - Speaker A - Enabled if switch low */
#define CS43L22_PWR_SWLO ((uint8_t)0x00)
/* Power control 2 - Speaker A - Enabled */
#define CS43L22_PWR_ON ((uint8_t)0x02)


/* Clocking control */
#define CS43L22_REG_CLOCK_CTL 0x05
/* Clocking control - Autodetection - Enabled */
#define CS43L22_CLK_AUTO_ON ((uint8_t)0x80)
/* Clocking control - Autodetection - Disabled */
#define CS43L22_CLK_AUTO_OFF ((uint8_t)0x00)
/* Clocking control - Speed mode - Mask */
#define CS43L22_CLK_SPD_MASK ((uint8_t)0x60)
/* Clocking control - Speed mode - Single speed */
#define CS43L22_CLK_SPD_SSM ((uint8_t)0x20)
/* Clocking control - Speed mode - Half speed */
#define CS43L22_CLK_SPD_HSM ((uint8_t)0x40)
/* Clocking control - Speed mode - Double speed */
#define CS43L22_CLK_SPD_DSM ((uint8_t)0x00)
/* Clocking control - Speed mode - Quarter speed */
#define CS43L22_CLK_SPD_QSM ((uint8_t)0x60)
/* Clocking control - 32KHz group - Disabled */
#define CS43L22_CLK_32K_OFF ((uint8_t)0x00)
/* Clocking control - 32KHz group - Enabled */
#define CS43L22_CLK_32K_ON ((uint8_t)0x10)
/* Clocking control - 27MHz video clock - Enabled */
#define CS43L22_CLK_VC_ON ((uint8_t)0x08)
/* Clocking control - 27MHz video clock - Disabled */
#define CS43L22_CLK_VC_OFF ((uint8_t)0x00)
/* Clocking control - Internal MCLK/LRCK ratio - Mask */
#define CS43L22_CLK_MLR_MASK ((uint8_t)0x06)
/* Clocking control - Internal MCLK/LRCK ratio - 68 */
#define CS43L22_CLK_MLR_68 ((uint8_t)0x06)
/* Clocking control - Internal MCLK/LRCK ratio - 66 */
#define CS43L22_CLK_MLR_66 ((uint8_t)0x04)
/* Clocking control - Internal MCLK/LRCK ratio - 64 */
#define CS43L22_CLK_MLR_64 ((uint8_t)0x00)
/* Clocking control - Internal MCLK/LRCK ratio - 62 */
#define CS43L22_CLK_MLR_62 ((uint8_t)0x02)
/* Clocking control - Master clock /= 2 - Enabled */
#define CS43L22_CLK_MCDIV_ON ((uint8_t)0x01)
/* Clocking control - Master clock /= 2 - Disabled */
#define CS43L22_CLK_MCDIV_OFF ((uint8_t)0x00)

/* Interface control 1 */
#define CS43L22_REG_INT_CTL1 0x06
/* Interface control 1 - Slave mode */
#define CS43L22_IC1_SLAVE ((uint8_t)0x00)
/* Interface control 1 - Master mode */
#define CS43L22_IC1_MASTER ((uint8_t)0x80)
/* Interface control 1 - SCLK inverted polarity - Enabled */
#define CS43L22_IC1_SCPOL_ON ((uint8_t)0x40)
/* Interface control 1 - SCLK inverted polarity - Disabled */
#define CS43L22_IC1_SCPOL_OFF ((uint8_t)0x00)
/* Interface control 1 - DSP - Disabled */
#define CS43L22_IC1_DSP_OFF ((uint8_t)0x00)
/* Interface control 1 - DSP - Enabled */
#define CS43L22_IC1_DSP_ON ((uint8_t)0x10)
/* Interface control 1 - DAC interface format - Right-justified */
#define CS43L22_IC1_DIF_RJ ((uint8_t)0x08)
/* Interface control 1 - DAC interface format - I2S, up to 24 bits */
#define CS43L22_IC1_DIF_I2S ((uint8_t)0x04)
/* Interface control 1 - DAC interface format - Left-justified, up to 24 bits */
#define CS43L22_IC1_DIF_LJ ((uint8_t)0x00)
/* Interface control 1 - Audio word length - 24-bit (DSP), 20-bit (RJ) */
#define CS43L22_IC1_AWL_24 ((uint8_t)0x01)
/* Interface control 1 - Audio word length - 16-bit (DSP), 16-bit (RJ) */
#define CS43L22_IC1_AWL_16 ((uint8_t)0x03)
/* Interface control 1 - Audio word length - 32-bit (DSP), 24-bit (RJ) */
#define CS43L22_IC1_AWL_32 ((uint8_t)0x00)
/* Interface control 1 - Audio word length - 20-bit (DSP), 18-bit (RJ) */
#define CS43L22_IC1_AWL_20 ((uint8_t)0x02)

/* Interface control 2 */
#define CS43L22_REG_INT_CTL2 0x07
/* Interface control 2 - SCLK = MCLK? - Equal */
#define CS43L22_IC2_SEM_ON ((uint8_t)0x40)
/* Interface control 2 - SCLK = MCLK? - Derived */
#define CS43L22_IC2_SEM_OFF ((uint8_t)0x00)
/* Interface control 2 - Speaker/Headphone switch invert - Enabled */
#define CS43L22_IC2_SHSI_ON ((uint8_t)0x08)
/* Interface control 2 - Speaker/Headphone switch invert - Disabled */
#define CS43L22_IC2_SHSI_OFF ((uint8_t)0x00)

/* Passthrough A select */
#define CS43L22_REG_PSELA 0x08
/* Passthrough A select - Mask */
#define CS43L22_PSELA_MASK ((uint8_t)0x0f)
/* Passthrough A select - AIN4 */
#define CS43L22_PSELA_AIN4 ((uint8_t)0x08)
/* Passthrough A select - AIN3 */
#define CS43L22_PSELA_AIN3 ((uint8_t)0x04)
/* Passthrough A select - AIN1 */
#define CS43L22_PSELA_AIN1 ((uint8_t)0x01)
/* Passthrough A select - NONE */
#define CS43L22_PSELA_NONE ((uint8_t)0x00)
/* Passthrough A select - AIN2 */
#define CS43L22_PSELA_AIN2 ((uint8_t)0x02)

/* Passthrough B select */
#define CS43L22_REG_PSELB 0x09
/* Passthrough B select - Mask */
#define CS43L22_PSELB_MASK ((uint8_t)0x0f)
/* Passthrough B select - AIN4 */
#define CS43L22_PSELB_AIN4 ((uint8_t)0x08)
/* Passthrough B select - AIN3 */
#define CS43L22_PSELB_AIN3 ((uint8_t)0x04)
/* Passthrough B select - AIN1 */
#define CS43L22_PSELB_AIN1 ((uint8_t)0x01)
/* Passthrough B select - NONE */
#define CS43L22_PSELB_NONE ((uint8_t)0x00)
/* Passthrough B select - AIN2 */
#define CS43L22_PSELB_AIN2 ((uint8_t)0x02)

/* Analog zero cross / soft ramp */
#define CS43L22_REG_AZCSR 0x0a
/* Analog zero cross / soft ramp - Channel B soft ramp - Enabled */
#define CS43L22_AZCSR_SRB_ON ((uint8_t)0x08)
/* Analog zero cross / soft ramp - Channel B soft ramp - Disabled */
#define CS43L22_AZCSR_SRB_OFF ((uint8_t)0x00)
/* Analog zero cross / soft ramp - Channel A soft ramp - Enabled */
#define CS43L22_AZCSR_SRA_ON ((uint8_t)0x04)
/* Analog zero cross / soft ramp - Channel A soft ramp - Disabled */
#define CS43L22_AZCSR_SRA_OFF ((uint8_t)0x00)
/* Analog zero cross / soft ramp - Channel B zero cross - Disabled */
#define CS43L22_AZCSR_ZCB_OFF ((uint8_t)0x00)
/* Analog zero cross / soft ramp - Channel B zero cross - Enabled */
#define CS43L22_AZCSR_ZCB_ON ((uint8_t)0x02)
/* Analog zero cross / soft ramp - Channel A zero cross - Enabled */
#define CS43L22_AZCSR_ZCA_ON ((uint8_t)0x01)
/* Analog zero cross / soft ramp - Channel A zero cross - Disabled */
#define CS43L22_AZCSR_ZCA_OFF ((uint8_t)0x00)

/* Passthrough gang control */
#define CS43L22_REG_PGC 0x0c
/* Passthrough gang control - Enabled */
#define CS43L22_PGC_ON ((uint8_t)0x80)
/* Passthrough gang control - Disabled */
#define CS43L22_PGC_OFF ((uint8_t)0x00)

/* Playback control 1 */
#define CS43L22_REG_PB_CTL1 0x0d
/* Playback control 1 - Headphone analog gain - Mask */
#define CS43L22_PB1_HGAIN_MASK ((uint8_t)0xe0)
/* Playback control 1 - Ganged volume control - Disabled */
#define CS43L22_PB1_SAMEVOLUME_OFF ((uint8_t)0x00)
/* Playback control 1 - Ganged volume control - Enabled */
#define CS43L22_PB1_SAMEVOLUME_ON ((uint8_t)0x10)
/* Playback control 1 - Invert channel B polarity - Enabled */
#define CS43L22_PB1_INVB_ON ((uint8_t)0x08)
/* Playback control 1 - Invert channel B polarity - Disabled */
#define CS43L22_PB1_INVB_OFF ((uint8_t)0x00)
/* Playback control 1 - Invert channel A polarity - Enabled */
#define CS43L22_PB1_INVA_ON ((uint8_t)0x04)
/* Playback control 1 - Invert channel A polarity - Disabled */
#define CS43L22_PB1_INVA_OFF ((uint8_t)0x00)
/* Playback control 1 - Channel B muted - Disabled */
#define CS43L22_PB1_MUTEB_OFF ((uint8_t)0x00)
/* Playback control 1 - Channel B muted - Enabled */
#define CS43L22_PB1_MUTEB_ON ((uint8_t)0x02)
/* Playback control 1 - Channel A muted - Enabled */
#define CS43L22_PB1_MUTEA_ON ((uint8_t)0x01)
/* Playback control 1 - Channel A muted - Disabled */
#define CS43L22_PB1_MUTEA_OFF ((uint8_t)0x00)

/* Miscellaneous controls */
#define CS43L22_REG_MISC_CTL 0x0e
/* Miscellaneous controls - Passthrough analog B - Enabled */
#define CS43L22_MISC_PTHRUB_ON ((uint8_t)0x80)
/* Miscellaneous controls - Passthrough analog B - Disabled */
#define CS43L22_MISC_PTHRUB_OFF ((uint8_t)0x00)
/* Miscellaneous controls - Passthrough analog A - Enabled */
#define CS43L22_MISC_PTHRUA_ON ((uint8_t)0x40)
/* Miscellaneous controls - Passthrough analog A - Disabled */
#define CS43L22_MISC_PTHRUA_OFF ((uint8_t)0x00)
/* Miscellaneous controls - Passthrough mute B - Enabled */
#define CS43L22_MISC_PTMUTEB_ON ((uint8_t)0x20)
/* Miscellaneous controls - Passthrough mute B - Disabled */
#define CS43L22_MISC_PTMUTEB_OFF ((uint8_t)0x00)
/* Miscellaneous controls - Passthrough mute A - Disabled */
#define CS43L22_MISC_PTMUTEA_OFF ((uint8_t)0x00)
/* Miscellaneous controls - Passthrough mute A - Enabled */
#define CS43L22_MISC_PTMUTEA_ON ((uint8_t)0x10)
/* Miscellaneous controls - Freeze registers - Enabled */
#define CS43L22_MISC_FREEZE_ON ((uint8_t)0x08)
/* Miscellaneous controls - Freeze registers - Disabled */
#define CS43L22_MISC_FREEZE_OFF ((uint8_t)0x00)
/* Miscellaneous controls - De-emphasis filter - Enabled */
#define CS43L22_MISC_DEEMPHASIS_ON ((uint8_t)0x04)
/* Miscellaneous controls - De-emphasis filter - Disabled */
#define CS43L22_MISC_DEEMPHASIS_OFF ((uint8_t)0x00)
/* Miscellaneous controls - Digital soft ramp - Disabled */
#define CS43L22_MISC_DSR_OFF ((uint8_t)0x00)
/* Miscellaneous controls - Digital soft ramp - Enabled */
#define CS43L22_MISC_DSR_ON ((uint8_t)0x02)
/* Miscellaneous controls - Digital zero cross - Enabled */
#define CS43L22_MISC_DZC_ON ((uint8_t)0x01)
/* Miscellaneous controls - Digital zero cross - Disabled */
#define CS43L22_MISC_DZC_OFF ((uint8_t)0x00)

/* Playback control 2 */
#define CS43L22_REG_PB_CTL2 0x0f
/* Playback control 2 - Headphone B mute - Enabled */
#define CS43L22_PB2_HPB_MUTE_ON ((uint8_t)0x80)
/* Playback control 2 - Headphone B mute - Disabled */
#define CS43L22_PB2_HPB_MUTE_OFF ((uint8_t)0x00)
/* Playback control 2 - Headphone A mute - Enabled */
#define CS43L22_PB2_HPA_MUTE_ON ((uint8_t)0x40)
/* Playback control 2 - Headphone A mute - Disabled */
#define CS43L22_PB2_HPA_MUTE_OFF ((uint8_t)0x00)
/* Playback control 2 - Speaker B mute - Enabled */
#define CS43L22_PB2_SPKB_MUTE_ON ((uint8_t)0x20)
/* Playback control 2 - Speaker B mute - Disabled */
#define CS43L22_PB2_SPKB_MUTE_OFF ((uint8_t)0x00)
/* Playback control 2 - Speaker A mute - Disabled */
#define CS43L22_PB2_SPKA_MUTE_OFF ((uint8_t)0x00)
/* Playback control 2 - Speaker A mute - Enabled */
#define CS43L22_PB2_SPKA_MUTE_ON ((uint8_t)0x10)
/* Playback control 2 - Ganged speaker volume settings - Enabled */
#define CS43L22_PB2_SPK_GANG_ON ((uint8_t)0x08)
/* Playback control 2 - Ganged speaker volume settings - Disabled */
#define CS43L22_PB2_SPK_GANG_OFF ((uint8_t)0x00)
/* Playback control 2 - Speaker channel swap - Enabled */
#define CS43L22_PB2_SPK_SWAP_ON ((uint8_t)0x04)
/* Playback control 2 - Speaker channel swap - Disabled */
#define CS43L22_PB2_SPK_SWAP_OFF ((uint8_t)0x00)
/* Playback control 2 - Speaker mono mode - Disabled */
#define CS43L22_PB2_SPK_MONO_OFF ((uint8_t)0x00)
/* Playback control 2 - Speaker mono mode - Enabled */
#define CS43L22_PB2_SPK_MONO_ON ((uint8_t)0x02)
/* Playback control 2 - Speaker mute 50/50 control - Enabled */
#define CS43L22_PB2_SPK_M50_ON ((uint8_t)0x01)
/* Playback control 2 - Speaker mute 50/50 control - Disabled */
#define CS43L22_PB2_SPK_M50_OFF ((uint8_t)0x00)

/* Passthrough volume (channel A) */
#define CS43L22_REG_PASS_A_VOL 0x14

/* Passthrough volume (channel B) */
#define CS43L22_REG_PASS_B_VOL 0x15

/* PCM channel A control */
#define CS43L22_REG_PCM_A 0x1a
/* PCM channel A control - Mute channel - Enabled */
#define CS43L22_PCM_A_MUTE_ON ((uint8_t)0x80)
/* PCM channel A control - Mute channel - Disabled */
#define CS43L22_PCM_A_MUTE_OFF ((uint8_t)0x00)
/* PCM channel A control - Channel volume - Mask */
#define CS43L22_PCM_A_VOLUME_MASK ((uint8_t)0x7f)

/* PCM channel B control */
#define CS43L22_REG_PCM_B 0x1b
/* PCM channel B control - Mute channel - Enabled */
#define CS43L22_PCM_B_MUTE_ON ((uint8_t)0x80)
/* PCM channel B control - Mute channel - Disabled */
#define CS43L22_PCM_B_MUTE_OFF ((uint8_t)0x00)
/* PCM channel B control - Channel volume - Mask */
#define CS43L22_PCM_B_VOLUME_MASK ((uint8_t)0x7f)

/* Beep control 1 */
#define CS43L22_REG_BEEP_CTL1 0x1c
/* Beep control 1 - Frequency - Mask */
#define CS43L22_BEEP_FREQ_MASK ((uint8_t)0xf0)
/* Beep control 1 - Duration - Mask */
#define CS43L22_BEEP_ONTIME_MASK ((uint8_t)0x0f)

/* Beep control 2 */
#define CS43L22_REG_BEEP_CTL2 0x1d
/* Beep control 2 - Off time - Mask */
#define CS43L22_BEEP_OFFTIME_MASK ((uint8_t)0xf0)
/* Beep control 2 - Volume - Mask */
#define CS43L22_BEEP_VOL_MASK ((uint8_t)0x0f)

/* Beep & tone configuration */
#define CS43L22_REG_BEEP_TONE_CFG 0x1e
/* Beep & tone configuration - Beep configuration - Multiple */
#define CS43L22_B3_BEEP_CFG_MULTI ((uint8_t)0x80)
/* Beep & tone configuration - Beep configuration - Single */
#define CS43L22_B3_BEEP_CFG_SINGLE ((uint8_t)0x40)
/* Beep & tone configuration - Beep configuration - Off */
#define CS43L22_B3_BEEP_CFG_OFF ((uint8_t)0x00)
/* Beep & tone configuration - Beep configuration - Continuous */
#define CS43L22_B3_BEEP_CFG_CONT ((uint8_t)0xc0)
/* Beep & tone configuration - Beep mix disable - OFF */
#define CS43L22_B3_BEEP_NOMIX_OFF ((uint8_t)0x20)
/* Beep & tone configuration - Beep mix disable - ON */
#define CS43L22_B3_BEEP_NOMIX_ON ((uint8_t)0x00)
/* Beep & tone configuration - Treble corner frequency - 7KHZ */
#define CS43L22_B3_TREBLE_CF_7KHZ ((uint8_t)0x08)
/* Beep & tone configuration - Treble corner frequency - 5KHZ */
#define CS43L22_B3_TREBLE_CF_5KHZ ((uint8_t)0x00)
/* Beep & tone configuration - Treble corner frequency - 15KHZ */
#define CS43L22_B3_TREBLE_CF_15KHZ ((uint8_t)0x18)
/* Beep & tone configuration - Treble corner frequency - 10KHZ */
#define CS43L22_B3_TREBLE_CF_10KHZ ((uint8_t)0x10)
/* Beep & tone configuration - Bass corner frequency - 250HZ */
#define CS43L22_B3_BASS_CF_250HZ ((uint8_t)0x06)
/* Beep & tone configuration - Bass corner frequency - 200HZ */
#define CS43L22_B3_BASS_CF_200HZ ((uint8_t)0x04)
/* Beep & tone configuration - Bass corner frequency - 50HZ */
#define CS43L22_B3_BASS_CF_50HZ ((uint8_t)0x00)
/* Beep & tone configuration - Bass corner frequency - 100HZ */
#define CS43L22_B3_BASS_CF_100HZ ((uint8_t)0x02)
/* Beep & tone configuration - Tone control - Enabled */
#define CS43L22_B3_TONECTL_ON ((uint8_t)0x01)
/* Beep & tone configuration - Tone control - Disabled */
#define CS43L22_B3_TONECTL_OFF ((uint8_t)0x00)

/* Tone control */
#define CS43L22_REG_TONE_CTL 0x1f
/* Tone control - Treble gain - Mask */
#define CS43L22_TCTL_TREBLE_MASK ((uint8_t)0xf0)
/* Tone control - Bass gain - Mask */
#define CS43L22_TCTL_BASS_MASK ((uint8_t)0x0f)
/* Master volume control (channel A) */
#define CS43L22_REG_MASTER_VOLUME_A 0x20

/* PCM channel swap */
#define CS43L22_REG_PCM_SWAP 0x26
/* PCM channel swap - Channel A - (Left + Right) / 2 */
#define CS43L22_PCS_A_MIX ((uint8_t)0x40)
/* PCM channel swap - Channel A - Left */
#define CS43L22_PCS_A_LEFT ((uint8_t)0x00)
/* PCM channel swap - Channel A - Right */
#define CS43L22_PCS_A_RIGHT ((uint8_t)0xc0)
/* PCM channel swap - Channel B - Right */
#define CS43L22_PCS_B_RIGHT ((uint8_t)0x00)
/* PCM channel swap - Channel B - (Left + Right) / 2 */
#define CS43L22_PCS_B_MIX ((uint8_t)0x10)
/* PCM channel swap - Channel B - Left */
#define CS43L22_PCS_B_LEFT ((uint8_t)0x30)

/* Limiter control 1 */
#define CS43L22_REG_LIM_CTL1 0x27
/* Limiter control 1 - Maximum threshold - Mask */
#define CS43L22_LIM1_MAX_MASK ((uint8_t)0xe0)
/* Limiter control 1 - Cushion threshold - Mask */
#define CS43L22_LIM1_CUSHION_MASK ((uint8_t)0x1c)
/* Limiter control 1 - Soft ramp - Use digital soft ramp */
#define CS43L22_LIM1_SRD_OFF ((uint8_t)0x00)
/* Limiter control 1 - Soft ramp - Ignore digital soft ramp */
#define CS43L22_LIM1_SRD_ON ((uint8_t)0x02)
/* Limiter control 1 - Zero cross - Ignore digital zero cross */
#define CS43L22_LIM1_ZCD_ON ((uint8_t)0x01)
/* Limiter control 1 - Zero cross - Use digital zero cross */
#define CS43L22_LIM1_ZCD_OFF ((uint8_t)0x00)

/* Limiter control 2 */
#define CS43L22_REG_LIM_CTL2 0x28
/* Limiter control 2 - Peak detect and limiter - Enabled */
#define CS43L22_LIM2_ON ((uint8_t)0x80)
/* Limiter control 2 - Peak detect and limiter - Disabled */
#define CS43L22_LIM2_OFF ((uint8_t)0x00)
/* Limiter control 2 - Limit both channels - Enabled */
#define CS43L22_LIM2_LAC_ON ((uint8_t)0x40)
/* Limiter control 2 - Limit both channels - Disabled */
#define CS43L22_LIM2_LAC_OFF ((uint8_t)0x00)
/* Limiter control 2 - Release rate - Mask */
#define CS43L22_LIM2_RRATE_MASK ((uint8_t)0x3f)

/* Limiter control 3 */
#define CS43L22_REG_LIM_CTL3 0x29
/* Limiter control 3 - Attack rate - Mask */
#define CS43L22_LIM3_ARATE_MASK ((uint8_t)0x3f)

/* Status (RO) */
#define CS43L22_REG_STATUS 0x2e
/* Status (RO) - Serial port clock error */
#define CS43L22_STATUS_SERIAL_ERROR ((uint8_t)0x40)
/* Status (RO) - DSP A overflow */
#define CS43L22_STATUS_DSP_A_OVERFLOW ((uint8_t)0x20)
/* Status (RO) - DSP B overflow */
#define CS43L22_STATUS_DSP_B_OVERFLOW ((uint8_t)0x10)
/* Status (RO) - PCM A overflow */
#define CS43L22_STATUS_PCM_A_OVERFLOW ((uint8_t)0x08)
/* Status (RO) - PCM B overflow */
#define CS43L22_STATUS_PCM_B_OVERFLOW ((uint8_t)0x04)

/* Battery compensation */
#define CS43L22_REG_BATTERY_COMPENSATION 0x2f
/* Battery compensation - Enabled */
#define CS43L22_BC_ON ((uint8_t)0x80)
/* Battery compensation - Disabled */
#define CS43L22_BC_OFF ((uint8_t)0x00)
/* Battery compensation - VP voltage level monitor - Enabled */
#define CS43L22_BC_VPM_ON ((uint8_t)0x40)
/* Battery compensation - VP voltage level monitor - Disabled */
#define CS43L22_BC_VPM_OFF ((uint8_t)0x00)
/* Battery compensation - VP reference - Mask */
#define CS43L22_BC_VPREF_MASK ((uint8_t)0x0f)

/* Battery level (RO) */
#define CS43L22_REG_BATTERY_LEVEL 0x30

/* Speaker status (RO) */
#define CS43L22_REG_SPKR_STATUS 0x31
/* Speaker status (RO) - Channel A overload detected */
#define CS43L22_SPKS_OVERLOAD_A ((uint8_t)0x20)
/* Speaker status (RO) - Channel B overload detected */
#define CS43L22_SPKS_OVERLOAD_B ((uint8_t)0x10)
/* Speaker status (RO) - Speaker/headphones pin status */
#define CS43L22_SPKS_SPKHP_SWITCH ((uint8_t)0x08)

/* Charge pump frequency */
#define CS43L22_REG_CPFREQ 0x34
/* Charge pump frequency - Mask */
#define CS43L22_CPFREQ_MASK ((uint8_t)0xf0)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/
/**
 * @brief
 * @note
 */
#if !defined(CS43L22_I2C_TIMEOUT) || defined(__DOXYGEN__)
#define CS43L22_I2C_TIMEOUT 1000
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  CS43L22_UNINIT = 0,                   /**< Not initialized.                   */
  CS43L22_STOP = 1,                     /**< Stopped.                           */
  CS43L22_READY = 2,                    /**< Ready.                             */
  CS43L22_ACTIVE = 3,                   /**< Active.                            */
  CS43L22_COMPLETE = 4                  /**< Transmission complete.             */
} cs43l22state_t;
/**
 * @brief   Type of a structure representing an CS43L22Driver driver.
 */
typedef struct CS43L22Driver CS43L22Driver;

/**
 * @brief   I2S notification callback type.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] offset    offset in buffers of the data to read/write
 * @param[in] n         number of samples to read/write
 */
typedef void (*cs43l22_reset_callback_t)(void);

/**
 * @brief   I2S notification callback type.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] offset    offset in buffers of the data to read/write
 * @param[in] n         number of samples to read/write
 */
typedef void (*cs43l22_audio_reconfigure_callback_t)(uint16_t samplerate, uint8_t bitsPerSample);

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
	uint16_t ctrl_address;
    I2CDriver *ctrlp;
    I2SDriver *audiop;
	cs43l22_reset_callback_t reset_cb;
    cs43l22_audio_reconfigure_callback_t reconfigure_cb;
} CS43L22Config;


/**
 * @brief   Structure representing an CS43L22 driver.
 */
struct CS43L22Driver {
  /**
   * @brief   Driver state.
   */
	cs43l22state_t                state;
  /**
   * @brief   Current configuration data.
   */
  const CS43L22Config           *config;
  /* End of the mandatory fields.*/
};
/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void cs43l22Init(void);
  void cs43l22ObjectInit(CS43L22Driver *cs43l22p);
  void cs43l22Start(CS43L22Driver *cs43l22p, const CS43L22Config *config);
  void cs43l22Stop(CS43L22Driver *cs43l22p);
  void cs43l22ConfigureAudio(CS43L22Driver *cs43l22p, uint16_t samplerate, uint8_t bitsPerSample);
  void cs43l22StartTransfer(CS43L22Driver *cs43l22p);
  void cs43l22StopTransfer(CS43L22Driver *cs43l22p);
  void cs43l22PowerCtrl(CS43L22Driver *cs43l22p, uint8_t spk_a, uint8_t spk_b, uint8_t hd_a, uint8_t hd_b);
  void cs43l22MasterVolume(CS43L22Driver *cs43l22p, uint8_t a, uint8_t b);
  void cs43l22HeadphoneVolume(CS43L22Driver *cs43l22p, uint8_t a, uint8_t b);
  void cs43l22SpeakerVolume(CS43L22Driver *cs43l22p, uint8_t a, uint8_t b);

  void cs43l22Beep(CS43L22Driver *cs43l22p);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CS43L22 */

#endif /* _CS43L22_H_ */

/** @} */
