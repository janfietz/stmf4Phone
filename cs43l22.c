/**
 * @file    cs43l22.c
 * @brief   CS43L22 Driver code.
 *
 * @addtogroup CS43L22
 * @{
 */

#include "hal.h"
#include "cs43l22.h"

#if HAL_USE_CS43L22 || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
/* Power control 1 */
#define CS43L22_REG_PWR_CTL1 0x02
/* Power control 1 - Power up */
#define CS43L22_PWR1_UP ((uint8_t)0x9e)
/* Power control 1 - Power down */
#define CS43L22_PWR1_DOWN ((uint8_t)0x01)

/* Power control 2 */
#define CS43L22_REG_PWR_CTL2 0x04
/* Power control 2 - Speaker A - Mask */
#define CS43L22_PWR2_SPKA_OFFSET 0
#define CS43L22_PWR2_SPKA_MASK ((uint8_t)0x03)

/* Power control 2 - Speaker B - Mask */
#define CS43L22_PWR2_SPKB_OFFSET 2
#define CS43L22_PWR2_SPKB_MASK ((uint8_t)0x0c)

/* Power control 2 - Headphone A - Mask */
#define CS43L22_PWR2_HDA_OFFSET 4
#define CS43L22_PWR2_HDA_MASK ((uint8_t)0x30)

/* Power control 2 - Headphone B - Mask */
#define CS43L22_PWR2_HDB_OFFSET 6
#define CS43L22_PWR2_HDB_MASK ((uint8_t)0xc0)

/* Master volume control (channel B) */
#define CS43L22_REG_MASTER_VOLUME_B 0x21

/* Headphone volume control (channel A) */
#define CS43L22_REG_HP_VOLUME_A 0x22

/* Headphone volume control (channel B) */
#define CS43L22_REG_HP_VOLUME_B 0x23

/* Speaker volume control (channel A) */
#define CS43L22_REG_SPK_VOLUME_A 0x24

/* Speaker volume control (channel B) */
#define CS43L22_REG_SPK_VOLUME_B 0x25

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/* Send a command to the CS43L22 through I2C */
static msg_t _cs43l22_set(CS43L22Driver *cs43l22p, uint8_t reg , uint8_t value)
{
    uint8_t txBuffer[ 2 ];
    txBuffer[0] = reg;
    txBuffer[1] = value;
    msg_t rv = i2cMasterTransmitTimeout(
            cs43l22p->config->ctrlp,
            cs43l22p->config->ctrl_address,
            txBuffer , 2,
            NULL , 0,
            CS43L22_I2C_TIMEOUT);
    return rv;
}

static msg_t _cs43l22_get(CS43L22Driver *cs43l22p, uint8_t reg, uint8_t* value)
{
    msg_t rv = i2cMasterTransmitTimeout(
            cs43l22p->config->ctrlp,
            cs43l22p->config->ctrl_address,
            &reg , 1,
            value , 1,
            CS43L22_I2C_TIMEOUT);
    return rv;
}


void _cs43l22_set_volume(CS43L22Driver *cs43l22p, uint8_t reg_a, uint8_t volume_a, uint8_t reg_b, uint8_t volume_b )
{
    if (volume_a > 0xe6)
    {
    	volume_a -= 0xe7;
    } else {
    	volume_a += 0x19;
    }

    if (volume_b > 0xe6)
    {
    	volume_b -= 0xe7;
	} else {
		volume_b += 0x19;
	}
    _cs43l22_set(cs43l22p, reg_a , volume_a);
    _cs43l22_set(cs43l22p, reg_b , volume_b);
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   CS43L22 Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void cs43l22Init(void) {

}

/**
 * @brief   Initializes the standard part of a @p CS43L22Driver structure.
 *
 * @param[out] cs43l22p     pointer to the @p CS43L22Driver object
 *
 * @init
 */
void cs43l22ObjectInit(CS43L22Driver *cs43l22p) {

  cs43l22p->state  = CS43L22_STOP;
  cs43l22p->config = NULL;
}

/**
 * @brief   Configures and activates the CS43L22 peripheral.
 *
 * @param[in] cs43l22p      pointer to the @p CS43L22Driver object
 * @param[in] config    pointer to the @p CS43L22Config object
 *
 * @api
 */
void cs43l22Start(CS43L22Driver *cs43l22p, const CS43L22Config *config) {

  osalDbgCheck((cs43l22p != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((cs43l22p->state == CS43L22_STOP) || (cs43l22p->state == CS43L22_READY),
                "invalid state");
  cs43l22p->config = config;

  osalSysUnlock();

  cs43l22p->config->reset_cb();

  // Make sure the device is powered down
  _cs43l22_set(cs43l22p, CS43L22_REG_PWR_CTL1 , CS43L22_PWR1_DOWN );

  // read some stuff
  uint8_t idRegister = 0;
  _cs43l22_get(cs43l22p, CS43L22_REG_GET_ID, &idRegister);

  // Activate headphone channels
  cs43l22PowerCtrl(cs43l22p, CS43L22_PWR_OFF, CS43L22_PWR_OFF, CS43L22_PWR_ON, CS43L22_PWR_ON);

  // Set serial clock
  _cs43l22_set(cs43l22p, CS43L22_REG_CLOCK_CTL , CS43L22_CLK_AUTO_ON
          | CS43L22_CLK_MCDIV_ON );
  // Set input data format
  _cs43l22_set(cs43l22p, CS43L22_REG_INT_CTL1 , CS43L22_IC1_SLAVE
          | CS43L22_IC1_SCPOL_OFF | CS43L22_IC1_DSP_OFF
          | CS43L22_IC1_DIF_I2S | CS43L22_IC1_AWL_16 );
  // Fire it up
  _cs43l22_set(cs43l22p, CS43L22_REG_PWR_CTL1 , CS43L22_PWR1_UP );
  // Analog soft ramp/zero cross disabled
  _cs43l22_set(cs43l22p, CS43L22_REG_AZCSR ,
          CS43L22_AZCSR_SRB_OFF | CS43L22_AZCSR_SRA_OFF
          | CS43L22_AZCSR_ZCB_OFF | CS43L22_AZCSR_ZCA_OFF );
  // Digital soft ramp disabled
  _cs43l22_set(cs43l22p, CS43L22_REG_MISC_CTL , CS43L22_MISC_DEEMPHASIS_ON );
  // Limiter: no soft ramp/zero cross, no attack level
  _cs43l22_set(cs43l22p, CS43L22_REG_LIM_CTL1 , CS43L22_LIM1_SRD_OFF
          | CS43L22_LIM1_ZCD_OFF );

  // Initial volume and tone controls
  _cs43l22_set(cs43l22p, CS43L22_REG_TONE_CTL , 0xf );
  _cs43l22_set(cs43l22p, CS43L22_REG_PCM_A , 0x00 );
  _cs43l22_set(cs43l22p, CS43L22_REG_PCM_B , 0x00 );


  cs43l22MasterVolume(cs43l22p, 200, 200);


  osalSysLock();
  cs43l22p->state = CS43L22_READY;
  osalSysUnlock();
}

/**
 * @brief   Deactivates the CS43L22 peripheral.
 *
 * @param[in] cs43l22p      pointer to the @p CS43L22Driver object
 *
 * @api
 */
void cs43l22Stop(CS43L22Driver *cs43l22p) {

  osalDbgCheck(cs43l22p != NULL);

  osalSysLock();
  osalDbgAssert((cs43l22p->state == CS43L22_STOP) || (cs43l22p->state == CS43L22_READY),
                "invalid state");

  cs43l22p->state = CS43L22_STOP;
  osalSysUnlock();
}

void cs43l22ConfigureAudio(CS43L22Driver *cs43l22p, uint16_t samplerate, uint8_t bitsPerSample)
{
	cs43l22p->config->reconfigure_cb(samplerate, bitsPerSample);
}

void cs43l22StartTransfer(CS43L22Driver *cs43l22p)
{
	i2sStartExchange(cs43l22p->config->audiop);
}

void cs43l22StopTransfer(CS43L22Driver *cs43l22p)
{
	i2sStopExchange(cs43l22p->config->audiop);
}

void cs43l22Beep(CS43L22Driver *cs43l22p)
{
    if (cs43l22p->state == CS43L22_READY)
    {
        _cs43l22_set(cs43l22p, CS43L22_REG_BEEP_TONE_CFG , CS43L22_B3_BEEP_CFG_OFF);
        _cs43l22_set(cs43l22p, CS43L22_REG_BEEP_TONE_CFG , CS43L22_B3_BEEP_CFG_SINGLE);
    }
}

void cs43l22PowerCtrl(CS43L22Driver *cs43l22p, uint8_t spk_a, uint8_t spk_b, uint8_t hd_a, uint8_t hd_b)
{
	_cs43l22_set(cs43l22p, CS43L22_REG_PWR_CTL2 ,
			spk_a << CS43L22_PWR2_SPKA_OFFSET |
			spk_b << CS43L22_PWR2_SPKA_OFFSET |
			hd_a << CS43L22_PWR2_HDA_OFFSET |
			hd_b << CS43L22_PWR2_HDB_OFFSET);
}

void cs43l22MasterVolume(CS43L22Driver *cs43l22p, uint8_t a, uint8_t b)
{
	_cs43l22_set_volume(cs43l22p, CS43L22_REG_MASTER_VOLUME_A, a, CS43L22_REG_MASTER_VOLUME_B, b);
}
void cs43l22HeadphoneVolume(CS43L22Driver *cs43l22p, uint8_t a, uint8_t b)
{
	_cs43l22_set_volume(cs43l22p, CS43L22_REG_HP_VOLUME_A, a, CS43L22_REG_HP_VOLUME_B, b);
}
void cs43l22SpeakerVolume(CS43L22Driver *cs43l22p, uint8_t a, uint8_t b)
{
	_cs43l22_set_volume(cs43l22p, CS43L22_REG_SPK_VOLUME_A, a, CS43L22_REG_SPK_VOLUME_B, b);
}

#endif /* HAL_USE_CS43L22 */

/** @} */
