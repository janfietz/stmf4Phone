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
            cs43l22p->config->i2cp,
            cs43l22p->config->address,
            txBuffer , 2,
            NULL , 0,
            CS43L22_I2C_TIMEOUT);
    return rv;
}

static msg_t _cs43l22_get(CS43L22Driver *cs43l22p, uint8_t reg , uint8_t* value)
{
    msg_t rv = i2cMasterTransmitTimeout(
            cs43l22p->config->i2cp,
            cs43l22p->config->address,
            &reg , 1,
            value , 1,
            CS43L22_I2C_TIMEOUT);
    return rv;
}

/* Reset the CS43L22 */
static void _cs43l22_reset_output(CS43L22Driver *cs43l22p)
{
    palClearPad(cs43l22p->config->reset_port , cs43l22p->config->reset_pad);
    chThdSleep(MS2ST(CS43L22_RESET_DELAY));
    palSetPad(cs43l22p->config->reset_port , cs43l22p->config->reset_pad);
    chThdSleep(MS2ST(CS43L22_RESET_DELAY));
}


void _cs43l22_set_volume(CS43L22Driver *cs43l22p, uint8_t volume )
{
    if ( volume > 0xe6 ) {
        volume -= 0xe7;
    } else {
        volume += 0x19;
    }
    _cs43l22_set(cs43l22p, CS43L22_REG_MASTER_VOLUME_A , volume);
    _cs43l22_set(cs43l22p, CS43L22_REG_MASTER_VOLUME_B , volume);
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

  _cs43l22_reset_output(cs43l22p);

  // Make sure the device is powered down
  _cs43l22_set(cs43l22p, CS43L22_REG_PWR_CTL1 , CS43L22_PWR1_DOWN );

  // read some stuff
  uint8_t idRegister = 0;
  _cs43l22_get(cs43l22p, CS43L22_REG_GET_ID, &idRegister);

  // Activate headphone channels
  _cs43l22_set(cs43l22p, CS43L22_REG_PWR_CTL2 ,
          CS43L22_PWR2_SPKA_OFF | CS43L22_PWR2_SPKB_OFF
          | CS43L22_PWR2_HDA_ON | CS43L22_PWR2_HDB_ON );
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

  _cs43l22_set_volume(cs43l22p, 200);

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

void cs43l22Beep(CS43L22Driver *cs43l22p)
{
    if (cs43l22p->state == CS43L22_READY)
    {
        _cs43l22_set(cs43l22p, CS43L22_REG_BEEP_TONE_CFG , CS43L22_B3_BEEP_CFG_OFF);
        _cs43l22_set(cs43l22p, CS43L22_REG_BEEP_TONE_CFG , CS43L22_B3_BEEP_CFG_SINGLE);
    }
}

#endif /* HAL_USE_CS43L22 */

/** @} */
