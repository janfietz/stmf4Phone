/**
 * @file    cs43l22.c
 * @brief   CS43L22 Driver code.
 *
 * @addtogroup CS43L22
 * @{
 */

#include "hal.h"

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

#endif /* HAL_USE_CS43L22 */

/** @} */
