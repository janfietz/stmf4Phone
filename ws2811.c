/**
 * @file    ws2811.c
 * @brief   WS2811 Driver code.
 *
 * @addtogroup WS2811
 * @{
 */

#include "hal.h"
#include "ws2811.h"

#if HAL_USE_WS2811 || defined(__DOXYGEN__)

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
/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] i2sp      pointer to the @p ws2811Driver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void ws2811_update_interrupt(ws2811Driver *ws2811p, uint32_t flags)
{
    if ((flags & STM32_DMA_ISR_TCIF) != 0) {
        /* Transfer complete processing.*/
        osalSysLockFromISR();

        ws2811p->config->pwmd->tim->CR1 &= ~TIM_CR1_CEN;
        ws2811p->state = WS2811_READY;

        osalSysUnlockFromISR();
    }
}
/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   WS2811 Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void ws2811Init(void) {

}

/**
 * @brief   Initializes the standard part of a @p WS2811Driver structure.
 *
 * @param[out] ws2811p     pointer to the @p WS2811Driver object
 *
 * @init
 */
void ws2811ObjectInit(ws2811Driver *ws2811p) {

	ws2811p->state = WS2811_STOP;
	ws2811p->config = NULL;
}

/**
 * @brief   Configures and activates the WS2811 peripheral.
 *
 * @param[in] ws2811p      pointer to the @p WS2811Driver object
 * @param[in] config    pointer to the @p WS2811Config object
 *
 * @api
 */

void ws2811Start(ws2811Driver *ws2811p, const ws2811Config *config) {

	osalDbgCheck((ws2811p != NULL) && (config != NULL));

	osalSysLock();
	osalDbgAssert((ws2811p->state == WS2811_STOP) || (ws2811p->state == WS2811_READY),
			"invalid state");
	ws2811p->config = config;

	osalSysUnlock();

	ws2811p->framebuffer = chHeapAlloc(NULL, ((ws2811p->config->ledCount) * 24)+10);
	ws2811p->dma_source  = chHeapAlloc(NULL, ((ws2811p->config->ledCount) * 24)+10);
	int j;
	for (j = 0; j < (ws2811p->config->ledCount) * 24; j++)
	{
	    ws2811p->framebuffer[j] = 0;
	    ws2811p->dma_source[j] = config->portmask;
	}

   dmaStreamAllocate(ws2811p->config->dmastp_reset, WS2811_IRQ_PRIORITY  + 1,
           (stm32_dmaisr_t)ws2811_update_interrupt,
          (void*)ws2811p);
   dmaStreamSetPeripheral(ws2811p->config->dmastp_reset, &(GPIOA->BSRR.H.set));
   dmaStreamSetMemory0(ws2811p->config->dmastp_reset, ws2811p->dma_source);
   dmaStreamSetTransactionSize(ws2811p->config->dmastp_reset, ((ws2811p->config->ledCount) * 24)+1);
   dmaStreamSetMode(ws2811p->config->dmastp_reset,
           STM32_DMA_CR_TEIE |
           STM32_DMA_CR_TCIE |
           STM32_DMA_CR_DIR_M2P |
           STM32_DMA_CR_PSIZE_BYTE |
           STM32_DMA_CR_MSIZE_BYTE |
           STM32_DMA_CR_PL(3) |
           STM32_DMA_CR_CHSEL(6));

    dmaStreamAllocate(ws2811p->config->dmastp_zero, WS2811_IRQ_PRIORITY,
           NULL,
           NULL);
    dmaStreamSetPeripheral(ws2811p->config->dmastp_zero, &(GPIOA->BSRR.H.clear));
    dmaStreamSetMemory0(ws2811p->config->dmastp_zero, ws2811p->framebuffer);
    dmaStreamSetTransactionSize(ws2811p->config->dmastp_zero, ((ws2811p->config->ledCount) * 24)+1);
    dmaStreamSetMode(ws2811p->config->dmastp_zero,
            STM32_DMA_CR_TCIE |
             STM32_DMA_CR_DIR_M2P |
             STM32_DMA_CR_MINC |
             STM32_DMA_CR_PSIZE_BYTE |
             STM32_DMA_CR_MSIZE_BYTE |
             STM32_DMA_CR_CIRC |
             STM32_DMA_CR_PL(2) |
             STM32_DMA_CR_CHSEL(6));

   dmaStreamAllocate(ws2811p->config->dmastp_one, WS2811_IRQ_PRIORITY,
           NULL,
           NULL);
   dmaStreamSetPeripheral(ws2811p->config->dmastp_one, &(GPIOA->BSRR.H.clear));
   dmaStreamSetMemory0(ws2811p->config->dmastp_one, ws2811p->dma_source);
   dmaStreamSetTransactionSize(ws2811p->config->dmastp_one, ((ws2811p->config->ledCount) * 24)+1);
   dmaStreamSetMode(ws2811p->config->dmastp_one,
           STM32_DMA_CR_TEIE |
           STM32_DMA_CR_TCIE |
           STM32_DMA_CR_DIR_M2P |
           STM32_DMA_CR_PSIZE_BYTE |
           STM32_DMA_CR_MSIZE_BYTE |
           STM32_DMA_CR_CIRC |
           STM32_DMA_CR_PL(3) |
           STM32_DMA_CR_CHSEL(6));

    pwmStart(ws2811p->config->pwmd, &ws2811p->config->pwmConfig);

    // disable timer
    ws2811p->config->pwmd->tim->CR1 &= ~TIM_CR1_CEN;

	// set pwm values.
	pwmEnableChannel(ws2811p->config->pwmd, 0, 100);
	pwmEnableChannel(ws2811p->config->pwmd, 2, 41);

    ws2811p->config->pwmd->tim->CNT = 209;

    dmaStreamEnable(ws2811p->config->dmastp_one);
    dmaStreamEnable(ws2811p->config->dmastp_zero);

	osalSysLock();
	ws2811p->state = WS2811_READY;
	osalSysUnlock();
}

/**
 * @brief   Deactivates the WS2811 peripheral.
 *
 * @param[in] ws2811p      pointer to the @p WS2811Driver object
 *
 * @api
 */
void ws2811Stop(ws2811Driver *ws2811p) {

	osalDbgCheck(ws2811p != NULL);

	osalSysLock();
	osalDbgAssert((ws2811p->state == WS2811_STOP) || (ws2811p->state == WS2811_READY),
			"invalid state");

	ws2811p->state = WS2811_STOP;
	osalSysUnlock();
}

void ws2811SetColor(ws2811Driver *ws2811p, int ledNum, struct Color *color)
{
    osalDbgCheck(ws2811p != NULL);
    osalSysLock();
    osalDbgAssert((ws2811p->state == WS2811_READY) || (ws2811p->state == WS2811_ACTIVE), "invalid state");
    osalSysUnlock();

    osalDbgCheck(ledNum < ws2811p->config->ledCount);

    uint8_t *ledBuffer = ws2811p->framebuffer + (24 * ledNum);
    int i;
    for (i=0; i<8; i++)
    {
        ledBuffer[0] = ((color->R << i) & 0b10000000 ? 0 : ws2811p->config->portmask);
        ledBuffer[8] = ((color->G << i) & 0b10000000 ?  0 : ws2811p->config->portmask);
        ledBuffer[16] = ((color->B << i) & 0b10000000 ?  0 : ws2811p->config->portmask);
        ledBuffer++;
    }
}

void ws2811Update(ws2811Driver *ws2811p)
{
    osalDbgCheck(ws2811p != NULL);
    osalSysLock();
    if (ws2811p->state != WS2811_READY)
    {
        osalSysUnlock();
        return;
    }

    osalDbgAssert((ws2811p->state == WS2811_READY), "invalid state");
    /* start timer */
    ws2811p->state = WS2811_ACTIVE;
    dmaStreamEnable(ws2811p->config->dmastp_reset);

    /* enable timer */
    ws2811p->config->pwmd->tim->CR1 |= TIM_CR1_CEN;
    osalSysUnlock();

}

#endif /* HAL_USE_WS2811 */

/** @} */
