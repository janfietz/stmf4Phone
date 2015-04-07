/**
 * @file    ws2811.c
 * @brief   WS2811 Driver code.
 *
 * @addtogroup WS2811
 * @{
 */

#include "hal.h"
#include "ws2811.h"
#include <string.h>

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
static void ws2811SetColorBits(const struct Color* color, uint8_t* frame, uint8_t portmask)
{
    int bit;
    for (bit = 0; bit < 8; bit++)
    {
        frame[0] = ((color->R << bit) & 0b10000000 ? 0 : portmask);
        frame[8] = ((color->G << bit) & 0b10000000 ?  0 : portmask);
        frame[16] = ((color->B << bit) & 0b10000000 ?  0 : portmask);
        frame++;
    }
}

static void ws2811UpdateFrameBuffer(ws2811Driver *ws2811p)
{
    int ledNum;
    for (ledNum = 0; ledNum < ws2811p->config->ledCount; ledNum++)
    {
        uint8_t* frame = ws2811p->framebuffer + (24 * ledNum);
        ws2811SetColorBits(&ws2811p->colors[ledNum], frame, ws2811p->config->portmask);
    }
}

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

        if (ws2811p->updateframebuffer == true)
        {
            ws2811UpdateFrameBuffer(ws2811p);
            ws2811p->updateframebuffer = false;
            chBSemSignalI(&ws2811p->bsemUpdateFrame);
        }
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
	ws2811p->colors  = chHeapAlloc(NULL, ws2811p->config->ledCount * sizeof(struct Color));
	ws2811p->updateframebuffer = false;
	ws2811p->dma_source = config->portmask;

	chBSemObjectInit(&ws2811p->bsemUpdateFrame, TRUE);
	chBSemReset(&ws2811p->bsemUpdateFrame, TRUE);

	int j;
	for (j = 0; j < ((ws2811p->config->ledCount) * 24); j++)
	{
	    ws2811p->framebuffer[j] = 0;
	}

    dmaStreamAllocate(ws2811p->config->dmastp_reset, WS2811_IRQ_PRIORITY,
        NULL,
        NULL);
    dmaStreamSetPeripheral(ws2811p->config->dmastp_reset, ws2811p->config->periphalset);
    dmaStreamSetMemory0(ws2811p->config->dmastp_reset, &ws2811p->dma_source);
    dmaStreamSetTransactionSize(ws2811p->config->dmastp_reset, 1);
    dmaStreamSetMode(ws2811p->config->dmastp_reset,
       STM32_DMA_CR_DIR_M2P |
       STM32_DMA_CR_PSIZE_BYTE |
       STM32_DMA_CR_MSIZE_BYTE |
       STM32_DMA_CR_CIRC |
       STM32_DMA_CR_PL(3) |
       STM32_DMA_CR_CHSEL(6));

    dmaStreamAllocate(ws2811p->config->dmastp_zero, WS2811_IRQ_PRIORITY,
        (stm32_dmaisr_t)ws2811_update_interrupt,
        (void*)ws2811p);
    dmaStreamSetPeripheral(ws2811p->config->dmastp_zero, ws2811p->config->periphalclear);
    dmaStreamSetMemory0(ws2811p->config->dmastp_zero, ws2811p->framebuffer);
    dmaStreamSetTransactionSize(ws2811p->config->dmastp_zero, ((ws2811p->config->ledCount) * 24));
    dmaStreamSetMode(ws2811p->config->dmastp_zero,
         STM32_DMA_CR_TCIE |
         STM32_DMA_CR_DIR_M2P |
         STM32_DMA_CR_MINC |
         STM32_DMA_CR_PSIZE_BYTE |
         STM32_DMA_CR_MSIZE_BYTE |
         STM32_DMA_CR_CIRC |
         STM32_DMA_CR_PL(3) |
         STM32_DMA_CR_CHSEL(6));

    dmaStreamAllocate(ws2811p->config->dmastp_one, WS2811_IRQ_PRIORITY,
        NULL,
        NULL);
    dmaStreamSetPeripheral(ws2811p->config->dmastp_one, ws2811p->config->periphalclear);
    dmaStreamSetMemory0(ws2811p->config->dmastp_one, &ws2811p->dma_source);
    dmaStreamSetTransactionSize(ws2811p->config->dmastp_one, 1);
    dmaStreamSetMode(ws2811p->config->dmastp_one,
       STM32_DMA_CR_DIR_M2P |
       STM32_DMA_CR_PSIZE_BYTE |
       STM32_DMA_CR_MSIZE_BYTE |
       STM32_DMA_CR_CIRC |
       STM32_DMA_CR_PL(2) |
       STM32_DMA_CR_CHSEL(6));

    pwmStart(ws2811p->config->pwmMaster, &ws2811p->config->pwmMasterConfig);
    pwmStart(ws2811p->config->pwmd, &ws2811p->config->pwmConfig);

    // disable timer
    ws2811p->config->pwmd->tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_TS_0;
    ws2811p->config->pwmMaster->tim->CR1 &= ~TIM_CR1_CEN;

	// set pwm values.
	pwmEnableChannel(ws2811p->config->pwmd, 0, ws2811p->config->onewidth);
	pwmEnableChannel(ws2811p->config->pwmd, 2, ws2811p->config->zerowidth);

	pwmEnableChannel(ws2811p->config->pwmMaster, 0, ws2811p->config->bitwidth * (ws2811p->config->ledCount) * 24 / ws2811p->config->bitwidth);

	ws2811p->config->pwmMaster->tim->CNT = 0;
    ws2811p->config->pwmd->tim->CNT = ws2811p->config->bitwidth - 1;
    dmaStreamEnable(ws2811p->config->dmastp_reset);
    dmaStreamEnable(ws2811p->config->dmastp_one);
    dmaStreamEnable(ws2811p->config->dmastp_zero);

    ws2811p->config->pwmMaster->tim->CR1 |= TIM_CR1_CEN;

	osalSysLock();
	ws2811p->state = WS2811_ACTIVE;
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
    osalDbgCheck(ledNum < ws2811p->config->ledCount);

    osalSysLock();
    osalDbgAssert((ws2811p->state == WS2811_READY) || (ws2811p->state == WS2811_ACTIVE), "invalid state");

    if (ws2811p->updateframebuffer == true)
    {
        // wait for finishing previous update
        chBSemWaitS(&ws2811p->bsemUpdateFrame);
    }
    memcpy(&ws2811p->colors[ledNum], color, sizeof(struct Color));
    osalSysUnlock();
}

void ws2811Update(ws2811Driver *ws2811p)
{
    osalDbgCheck(ws2811p != NULL);
    osalSysLock();
    if ((ws2811p->state != WS2811_READY) && (ws2811p->state != WS2811_ACTIVE))
    {
        osalSysUnlock();
        return;
    }

    if (ws2811p->updateframebuffer == true)
    {
        // wait for finishing previous update
        chBSemWaitS(&ws2811p->bsemUpdateFrame);
    }
    ws2811p->updateframebuffer = true;

    osalSysUnlock();

}

#endif /* HAL_USE_WS2811 */

/** @} */
