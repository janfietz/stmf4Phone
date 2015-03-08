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

	// configure pwm timers -
	// timer 2 as master, active for data transmission and inactive to disable transmission during reset period (50uS)
	// timer 3 as slave, during active time creates a 1.25 uS signal, with duty cycle controlled by frame buffer values

	ws2811p->framebuffer = chHeapAlloc(NULL, ((ws2811p->config->ledCount) * 24)+10);

	int j;
	for (j = 0; j < (ws2811p->config->ledCount) * 24; j++)
	ws2811p->framebuffer[j] = 0;

	ws2811p->dma_source = ws2811p->config->mask;

	// DMA stream 2, triggered by channel3 pwm signal. if FB indicates, reset output value early to indicate "0" bit to ws2812
	dmaStreamAllocate(ws2811p->config->dmaStreamZero, 10, NULL, NULL);
	dmaStreamSetPeripheral(ws2811p->config->dmaStreamZero, ws2811p->config->dmaStreamZero_periphal);
	dmaStreamSetMemory0(ws2811p->config->dmaStreamZero, ws2811p->framebuffer);
	dmaStreamSetTransactionSize(ws2811p->config->dmaStreamZero, (ws2811p->config->ledCount) * 24);
	dmaStreamSetMode(
			ws2811p->config->dmaStreamZero,
			STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_MINC | STM32_DMA_CR_PSIZE_BYTE
			| STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(2));
	// DMA stream 3, triggered by pwm update event. output high at beginning of signal
	dmaStreamAllocate(ws2811p->config->dmaStreamReset, 10, NULL, NULL);
	dmaStreamSetPeripheral(ws2811p->config->dmaStreamReset, ws2811p->config->dmaStreamReset_periphal);
	dmaStreamSetMemory0(ws2811p->config->dmaStreamReset, &ws2811p->dma_source);
	dmaStreamSetTransactionSize(ws2811p->config->dmaStreamReset, 1);
	dmaStreamSetMode(
			ws2811p->config->dmaStreamReset, STM32_DMA_CR_TEIE |
			STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE
			| STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));
  
	// DMA stream 6, triggered by channel1 update event. reset output value late to indicate "1" bit to ws2812.
	// always triggers but no affect if dma stream 2 already change output value to 0
	dmaStreamAllocate(ws2811p->config->dmaStreamOne, 10, NULL, NULL);
	dmaStreamSetPeripheral(ws2811p->config->dmaStreamOne, ws2811p->config->dmaStreamOne_periphal);
	dmaStreamSetMemory0(ws2811p->config->dmaStreamOne, &ws2811p->dma_source);
	dmaStreamSetTransactionSize(ws2811p->config->dmaStreamOne, 1);
	dmaStreamSetMode(
			ws2811p->config->dmaStreamOne,
			STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE
			| STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));

	pwmStart(ws2811p->config->pwmMaster, &ws2811p->config->pwmMasterConfig);
	pwmStart(ws2811p->config->pwmSlave, &ws2811p->config->pwmSlaveConfig);
	// set pwm3 as slave, triggerd by pwm2 oc1 event. disables pwmd2 for synchronization.
	ws2811p->config->pwmSlave->tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_TS_0;
	ws2811p->config->pwmMaster->tim->CR1 &= ~TIM_CR1_CEN;
	// set pwm values.
	// 51 (duty in ticks) / 105 (period in ticks) * 1.25uS (period in S) = 0.39 uS
	pwmEnableChannel(ws2811p->config->pwmSlave, 2, 51);
	// 105 (duty in ticks) / 105 (period in ticks) * 1.25uS (period in S) = 0.806 uS
	pwmEnableChannel(ws2811p->config->pwmSlave, 0, 105);
	// active during transfer of 105 cycles * ws2811p->config->ledCount * 24 bytes * 1/105 multiplier
	pwmEnableChannel(ws2811p->config->pwmMaster, 0, 105 * ws2811p->config->ledCount * 24 / 105);
	// stop and reset counters for synchronization
	ws2811p->config->pwmMaster->tim->CNT = 0;
	// Slave (TIM3) needs to "update" immediately after master (TIM2) start in order to start in sync.
	// this initial sync is crucial for the stability of the run
	ws2811p->config->pwmSlave->tim->CNT = 104;
	ws2811p->config->pwmSlave->tim->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC1DE | TIM_DIER_UDE;
	dmaStreamEnable(ws2811p->config->dmaStreamReset);
	dmaStreamEnable(ws2811p->config->dmaStreamOne);
	dmaStreamEnable(ws2811p->config->dmaStreamZero);
	// all systems go! both timers and all channels are configured to resonate
	// in complete sync without any need for CPU cycles (only DMA and timers)
	// start pwm2 for system to start resonating
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

void ws2811SetColorRGB(ws2811Driver *ws2811p, int ledNum, struct Color color)
{
    osalDbgCheck(ws2811p != NULL);
    osalSysLock();
    osalDbgAssert((ws2811p->state == WS2811_ACTIVE), "invalid state");
    osalSysUnlock();

    osalDbgCheck(ledNum < ws2811p->config->ledCount);

    uint8_t *ledBuffer = ws2811p->framebuffer + 24 * ledNum;
    int i;
    for (i=0;i<8;i++)
    {
        ledBuffer = ((color.G << i) &0b10000000 ? 0x0:ws2811p->config->mask);
        ledBuffer[8] = ((color.R << i) &0b10000000 ? 0x0:ws2811p->config->mask);
        ledBuffer[16] = ((color.B << i) &0b10000000 ? 0x0:ws2811p->config->mask);
        ledBuffer++;
    }

}
#endif /* HAL_USE_WS2811 */

/** @} */
