
/**
 * @file    ws2811.h
 * @brief   WS2811 Driver macros and structures.
 *
 * @addtogroup WS2811
 * @{
 */

#ifndef _WS2811_H_
#define _WS2811_H_

#if HAL_USE_WS2811 || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/


struct Color {
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  WS2811_UNINIT = 0,                   /**< Not initialized.                   */
  WS2811_STOP = 1,                     /**< Stopped.                           */
  WS2811_READY = 2,                    /**< Ready.                             */
  WS2811_ACTIVE = 3,                   /**< Active.                            */
} ws2811state_t;
/**
 * @brief   Type of a structure representing an ws2811Driver driver.
 */
typedef struct ws2811Driver ws2811Driver;


/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  uint16_t ledCount;
  uint32_t mask;
  PWMConfig pwmMasterConfig;
  PWMDriver *pwmMaster;
  PWMConfig pwmSlaveConfig;
  PWMDriver *pwmSlave;
  stm32_dma_stream_t *dmaStreamZero;
  void* dmaStreamZero_periphal;
  stm32_dma_stream_t *dmaStreamOne;
  void* dmaStreamOne_periphal;
  stm32_dma_stream_t *dmaStreamReset;
  void* dmaStreamReset_periphal;
} ws2811Config;


/**
 * @brief   Structure representing an ws2811 driver.
 */
struct ws2811Driver {
  /**
   * @brief   Driver state.
   */
	ws2811state_t                state;
  /**
   * @brief   Current configuration data.
   */
  const ws2811Config           *config;
  /* End of the mandatory fields.*/

  uint8_t dma_source;
  uint8_t *framebuffer;
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
  void ws2811Init(void);
  void ws2811ObjectInit(ws2811Driver *ws2811p);
  void ws2811Start(ws2811Driver *ws2811p, const ws2811Config *config);
  void ws2811Stop(ws2811Driver *ws2811p);
  void ws2811SetColorRGB(ws2811Driver *ws2811p, int ledNum, struct Color *color);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ws2811 */

#endif /* _WS2811_H_ */

/** @} */
