/*
    ChibiOS - Copyright (C) 2006-2014 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "cs43l22.h"

#define I2S_BUF_SIZE            256
#define CS43L22_ADDR 0x4a

static i2cflags_t errors = 0;

static uint16_t i2s_rx_buf[I2S_BUF_SIZE];
static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n);

static CS43L22Driver cs43l22;

/* I2C interface #2 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};


static const I2SConfig i2scfg = {
  NULL,
  i2s_rx_buf,
  I2S_BUF_SIZE,
  i2scallback,
  0,
  16
};

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n) {

  (void)i2sp;
  (void)offset;
  (void)n;
}

static CS43L22Driver cs43l22;
static const CS43L22Config ics43l22cfg = {
    CS43L22_ADDR,
    GPIOD,
    4,
    &I2CD1,
    &I2SD3,
};
/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    chThdSleepMilliseconds(500);

    cs43l22Beep(&cs43l22);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

    /*
    * Activates the serial driver 2 using the driver default configuration.
    * PA2(TX) and PA3(RX) are routed to USART2.
    */
    sdStart(&SD2, NULL);
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));



    /*
     * Starts I2C
     */
    i2cStart(&I2CD1, &i2cfg1);
    palSetPadMode(GPIOB, 6, PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(4)); //SCL
    palSetPadMode(GPIOB, 9, PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(4)); //SDA

    /*
    * Starting and configuring the I2S driver 3.
    */
    i2sStart(&I2SD3, &i2scfg);
    palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(6)); //LRCK
    palSetPadMode(GPIOC, 7, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(6)); //MCLK
    palSetPadMode(GPIOC, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(6)); //SCLK
    palSetPadMode(GPIOC, 12, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(6)); //SDIN

    palSetPadMode(GPIOD, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_MODE_ALTERNATE(7)); //RESET


    cs43l22Init();
    cs43l22ObjectInit(&cs43l22);

    cs43l22Start(&cs43l22, &ics43l22cfg);

    i2sStartExchange(&I2SD3);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it just performs
   * a shell respawn upon its termination.
   */
  while (TRUE) {
    if (palReadPad(GPIOA, GPIOA_BUTTON))
      TestThread(&SD2);
    chThdSleepMilliseconds(500);
  }
}
