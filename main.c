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

#include "float.h"
#include "math.h"

#define I2S_BUF_SIZE            256
#define CS43L22_ADDR 0x4a
#define SAMPLERATE 44100
#define BITS 16

#define STM32F4_I2S_CFG_MODE_I2S ((uint16_t)0x0800)
/* I2S configuration - Peripheral enabled */
#define STM32F4_I2S_CFG_ENABLED ((uint16_t)0x0400)
/* I2S configuration - Peripheral disabled */
#define STM32F4_I2S_CFG_DISABLED ((uint16_t)0x0000)
/* I2S configuration - I2S configuration mode - Master/transmit */
#define STM32F4_I2S_CFG_CFG_MS_TX ((uint16_t)0x0200)
/* I2S configuration - I2S configuration mode - Slave/transmit */
#define STM32F4_I2S_CFG_CFG_SL_TX ((uint16_t)0x0000)
/* I2S configuration - I2S configuration mode - Slave/receive */
#define STM32F4_I2S_CFG_CFG_SL_RX ((uint16_t)0x0100)
/* I2S configuration - I2S configuration mode - Master/receive */
#define STM32F4_I2S_CFG_CFG_MS_RX ((uint16_t)0x0300)
/* I2S configuration - PCM frame sync - Long frame */
#define STM32F4_I2S_CFG_PCMSYNC_LONG ((uint16_t)0x0080)
/* I2S configuration - PCM frame sync - Short frame */
#define STM32F4_I2S_CFG_PCMSYNC_SHORT ((uint16_t)0x0000)
/* I2S configuration - I2S standard - Right justified */
#define STM32F4_I2S_CFG_STD_LSB ((uint16_t)0x0020)
/* I2S configuration - I2S standard - I2S Philips standard */
#define STM32F4_I2S_CFG_STD_I2S ((uint16_t)0x0000)
/* I2S configuration - I2S standard - Left justified */
#define STM32F4_I2S_CFG_STD_MSB ((uint16_t)0x0010)
/* I2S configuration - I2S standard - PCM standard */
#define STM32F4_I2S_CFG_STD_PCM ((uint16_t)0x0030)
/* I2S configuration - Reverse clock polarity - ON */
#define STM32F4_I2S_CFG_RCPOL_ON ((uint16_t)0x0008)
/* I2S configuration - Reverse clock polarity - OFF */
#define STM32F4_I2S_CFG_RCPOL_OFF ((uint16_t)0x0000)
/* I2S configuration - Data length - 32-bit */
#define STM32F4_I2S_CFG_DLEN_32 ((uint16_t)0x0004)
/* I2S configuration - Data length - 16-bit */
#define STM32F4_I2S_CFG_DLEN_16 ((uint16_t)0x0000)
/* I2S configuration - Data length - 24-bit */
#define STM32F4_I2S_CFG_DLEN_24 ((uint16_t)0x0002)
/* I2S configuration - Channel length - 32-bit */
#define STM32F4_I2S_CFG_CLEN_32 ((uint16_t)0x0001)
/* I2S configuration - Channel length - 16-bit */
#define STM32F4_I2S_CFG_CLEN_16 ((uint16_t)0x0000)

/* I2S prescaler */
#define STM32F4_I2S_OFFS_PR 0x00000020
/* I2S prescaler - Master clock output - ON */
#define STM32F4_I2S_PR_MCLK_ON ((uint16_t)0x0200)
/* I2S prescaler - Master clock output - OFF */
#define STM32F4_I2S_PR_MCLK_OFF ((uint16_t)0x0000)
/* I2S prescaler - Odd factor - Divider is I2SDIV * 2 */
#define STM32F4_I2S_PR_EVEN ((uint16_t)0x0000)
/* I2S prescaler - Odd factor - Divider is 1 + I2SDIV * 2 */
#define STM32F4_I2S_PR_ODD ((uint16_t)0x0100)
/* I2S prescaler - Divider - Mask */
#define STM32F4_I2S_PR_DIV_MASK ((uint16_t)0x00ff)

static int16_t i2s_tx_buf[I2S_BUF_SIZE];

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n);

static CS43L22Driver cs43l22;

/* I2C interface #2 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};


static I2SConfig i2scfg = {
        i2s_tx_buf,
  NULL,
  I2S_BUF_SIZE,
  i2scallback,
  SPI_I2SCFGR_I2SCFG_1 | SPI_I2SCFGR_I2SE,
  STM32F4_I2S_PR_MCLK_ON
};
#define TWO_PI (M_PI * 2)
static float currentPhase = 0.0f;
static float amplitude = 16000.0f;
static void sine(uint16_t frequency, uint16_t sampleRate, size_t samples, int16_t* buffer)
{
    float phaseInc = (TWO_PI) / sampleRate * frequency;
    size_t i;
    for (i = 0; i < samples / 2; i++)
    {
        buffer[i] = (int16_t)((amplitude * sinf(currentPhase)) + 32000.0f);
        buffer[i+1] = buffer[i];
        currentPhase += phaseInc;
        if (currentPhase > TWO_PI)
        {
            currentPhase = currentPhase - TWO_PI;
        }
    }




}

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n) {

  (void)n;
  /* catch half and full tx events*/
  sine(1000, SAMPLERATE, n, i2s_tx_buf + offset);
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
    /*
    * Starting and configuring the I2S driver 3.
    */
    palSetPadMode(GPIOD, 4, PAL_MODE_OUTPUT_PUSHPULL); //RESET


    //calc frequency
    uint16_t prescale;
    uint32_t pllfreq = STM32_PLLI2SVCO / STM32_PLLI2SR_VALUE;
    if (BITS!=16)
        return;

    // Master clock mode Fs * 256
    prescale=(pllfreq*10)/(256*SAMPLERATE) + 5;
    prescale/=10;
    if (prescale > 0xFF || prescale < 2)
        prescale=2;
    i2scfg.i2spr = SPI_I2SPR_MCKOE | (prescale>>1);
    if (prescale & 0x01)
    {
        i2scfg.i2spr |= SPI_I2SPR_ODD;
    }
//    // I2S WS/MCK/SCK/SD pins
    i2sStart(&I2SD3, &i2scfg);

    palSetPadMode(GPIOA , GPIOA_LRCK, PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST | PAL_MODE_ALTERNATE(6));
    palSetPadMode(GPIOC , GPIOC_MCLK, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOC , GPIOC_SCLK, PAL_MODE_ALTERNATE(6));
    palSetPadMode(GPIOC , GPIOC_SDIN, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);

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
