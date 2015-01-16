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

#define I2S_BUF_SIZE            512
#define CS43L22_ADDR 0x4a
#define SAMPLERATE 44100
#define BITS 16

static int16_t i2s_tx_buf[I2S_BUF_SIZE];
static int16_t sineTable[SAMPLERATE];
static uint16_t frequency = 261;

static size_t txBufferWriteOffset = 0;
static size_t txBufferWriteN = 0;
SEMAPHORE_DECL(txBufferWriteSem, 1);

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n);

static CS43L22Driver cs43l22;

/* I2C interface #2 */
static const I2CConfig i2cfg1 =
{
        OPMODE_I2C,
        400000,
        FAST_DUTY_CYCLE_2,
};

static I2SConfig i2scfg =
{
        i2s_tx_buf,
        NULL,
        I2S_BUF_SIZE,
        i2scallback,
        0,
        0
};

#define TWO_PI (M_PI * 2)
static float currentAmplitude = 1.0f;
static uint16_t currentPhase = 0;
static uint16_t counter = 0;

static void precalcSine(size_t samples, int16_t* buffer)
{
    float phaseInc = (TWO_PI) / samples;
    int32_t offset = ((uint16_t) - 1) / 2;
    size_t i;
    float phase = 0.0f;
    for (i = 0; i < samples; i++)
    {
        buffer[i] = (int16_t)((offset * sinf(phase)));
        phase += phaseInc;
    }
}

static void sine(uint16_t frequency, uint16_t sampleRate, size_t samples,
        int16_t* buffer)
{
    uint16_t phaseInc = frequency;
    size_t i = 0;
    while (i < samples)
    {
        int16_t value = (int16_t)(
                (float) sineTable[currentPhase] * currentAmplitude);
        buffer[i] = value;
        buffer[++i] = value;
        ++i;
        currentPhase += phaseInc;
        if (currentPhase > sampleRate)
        {
            currentPhase = currentPhase - sampleRate;
        }
    }
}

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n)
{
    (void) i2sp;
    chSysLockFromISR();
    txBufferWriteOffset = offset;
    txBufferWriteN = n;
    chSemSignalI(&txBufferWriteSem);
    chSysUnlockFromISR();
}

static CS43L22Driver cs43l22;
static const CS43L22Config ics43l22cfg =
{
CS43L22_ADDR, GPIOD, 4, &I2CD1, &I2SD3, };

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
    (void) arg;
    chRegSetThreadName("melody");
    while (TRUE)
    {
        frequency = 261;
        palSetPad(GPIOD, GPIOD_LED3); /* Orange.  */
        chThdSleepMilliseconds(500);
        frequency = 293;
        palClearPad(GPIOD, GPIOD_LED3); /* Orange.  */
        chThdSleepMilliseconds(500);
        frequency = 329;
        chThdSleepMilliseconds(500);
        frequency = 349;
        chThdSleepMilliseconds(500);
        frequency = 391;
        chThdSleepMilliseconds(500);
        //cs43l22Beep(&cs43l22);
    }
}

/*
 * TX buffer fill thread
 */
static THD_WORKING_AREA(waThreadPlayer, 128);
static THD_FUNCTION(ThreadPlayer, arg)
{

    (void) arg;
    chRegSetThreadName("player");
    while (TRUE)
    {
        chSemWait(&txBufferWriteSem);
        if (txBufferWriteOffset == 0)
        {
            palClearPad(GPIOD, GPIOD_LED4);
        }
        else
        {
            palSetPad(GPIOD, GPIOD_LED4);
        }
        sine(frequency, SAMPLERATE, txBufferWriteN,
                i2s_tx_buf + txBufferWriteOffset);
    }
}

/*
 * Application entry point.
 */
int main(void)
{

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    precalcSine(SAMPLERATE, sineTable);
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
    palSetPadMode(GPIOD, 4, PAL_MODE_OUTPUT_PUSHPULL); //RESET

    /*
     * Starting and configuring the I2S driver 3.
     */

    //calc frequency
    uint16_t prescale;
    uint32_t pllfreq = STM32_PLLI2SVCO / STM32_PLLI2SR_VALUE;
    if (BITS != 16)
        return;

    // Master clock mode Fs * 256
    prescale = (pllfreq * 10) / (256 * SAMPLERATE) + 5;
    prescale /= 10;
    if (prescale > 0xFF || prescale < 2)
        prescale = 2;
    i2scfg.i2spr = SPI_I2SPR_MCKOE | (prescale >> 1);
    if (prescale & 0x01)
    {
        i2scfg.i2spr |= SPI_I2SPR_ODD;
    }

//    // I2S WS/MCK/SCK/SD pins
    i2sStart(&I2SD3, &i2scfg);

    palSetPadMode(GPIOA , GPIOA_LRCK, PAL_STM32_OSPEED_HIGHEST | PAL_MODE_ALTERNATE(6));
    palSetPadMode(GPIOC , GPIOC_MCLK, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOC , GPIOC_SCLK, PAL_MODE_ALTERNATE(6));
    palSetPadMode(GPIOC , GPIOC_SDIN, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);

    cs43l22Init();
    cs43l22ObjectInit(&cs43l22);

    cs43l22Start(&cs43l22, &ics43l22cfg);

    /*prefill buffer*/
    sine(frequency, SAMPLERATE, I2S_BUF_SIZE, i2s_tx_buf);

    i2sStartExchange(&I2SD3);
    /*
     * Creates the example thread.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), LOWPRIO, Thread1, NULL);
    chThdCreateStatic(waThreadPlayer, sizeof(waThreadPlayer), NORMALPRIO,
            ThreadPlayer, NULL);

    /*
     * Normal main() thread activity, in this demo it just performs
     * a shell respawn upon its termination.
     */
    while (TRUE)
    {
        if (palReadPad(GPIOA, GPIOA_BUTTON))
        {
            frequency += 50;
            if (frequency > 10000)
            {
                frequency = 100;
            }
        }
        chThdSleepMilliseconds(500);
    }
}
