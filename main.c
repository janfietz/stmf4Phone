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
#include "ws2811.h"

#include "float.h"
#include "math.h"

#define I2S_BUF_SIZE            512
#define CS43L22_ADDR 0x4a
#define SAMPLERATE 44100
#define BITS 16
#define LEDCOUNT 4

static int16_t i2s_tx_buf[I2S_BUF_SIZE];
static int16_t sineTable[SAMPLERATE];
static uint16_t frequency = 261;

static size_t txBufferWriteOffset = 0;
static size_t txBufferWriteN = 0;
SEMAPHORE_DECL(txBufferWriteSem, 1);

static ws2811Driver ws2811;
static ws2811Config ws2811_cfg =
{
    LEDCOUNT,
    0b00000010,
    (uint32_t)(&(GPIOA->BSRR.H.set)),
    (uint32_t)(&(GPIOA->BSRR.H.clear)),
    210,
    41,
    100,
    {
        168000000 / 2 / 210,
        (LEDCOUNT * 24) + 20,
        NULL,
        {
            { PWM_OUTPUT_ACTIVE_HIGH, NULL },
            { PWM_OUTPUT_DISABLED, NULL },
            { PWM_OUTPUT_DISABLED, NULL },
            { PWM_OUTPUT_DISABLED, NULL }
        },
        TIM_CR2_MMS_2, /* master mode selection */
        0,
    },
    &PWMD2,
    {
        168000000 / 2,
        210,
        NULL,
        {
            { PWM_OUTPUT_ACTIVE_HIGH, NULL },
            { PWM_OUTPUT_ACTIVE_HIGH, NULL },
            { PWM_OUTPUT_ACTIVE_HIGH, NULL },
            { PWM_OUTPUT_ACTIVE_HIGH, NULL }
        },
        0,
        TIM_DIER_UDE | TIM_DIER_CC3DE | TIM_DIER_CC1DE,
    },
    &PWMD1,
    STM32_DMA2_STREAM5,
    STM32_DMA2_STREAM1,
    STM32_DMA2_STREAM6,
};

struct Color colors[LEDCOUNT];
static void testLedPattern()
{
    int i;
    for (i = ws2811_cfg.ledCount - 1; i > 0; i--)
    {
        colors[i].R = colors[i-1].R;
        colors[i].G = colors[i-1].G;
        colors[i].B = colors[i-1].B;
        ws2811SetColor(&ws2811, i, &colors[i]);
    }
    colors[0].R = rand() % 256;
    colors[0].G = rand() % 256;
    colors[0].B = rand() % 256;

   ws2811SetColor(&ws2811, 0, &colors[0]);
    ws2811Update(&ws2811);
}
static int ledpos = 0;
static int traildir = 1;
struct Color trailcolors[4];
static void testLedPattern2()
{
    if (ledpos == -4)
    {
        struct Color curcolor = {rand() % 256, rand() % 256 , rand() % 256};
        /* generate new random color */
        int i;
        for (i = 0; i < 4; i++)
        {
            trailcolors[i].R = curcolor.R / (i + 1);
            trailcolors[i].G = curcolor.G / (i + 1);
            trailcolors[i].B = curcolor.B / (i + 1);
        }
    }

    /* delete last led */
    struct Color black = {0,0,0};
    int i;
    for (i = ws2811_cfg.ledCount - 1; i >= 0; i--)
    {
        ws2811SetColor(&ws2811, i, &black);
    }

    if (ledpos >= (ws2811_cfg.ledCount + 4))
    {
        traildir = traildir * -1;
        ledpos = ws2811_cfg.ledCount - 1;
    }
    else if (ledpos == -4)
    {
        traildir = traildir * -1;
        ledpos = 0;
    }
    else
    {
        ledpos = ledpos + traildir;
    }

    /* set trail */
    for (i = 0; i < 4; i++)
    {
        int pos = ledpos - (traildir * i);
        if ((pos >= 0) && (pos < (ws2811_cfg.ledCount)))
        {
            ws2811SetColor(&ws2811, pos, &trailcolors[i]);
        }
    }

    /* set end points */
    if (ledpos >= ws2811_cfg.ledCount - 1)
    {
        ws2811SetColor(&ws2811, ws2811_cfg.ledCount - 1, &trailcolors[0]);
    }
    else if (ledpos < 0)
    {
        ws2811SetColor(&ws2811, 0, &trailcolors[0]);
    }

    ws2811Update(&ws2811);
}

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

/* Reset the CS43L22 */
static void cs43l22_reset(void)
{
    palClearPad(GPIOD, GPIOD_RESET);
    chThdSleep(MS2ST(10));
    palSetPad(GPIOD, GPIOD_RESET);
    chThdSleep(MS2ST(10));
}

/* Reset the CS43L22 */
static void cs43l22_reconfigure(uint16_t samplerate, uint8_t bitsPerSample)
{
	//calc frequency
	uint16_t prescale;
	uint32_t pllfreq = STM32_PLLI2SVCO / STM32_PLLI2SR_VALUE;
	if (bitsPerSample != 16)
		return;

	// Master clock mode Fs * 256
	prescale = (pllfreq * 10) / (256 * samplerate) + 5;
	prescale /= 10;
	if (prescale > 0xFF || prescale < 2)
		prescale = 2;
	i2scfg.i2spr = SPI_I2SPR_MCKOE | (prescale >> 1);
	if (prescale & 0x01)
	{
		i2scfg.i2spr |= SPI_I2SPR_ODD;
	}

	i2sStart(&I2SD3, &i2scfg);
}

static CS43L22Driver cs43l22;
static const CS43L22Config ics43l22cfg =
{
		CS43L22_ADDR,
		&I2CD1,
		&I2SD3,
		cs43l22_reset,
		cs43l22_reconfigure,
};

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
     * Starts ws2811 driver
     */
    ws2811ObjectInit(&ws2811);
    ws2811Start(&ws2811, &ws2811_cfg);
    palSetGroupMode(GPIOA, 0b00000010, 0, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_HIGHEST|PAL_STM32_PUDR_FLOATING);

    /* set color to all leds */
    struct Color color = {255, 0, 0};
    int i = 0;
    for (i=0;i<ws2811_cfg.ledCount;i++)
    {
        colors[i].R = color.R;
        colors[i].G = color.G;
        colors[i].B = color.B;
        ws2811SetColor(&ws2811, i, &colors[i]);
    }
    ws2811Update(&ws2811);

    /*
     * Starts I2C
     */
    i2cStart(&I2CD1, &i2cfg1);

    palSetPadMode(GPIOD, GPIOD_RESET, PAL_MODE_OUTPUT_PUSHPULL); //RESET
    cs43l22Init();
    cs43l22ObjectInit(&cs43l22);

    /*
     * Starting and configuring the I2S driver 3.
     */
    palSetPadMode(GPIOA , GPIOA_LRCK, PAL_STM32_OSPEED_HIGHEST | PAL_MODE_ALTERNATE(6));
    palSetPadMode(GPIOC , GPIOC_MCLK, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOC , GPIOC_SCLK, PAL_MODE_ALTERNATE(6));
    palSetPadMode(GPIOC , GPIOC_SDIN, PAL_MODE_ALTERNATE(6) |
            PAL_STM32_OSPEED_HIGHEST);

    cs43l22Start(&cs43l22, &ics43l22cfg);
    cs43l22ConfigureAudio(&cs43l22, SAMPLERATE, BITS);


    /*prefill buffer*/
    sine(frequency, SAMPLERATE, I2S_BUF_SIZE, i2s_tx_buf);

    cs43l22StartTransfer(&cs43l22);

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
            testLedPattern2();
            chThdSleepMilliseconds(50);
        }
        else
        {
            testLedPattern();
            chThdSleepMilliseconds(100);
        }
    }
}
