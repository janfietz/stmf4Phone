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

#include "chprintf.h"
#include "shell.h"

#include "float.h"
#include "math.h"

#include "usbcfg.h"
#include "mmc.h"


/**
 * MMC driver instance.
 */
MMCDriver MMCD1;

/* Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig hs_spicfg = {NULL, GPIOC, GPIOC_PIN4, SPI_CR1_BR_1 | SPI_CR1_BR_0};

/* Low speed SPI configuration (281.250kHz, CPHA=0, CPOL=0, MSb first).*/
static SPIConfig ls_spicfg = {NULL, GPIOC, GPIOC_PIN4,
                              SPI_CR1_BR_2 | SPI_CR1_BR_1};

/* MMC/SD over SPI driver configuration.*/
static MMCConfig mmccfg = {&SPID2, &ls_spicfg, &hs_spicfg};

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;


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

        chThdSleepMilliseconds(500);
        frequency = 293;
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

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%08lx %08lx %4lu %4lu %9s\r\n",
             (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state]);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  thread_t *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriorityX(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}


static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"test", cmd_test},
  {"mount", cmd_mount},
  {"umount", cmd_unmount},
  {"tree", cmd_tree},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

/*
 * Application entry point.
 */

int main(void)
{
    thread_t *shelltp = NULL;

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
     * Shell manager initialization.
     */
    shellInit();

    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

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
     * Start SD Driver
     */
    /*
     * Initializes the MMC driver to work with SPI2.
     */
    palSetPadMode(GPIOC, GPIOC_PIN4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, GPIOB_PIN13, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST );
    palSetPadMode(GPIOB, GPIOB_PIN14, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOB, GPIOB_PIN15, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);

    mmcObjectInit(&MMCD1);
    mmcStart(&MMCD1, &mmccfg);
    if (mmcConnect(&MMCD1) == HAL_SUCCESS)
    {
        palSetPad(GPIOD, GPIOD_LED3); /* Orange.  */
    }
    else
    {
        palClearPad(GPIOD, GPIOD_LED3); /* Orange.  */
    }

    /*
     * Normal main() thread activity, in this demo it just performs
     * a shell respawn upon its termination.
     */
    while (TRUE)
    {
//        if (palReadPad(GPIOA, GPIOA_BUTTON))
//        {
//            testLedPattern2();
//            chThdSleepMilliseconds(50);
//        }
//        else
//        {
//            testLedPattern();
//            chThdSleepMilliseconds(100);
//        }

        if (!shelltp) {
          if (SDU1.config->usbp->state == USB_ACTIVE) {
            /* Spawns a new shell.*/
            shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
          }
        }
        else {
          /* If the previous shell exited.*/
          if (chThdTerminatedX(shelltp)) {
            /* Recovers memory of the previous shell.*/
            chThdRelease(shelltp);
            shelltp = NULL;
          }
        }
        chThdSleepMilliseconds(500);
    }
}
