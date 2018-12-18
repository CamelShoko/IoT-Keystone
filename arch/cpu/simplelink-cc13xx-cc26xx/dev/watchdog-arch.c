/*
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 /**
 * \addtogroup cc13xx-cc26xx-cpu
 * @{
 *
 * \defgroup cc13xx-cc26xx-watchdog CC13xx/CC26xx watchdog timer driver
 *
 * Driver for the CC13xx/CC26xx Watchdog Timer
 *
 * This file is not called watchdog.c because the filename is in use by
 * TI CC26xxware/CC13xxware
 * @{
 *
 *  The Watchdog counts down at a rate of the device clock SCLK_HF (48 MHz)
 *  divided by a fixed-division ratio of 32, which equals to 1.5 MHz. The
 *  Watchdog rate will change if SCLK_HF deviates from 48 MHz.
 *
 * \file
 * Implementation of the CC13xx/CC26xx watchdog driver.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/watchdog.h"
/*---------------------------------------------------------------------------*/
#include <Board.h>

#include <ti/drivers/Watchdog.h>
/*---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define WATCHDOG_DISABLE    WATCHDOG_CONF_DISABLE
#define WATCHDOG_TIMER_TOP  WATCHDOG_CONF_TIMER_TOP
/*---------------------------------------------------------------------------*/
static Watchdog_Handle wdt_handle = NULL;
/*---------------------------------------------------------------------------*/

static void init(void)
{
    Watchdog_init();

    Watchdog_Params wdt_params;
    Watchdog_Params_init(&wdt_params);

    wdt_params.resetMode = Watchdog_RESET_ON;
    wdt_params.debugStallMode = Watchdog_DEBUG_STALL_ON;

    wdt_handle = Watchdog_open(Board_WATCHDOG0, &wdt_params);
}

/**
 * \brief  Initialises the Watchdog module.
 *
 *         Simply sets the reload counter to a default value. The WDT is not
 *         started yet. To start it, watchdog_start() must be called.
 */
void
watchdog_init(void)
{
  if(WATCHDOG_DISABLE) {
    return;
  }

  init();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Start the Watchdog.
 */
void
watchdog_start(void)
{
  if(WATCHDOG_DISABLE) {
    return;
  }

  watchdog_periodic();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Refresh (feed) the Watchdog.
 */
void
watchdog_periodic(void)
{
  if(WATCHDOG_DISABLE) {
    return;
  }

  Watchdog_setReload(wdt_handle, WATCHDOG_TIMER_TOP);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Stop the Watchdog such that it won't timeout and cause a
 *         system reset.
 */
void
watchdog_stop(void)
{
  if(WATCHDOG_DISABLE) {
    return;
  }

  Watchdog_clear(wdt_handle);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Manually trigger a Watchdog timeout.
 */
void
watchdog_reboot(void)
{
  /* Allow watchdog reboot even if it is configured as DISABLED by
   * the application.
   */
  if (wdt_handle == NULL) {
    init();
  }

  /* Used a fixed timeout here, independent of the application
   * timeout setting.  This value is approx 1000000/1500000 = 0.7 seconds
   */
  Watchdog_setReload(wdt_handle, 0xFFFFF);

  /* Busy loop until watchdog times out */
  for (;;) { /* hang */ }
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
