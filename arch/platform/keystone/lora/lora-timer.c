/*
 * Copyright (c) 2018, This. Is. IoT. - https://thisisiot.io
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of the copyright holders nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
 *
 * The stack uses timers in units of milliseconds so ensure to convert
 * to/from ticks using CLOCK_SECOND where necessary.
 *
 */
#include "contiki.h"
#include "lora-timer.h"
#include "lora-utilities.h"

void TimerInit(TimerEvent_t *obj, void(*callback)(void))
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
}

typedef void(*NoArgCallback)(void);

/* Need a timer callback wrapper to adapt the no-arg LoRaMac callbacks to the Contiki ctimer Void* arg callback. */
static void callbackAdapter(void* arg)
{
    /* The argument is the LoRaMac no-arg callback function to call. */
    ((NoArgCallback)arg)();
}

void TimerStart(TimerEvent_t *obj)
{
    /* The stack uses timers like this:
     *  1. TimerInit
     *  2. TimerSetValue
     *  3. TimerStart
     */

    /* We will use ctimer_set to start the timer with the values stored in SetValue */
    ctimer_set(&obj->c, obj->Timestamp, callbackAdapter, obj->Callback);
    obj->IsRunning = true;
}

void TimerStop(TimerEvent_t *obj)
{
    ctimer_stop(&obj->c);
    obj->IsRunning = false;
}

void TimerReset(TimerEvent_t *obj)
{
    /* From orginal timer.c: */
    TimerStop(obj);
    TimerStart(obj);
}

void TimerSetValue(TimerEvent_t *obj, uint32_t value_ms )
{
    TimerStop(obj);

    /* LoRa stack works with timer in terms of milliseconds so conversion
    * from ms -> ticks is necessary here
    */
    uint32_t ticks = MS2TICKS(value_ms);
    obj->Timestamp = ticks;
    obj->ReloadValue = ticks;
}


TimerTime_t TimerGetCurrentTime(void)
{
    uint32_t ticks = clock_time();
    return TICKS2MS(ticks);
}


TimerTime_t TimerGetElapsedTime(TimerTime_t past_ms)
{
    uint32_t ticks = clock_time();
    uint32_t past_ticks = MS2TICKS(past_ms);
    return TICKS2MS(ticks - past_ticks);
}



