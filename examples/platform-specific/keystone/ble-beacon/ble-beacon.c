/*
* Copyright (c) 2019, THIS. IS. IoT. - https://thisisiot.io
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

/**
* \file
*        BLE beacon.  Activate the Keystone platform beacon daemon.
*
*        The beacon will contain the board name, e.g. "IoT.Keystone.1A".
*
* \author Evan Ross <evan@thisisiot.io>
*/

#include "contiki.h"
#include "sys/log.h"
#include "dev/leds.h"
#include "services/shell/shell.h"
#include "services/shell/shell-commands.h"
#include "services/shell/serial-shell.h"
#include "rf/ble-beacond.h"
#include "rf/ble-addr.h"

#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "ble-beacon"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
PROCESS(ble_beacon_process, "ble beacon process");
AUTOSTART_PROCESSES(&ble_beacon_process);


/*---------------------------------------------------------------------------*/

/* Defaults */

#define BLE_ADV_INTERVAL            (CLOCK_SECOND * 5)

/*---------------------------------------------------------------------------*/

static bool check_result(rf_ble_beacond_result_t ble_result, const char* action)
{
    switch (ble_result)
    {
    case RF_BLE_BEACOND_OK:
        return true;
    default:
    case RF_BLE_BEACOND_ERROR:
        LOG_ERR("%s failed.\n", action);
        return false;
    case RF_BLE_BEACOND_DISABLED:
        LOG_ERR("%s failed : BEACOND disabled.\n", action);
        return false;
    }
}
/*---------------------------------------------------------------------------*/

#if 0
static void beacon_start_cb()
{
    leds_single_on(LEDS_GREEN);
}

/*---------------------------------------------------------------------------*/

static void beacon_end_cb()
{
    leds_single_off(LEDS_GREEN);
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_beacon_process, ev, data)
{
    PROCESS_BEGIN();

    leds_single_off(LEDS_GREEN);
    leds_single_off(LEDS_RED);

    uint8_t* ble_addr = ble_addr_ptr();

    LOG_INFO("Starting ble_beacon example application.\n");
    LOG_INFO("Advertisement interval: %d s\n", BLE_ADV_INTERVAL /  CLOCK_SECOND);
    LOG_INFO("Advertising: addr: %02X:%02X:%02X:%02X:%02X:%02X name: \"" BOARD_STRING "\"\n",
        ble_addr[5], ble_addr[4], ble_addr[3], ble_addr[2], ble_addr[1], ble_addr[0]);
#if 1
    if (!check_result(rf_ble_set_tx_power(4), "set tx power")) {
        PROCESS_EXIT();
    }
#endif
    LOG_INFO("Tx power: %d\n", rf_ble_get_tx_power());

    /* Init the BLE advertisement daemon */
    if (!check_result(rf_ble_beacond_config(0, BOARD_STRING), "beacond_config")) {
        PROCESS_EXIT();
    }
#if 0
    if (!check_result(rf_ble_beacond_set_callbacks(
        beacon_start_cb, beacon_end_cb), "set callbacks")) {
        PROCESS_EXIT();
    }
#endif
    if (!check_result(rf_ble_beacond_start(), "beacond start")) {
        PROCESS_EXIT();
    }

    serial_shell_show_prompt();

    while (1) {
        
        PROCESS_YIELD();

    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
