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
*        Play with LoRa.
*
*        Command and control the SX1262 LoRa radio on the IoT.Keystone:
*        - Setup parameters such as SF, CR, BW, frequency.
*        - define payloads and transmit data on command.
*        - start listening on a channel for data.
*        - enable TX CW and premable modulation modes.
*        - channel activity detection.
*        - raw LoRa register access.
*       
*        And more, all from the console.
*       
*        Use the 'lora' command to access these features.
*              
*
* \author Evan Ross <evan@thisisiot.io>
*/

#include "contiki.h"
#include "sys/log.h"
#include "dev/leds.h"
#include "services/shell/shell.h"
#include "services/shell/shell-commands.h"
#include "services/shell/serial-shell.h"
#include "lora/sx126x.h"
#include "lora/lora-utilities.h"
#include "lora/sx126x-board.h"

#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "lora-rf"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* The processs must be called "loramac_process" as required by
 * the radio driver IRQ polling function.
 */
PROCESS(loramac_process, "lora-rf process");
AUTOSTART_PROCESSES(&loramac_process);

/*!
* Radio events function pointer
*/
static RadioEvents_t RadioEvents;

/*---------------------------------------------------------------------------*/

/* Defaults */

#define LORA_TX_POWER                               20        // dBm
#define RF_FREQUENCY                                903900000 // Hz
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define LORA_RX_CONTINUOUS                          true  // false - program timer
                                                              // into setRx command to generate RxDone INT.
#define LORA_TX_TIMEOUT                             3000  // milliseconds
#define LORA_SYNC_WORD                              0x3444 // public network  / 0x1424 private network

#define DEFAULT_CW_TIMEOUT_S                        10


/*---------------------------------------------------------------------------*/

/* Radio settings modified by console commands and used by 
 * TX and RX actions.
 */
typedef struct {
    uint32_t freq;
    uint8_t bw;
    uint8_t sf;
    uint8_t cr;
    int8_t pwr;

    /* tx */
    uint8_t payload_len;
    uint8_t payload[LORA_RADIO_MAX_PAYLOAD];
    uint32_t tx_timeout;
    uint16_t sync_word;

    uint16_t preamble_len;
    bool iq_invert;

} RadioSettings_t;

static RadioSettings_t RadioSettings;


const char* getRadioIQInvertStr()
{
    return (RadioSettings.iq_invert ? "true" : "false");
}

void setRadioSettingsDefaults()
{
    RadioSettings.freq = RF_FREQUENCY;
    RadioSettings.bw = LORA_BANDWIDTH;
    RadioSettings.sf = LORA_SPREADING_FACTOR;
    RadioSettings.cr = LORA_CODINGRATE;

#if Board_SX1262_TX_POWER_LIMIT
    if (LORA_TX_POWER > Board_SX1262_TX_POWER_LIMIT) {
        RadioSettings.pwr = Board_SX1262_TX_POWER_LIMIT;
    }
    else
#endif
    {
        RadioSettings.pwr = LORA_TX_POWER;
    }

    RadioSettings.payload_len = 0;
    memset(RadioSettings.payload, 0, LORA_RADIO_MAX_PAYLOAD);
    RadioSettings.tx_timeout = LORA_TX_TIMEOUT;

    RadioSettings.preamble_len = LORA_PREAMBLE_LENGTH;
    RadioSettings.iq_invert = LORA_IQ_INVERSION_ON;
}

void PrintRadioSettings(shell_output_func output)
{
    SHELL_OUTPUT(output, "frequency: %lu Hz\n", RadioSettings.freq);
    SHELL_OUTPUT(output, "bandwidth: %d -> %s\n", RadioSettings.bw, LoraBWStrings[RadioSettings.bw]);
    SHELL_OUTPUT(output, "spreading factor: SF%d\n", RadioSettings.sf);
    SHELL_OUTPUT(output, "coding rate: %d -> %s\n", RadioSettings.cr, LoraCRStrings[RadioSettings.cr]);
#if Board_SX1262_TX_POWER_LIMIT
    if (RadioSettings.pwr > Board_SX1262_TX_POWER_LIMIT) {
        SHELL_OUTPUT(output, "tx power (board limited): %d dBm\n", Board_SX1262_TX_POWER_LIMIT);
    }
    else
#endif
    {
        SHELL_OUTPUT(output, "tx power : %d dBm\n", RadioSettings.pwr);
    }

    SHELL_OUTPUT(output, "tx size: %d bytes\n", RadioSettings.payload_len);
    SHELL_OUTPUT(output, "tx payload: [ ");
    for (int i = 0; i < RadioSettings.payload_len; i++) {
        SHELL_OUTPUT(output, "%02x ", RadioSettings.payload[i]);
    }
    SHELL_OUTPUT(output, "]\n");
    SHELL_OUTPUT(output, "tx timeout: %lu ms\n", RadioSettings.tx_timeout);
    SHELL_OUTPUT(output, "preamble length: %d symbols\n", RadioSettings.preamble_len);
    SHELL_OUTPUT(output, "IQ invert: %s\n", getRadioIQInvertStr());
}

static bool ValidateRadioBW(uint8_t bw)
{
    return (bw <= 2);
}

static bool ValidateRadioSF(uint8_t sf)
{
    return (sf >=5 && sf <= 12);
}

static bool ValidateRadioCR(uint8_t cr)
{
    return (cr >= 1 && cr <= 4);
}

static bool ValidateRadioFreq(uint32_t freq)
{
    return (freq >= 400e6 && freq <= 1e9);
}

static bool ValidateRadioPwr(int8_t pwr)
{
    return (pwr >= -9 &&
#if Board_SX1262_TX_POWER_LIMIT
        pwr <= Board_SX1262_TX_POWER_LIMIT);
#else
        pwr <= 22);
#endif
}
/*---------------------------------------------------------------------------*/


/* List of valid SX1262 registers */

typedef struct {
    const char* name;
    uint16_t addr;
    bool writeable;
    const char* def;
    const char* desc;
} SX1262Reg_t;

#define NUM_SX1262_LORA_REGS     10

static const SX1262Reg_t sx1262_regs[NUM_SX1262_LORA_REGS] = {
    { "LoRa Sync Word MSB",     0x0740, true,   "0x14", "Differentiate the LoRa signal for Public or Private Network." },
    { "LoRa Sync Word LSB",     0x0741, true,   "0x24", "" },
    { "RandomNumberGen[0]",     0x0819, false,  "-",    "Can be used to get a 32 - bit random number" },
    { "RandomNumberGen[1]",     0x081A, false,  "-",    "Can be used to get a 32 - bit random number" },
    { "RandomNumberGen[2]",     0x081B, false,  "-",    "Can be used to get a 32 - bit random number" },
    { "RandomNumberGen[3]",     0x081C, false,  "-",    "Can be used to get a 32 - bit random number" },
    { "Rx Gain",                0x08AC, true,   "0x94", "Set the gain used in Rx mode" },
    { "OCP Configuration",      0x08E7, true,   "0x18", "Set the Over Current Protection level." },
    { "XTA trim",               0x0911, true,   "0x05", "Value of the trimming cap on XTA pin" },
    { "XTB trim",               0x0912, true,   "0x05", "Value of the trimming cap on XTB pin" },
};

void PrintRadioLoraRegs(shell_output_func output, uint8_t* data)
{
    if (data == NULL) {
        SHELL_OUTPUT(output, "name                 address writable default description\n");
        SHELL_OUTPUT(output, "-------------------- ------- -------- ------- ---------------\n");
        for (int i = 0; i < NUM_SX1262_LORA_REGS; i++) {
            SHELL_OUTPUT(output, "%20s  0x%04X %8s %7s %s\n",
                sx1262_regs[i].name,
                sx1262_regs[i].addr,
                sx1262_regs[i].writeable ? "true" : "false",
                sx1262_regs[i].def,
                sx1262_regs[i].desc);
        }
    }
    else {
        SHELL_OUTPUT(output, "name                 address value\n");
        SHELL_OUTPUT(output, "-------------------- ------- -----\n");
        for (int i = 0; i < NUM_SX1262_LORA_REGS; i++) {
            SHELL_OUTPUT(output, "%20s  0x%04X  0x%02X\n",
                sx1262_regs[i].name,
                sx1262_regs[i].addr,
                data[i]);
        }
    }
}

static bool validateLoraReg(uint32_t reg, bool checkWrite)
{
    for (int i = 0; i < NUM_SX1262_LORA_REGS; i++) {
        if (reg == sx1262_regs[i].addr) {
            if (checkWrite) {
                return sx1262_regs[i].writeable;
            } 
            else {
                return true;
            }
        }
    }
    return false;
}

/*---------------------------------------------------------------------------*/


#define GET_UINT_ARG(val, type, convert) do {                               \
    char *ptr;                                                              \
    SHELL_ARGS_NEXT(args, next_args);                                       \
    if (args) val = (type)strtol(args, &ptr, convert); /* returns 0 other wise */      \
    else val = 0;                                                            \
} while (0)


/*---------------------------------------------------------------------------*/

/* 'lora' command handler 
 *
 */
static
PT_THREAD(cmd_lora(struct pt *pt, shell_output_func output, char *args))
{

    PT_BEGIN(pt);

    char *next_args;

    SHELL_ARGS_INIT(args, next_args);


    /* rf command options: 
     *   set bw [0-2] | pwr [dBm] | sf [7-12] chan [0-72] freq [kHz] : set parameter
     *   set : lists current parameter settings
     *   rx : enter rx continuous listen mode
     *   cw [duration (s)]: enter tx cw mode for duration seconds
     *   off : puts radio into standby mode
     */
    SHELL_ARGS_NEXT(args, next_args);

    bool result = true;

    if (args != NULL) {

        if (strcmp("off", args) == 0) {
            Radio.Standby();
        }
        /* ---------------- REG <reg> <data> ------------------------------------------------*/
        else if (strcmp("reg", args) == 0) {
            SHELL_ARGS_NEXT(args, next_args);
            if (strcmp("set", args) == 0) {
                uint16_t reg;
                GET_UINT_ARG(reg, uint16_t, 16);
                if (validateLoraReg(reg, true)) {
                    uint8_t data;
                    GET_UINT_ARG(data, uint8_t, 16);
                    SX126xWriteRegisters(reg, &data, 1);
                    uint8_t tmp = 0;
                    SX126xReadRegisters(reg, &tmp, 1);
                    if (tmp != data) {
                        result = false;
                        SHELL_OUTPUT(output, "reg set failed: readback failed. got:0x%02x\n", tmp);
                    }
                }
                else {
                    SHELL_OUTPUT(output, "invalid sx126x lora register: 0x%04X\n", reg);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("get", args) == 0) {
                uint16_t reg;
                GET_UINT_ARG(reg, uint16_t, 16);
                if (reg == 0) { /* implies no argument given */
                    /* Get all the registers then print them */
                    uint8_t data[NUM_SX1262_LORA_REGS];
                    for (int i = 0; i < NUM_SX1262_LORA_REGS; i++) {
                        SX126xReadRegisters(sx1262_regs[i].addr, &data[i], 1);
                    }
                    SHELL_OUTPUT(output, "Content of lora registers:\n");
                    PrintRadioLoraRegs(output, data);
                } else if (validateLoraReg(reg, false)) {
                    uint8_t data;
                    SX126xReadRegisters(reg, &data, 1);
                    SHELL_OUTPUT(output, "reg 0x%04X get: 0x%02X\n", reg, data);
                }
                else {
                    SHELL_OUTPUT(output, "invalid sx126x lora register: 0x%04X\n", reg);
                    PT_EXIT(pt);
                }
            }
            else {
                SHELL_OUTPUT(output, "List of available lora registers:\n");
                PrintRadioLoraRegs(output, NULL);
            }
        }
        else if (strcmp("set", args) == 0) {
            SHELL_ARGS_NEXT(args, next_args);
            if (args == NULL) {
                /* list the current settings */
                PrintRadioSettings(output);
            }
            else if (strcmp("bw", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                uint8_t bw = (uint8_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (ValidateRadioBW(bw)) {
                    RadioSettings.bw = bw;
                }
                else {
                    SHELL_OUTPUT(output, "invalid bandwidth: %d\n", bw);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("pwr", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                int8_t pwr = (int8_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (ValidateRadioPwr(pwr)) {
                    RadioSettings.pwr = pwr;
                }
                else {
                    SHELL_OUTPUT(output, "invalid power: %d\n", pwr);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("sf", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                uint8_t sf = (uint8_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (ValidateRadioSF(sf)) {
                    RadioSettings.sf = sf;
                }
                else {
                    SHELL_OUTPUT(output, "invalid spreading factor: %d\n", sf);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("cr", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                uint8_t cr = (uint8_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (ValidateRadioCR(cr)) {
                    RadioSettings.cr = cr;
                }
                else {
                    SHELL_OUTPUT(output, "invalid coding rate: %d\n", cr);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("freq", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                uint32_t freq = (uint32_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                freq *= 1000;
                if (ValidateRadioFreq(freq)) {
                    RadioSettings.freq = freq;
                }
                else {
                    SHELL_OUTPUT(output, "invalid frequency: %lu\n", freq);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("timeout", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                uint32_t timeout = (uint32_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (timeout < 100000) {
                    RadioSettings.tx_timeout = timeout;
                }
                else {
                    SHELL_OUTPUT(output, "invalid timeout: %lu\n", timeout);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("tx", args) == 0) {
                char *ptr; /* dummy */
                SHELL_ARGS_NEXT(args, next_args);
                uint16_t len = (uint16_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (len <= LORA_RADIO_MAX_PAYLOAD) {
                    RadioSettings.payload_len = len;
                    for (int i = 0; i < len; i++) {
                        SHELL_ARGS_NEXT(args, next_args);
                        if (args == NULL) {
                            /* no data provided - that's ok just supply 0's */
                            RadioSettings.payload[i] = 0;
                        }
                        else {
                            /* convert as hex character. */
                            uint8_t data = (uint8_t)strtol(args, &ptr, 16); /* returns 0 other wise */
                            RadioSettings.payload[i] = data;
                        }
                    }
                }
                else {
                    SHELL_OUTPUT(output, "payload length too large : %d > %d\n", len, LORA_RADIO_MAX_PAYLOAD);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("preamble", args) == 0) {
                uint32_t val;
                GET_UINT_ARG(val, uint32_t, 10);
                if (val <= 0xFFFF) {
                    RadioSettings.preamble_len = val;
                }
                else {
                    SHELL_OUTPUT(output, "invalid preamble len: %lu (<= 0xFFFF)\n", val);
                    PT_EXIT(pt);
                }
            }
            else if (strcmp("iq", args) == 0) {
                uint32_t val;
                GET_UINT_ARG(val, uint32_t, 10);
                if (val <= 1) {
                    RadioSettings.iq_invert = (bool)val;
                }
                else {
                    SHELL_OUTPUT(output, "invalid iq invert: %lu (0 or 1) \n", val);
                    PT_EXIT(pt);
                }
            }
            else {
                SHELL_OUTPUT(output, "unsupported rf set command : %s\n", args);
                PT_EXIT(pt);
            }
        }
        else if (strcmp("cw", args) == 0) {

            uint16_t tx_timeout_s;

            /* Get duration */
            SHELL_ARGS_NEXT(args, next_args);
            if (args == NULL) {
                tx_timeout_s = DEFAULT_CW_TIMEOUT_S;
            }
            else {
                char *ptr; /* dummy */
                tx_timeout_s = (uint16_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (tx_timeout_s < 1) {
                    SHELL_OUTPUT(output, "error: duration %d too small\n",
                        tx_timeout_s);
                    PT_EXIT(pt);
                }
            }

            /* Execute command */
            SHELL_OUTPUT(output, "Running CW TX on freq %lu kHz at power %d dBm for %d seconds...\n",
                RadioSettings.freq, RadioSettings.pwr, tx_timeout_s);

            leds_single_on(LEDS_GREEN);
            /* Radio stays in this mode until further notice.  The timeout just sets a lora-timer (etimer)
             * instance which triggers the OnRadioTxTimeout() callback
             */
            Radio.SetTxContinuousWave(RadioSettings.freq, RadioSettings.pwr, tx_timeout_s);

        }
        else if (strcmp("cwmod", args) == 0) {

            uint16_t tx_timeout_s;

            /* Get duration */
            SHELL_ARGS_NEXT(args, next_args);
            if (args == NULL) {
                tx_timeout_s = DEFAULT_CW_TIMEOUT_S;
            }
            else {
                char *ptr; /* dummy */
                tx_timeout_s = (uint16_t)strtol(args, &ptr, 10); /* returns 0 other wise */
                if (tx_timeout_s < 1) {
                    SHELL_OUTPUT(output, "error: duration %d too small\n",
                        tx_timeout_s);
                    PT_EXIT(pt);
                }
            }

            /* Execute command */
            SHELL_OUTPUT(output, "Running modulated CW TX on freq %lu kHz at power %d dBm for %d seconds...\n",
                RadioSettings.freq, RadioSettings.pwr, tx_timeout_s);

            leds_single_on(LEDS_GREEN);
            /* Radio stays in this mode until further notice.  The timeout just sets a lora-timer (etimer)
            * instance which triggers the OnRadioTxTimeout() callback
            */
            RadioSetTxInfinitePreamble(RadioSettings.freq, RadioSettings.pwr, tx_timeout_s);
        }
        else if (strcmp("rx", args) == 0) {

            Radio.SetChannel(RadioSettings.freq);

            Radio.SetRxConfig(MODEM_LORA, RadioSettings.bw, RadioSettings.sf,
                RadioSettings.cr, 0, RadioSettings.preamble_len,
                LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                0, true, 0, 0, RadioSettings.iq_invert, LORA_RX_CONTINUOUS);

            SHELL_OUTPUT(output, "Running RX on freq %lu kHz at SF%d CR:%d BW:%d preamble:%d IQ invert:%s ...\n",
                RadioSettings.freq, RadioSettings.sf, RadioSettings.cr, RadioSettings.bw,
                RadioSettings.preamble_len, getRadioIQInvertStr());

            /* Test IRQ by ensuring:
            1. Continuous RX mode is false
            2. Symbol timeout is less than the timeout given to Radio.Rx().
            3. 50 symbols is about 1 second.
            When continous RX mode is false, the RX timeout is determined by the shorter
            of the symbol timeout (implemented on the radio) and the timer timeout (implemented in the MCU).
            */
            Radio.Rx(0);
        }
        else if (strcmp("tx", args) == 0) {

            Radio.SetChannel(RadioSettings.freq);

            Radio.SetTxConfig(MODEM_LORA, RadioSettings.pwr, 0, RadioSettings.bw,
                RadioSettings.sf, RadioSettings.cr,
                RadioSettings.preamble_len, LORA_FIX_LENGTH_PAYLOAD_ON,
                true, 0, 0, RadioSettings.iq_invert, RadioSettings.tx_timeout);

            SHELL_OUTPUT(output, "TX %d bytes on freq %lu kHz at SF%d CR:%d BW:%d PWR:%d dBm airtime:%lu ms timeout:%lu ms  preamble:%d IQ invert:%s ...\n",
                RadioSettings.payload_len, RadioSettings.freq, RadioSettings.sf, RadioSettings.cr, RadioSettings.bw,
                RadioSettings.pwr, Radio.TimeOnAir(MODEM_LORA, RadioSettings.payload_len), RadioSettings.tx_timeout,
                RadioSettings.preamble_len, getRadioIQInvertStr());

            leds_single_on(LEDS_GREEN);

            Radio.Send(RadioSettings.payload, RadioSettings.payload_len);
        }
        else {
            SHELL_OUTPUT(output, "Invalid arg: %s\n", args);
            PT_EXIT(pt);
        }
    }

    RadioStatus_t status = SX126xGetStatus();
    SHELL_OUTPUT(output, "Command %s. Radio mode=%s status=%s\n",
        ((status.Fields.CmdStatus == 0x3 || status.Fields.CmdStatus == 0x4 || status.Fields.CmdStatus == 0x5 || (result == false)) ? "Failed" : "OK"),
        ChipModeStrings[status.Fields.ChipMode],
        CommandStatusStrings[status.Fields.CmdStatus]);

    PT_END(pt);
}

const struct shell_command_t lora_shell_commands[] = {
    { "lora",                 cmd_lora,                 "'> lora': [reg [set | get]] [set [cr | bw | sf | tx]] [off] [tx] [rx] [cw] [cwmod]" },
    { NULL, NULL, NULL },
};

static struct shell_command_set_t lora_shell_command_set = {
    .next = NULL,
    .commands = lora_shell_commands,
};


/*---------------------------------------------------------------------------*/


/*!
* \brief Function executed on Radio Tx Timeout event
*/
void OnRadioTxTimeout(void)
{
    LOG_INFO("TX timeout.\n");

    /* Get and report any errors */
    RadioError_t errors = SX126xGetDeviceErrors();
    if (errors.Value != 0) {
        SX126xClearDeviceErrors();
        LOG_ERR("Radio reported errors: 0x%04x\n", errors.Value);
    }

    leds_single_off(LEDS_GREEN);

    /* Put radio in standby */
    Radio.Standby();

    serial_shell_show_prompt();
}

/*!
* \brief Function executed on Radio Rx timeout event
*/
void OnRadioRxTimeout(void)
{
    LOG_INFO("RX timeout.\n");

    /* Get and report any errors */
    RadioError_t errors = SX126xGetDeviceErrors();
    if (errors.Value != 0) {
        SX126xClearDeviceErrors();
        LOG_ERR("Radio reported errors: 0x%04x\n", errors.Value);
    }

    /* Put radio in standby */
    Radio.Standby();

    serial_shell_show_prompt();
}

void OnTxDone(void)
{
    LOG_INFO("Tx completed.\n");

    leds_single_off(LEDS_GREEN);

    /* Put radio in standby */
    Radio.Standby();

    serial_shell_show_prompt();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    LOG_INFO("RxDone: size:%d rssi:%d snr:%d payload:[", size, rssi, snr);
    for (int i = 0; i < size; i++) {
        printf(" %02x", payload[i]);
    }
    printf(" ]\n");
}

static const char* getRxErrorCodeStr(RadioIrqErrorCode_t code)
{
    switch (code) {
    case IRQ_HEADER_ERROR_CODE:     return "HEADER ERROR";      // 1
    case IRQ_SYNCWORD_ERROR_CODE:   return "SYNCWORD ERROR";    // 2
    case IRQ_CRC_ERROR_CODE:        return "CRC ERROR";         // 4
    default:
        return "unknown";
    }
};

void OnRxError(RadioIrqErrorCode_t code)
{
    LOG_ERR("RxError code:%d -> %s\n", code, getRxErrorCodeStr(code));
    /* Get and report any errors */
    RadioError_t errors = SX126xGetDeviceErrors();
    if (errors.Value != 0) {
        SX126xClearDeviceErrors();
        LOG_ERR("Radio reported errors: 0x%04x\n", errors.Value);
    }

    serial_shell_show_prompt();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(loramac_process, ev, data)
{
    PROCESS_BEGIN();

    leds_single_off(LEDS_GREEN);
    leds_single_off(LEDS_RED);

    LOG_INFO("Starting lora-rf radio testing application\n");

    shell_command_set_register(&lora_shell_command_set);
    LOG_INFO("Registered command: lora.\n");

    setRadioSettingsDefaults();

    // Radio initialization
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    RadioEvents.RxTimeout = OnRadioRxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxError = OnRxError;
    Radio.Init(&RadioEvents);

    Radio.SetPublicNetwork(true);

    serial_shell_show_prompt();

    while (1) {
        
        PROCESS_YIELD();

        // Process Radio IRQ
        if (Radio.IrqProcess != NULL)
        {
            Radio.IrqProcess();
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
