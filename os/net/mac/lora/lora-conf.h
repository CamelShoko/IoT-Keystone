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
/**
* \file
*         LoRaMac stack configuration.
*
*
* \author Evan Ross <evan@thisisiot.io>
*/
/*!
* \file      Commissioning.h
*
* \brief     End device commissioning parameters
*
* \copyright Revised BSD License, see section \ref LICENSE.
*
* \code
*                ______                              _
*               / _____)             _              | |
*              ( (____  _____ ____ _| |_ _____  ____| |__
*               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
*               _____) ) ____| | | || |_| ____( (___| | | |
*              (______/|_____)_|_|_| \__)_____)\____)_| |_|
*              (C)2013-2017 Semtech
*
* \endcode
*
* \author    Miguel Luis ( Semtech )
*
* \author    Gregory Cristian ( Semtech )
*/
#ifndef __LORA_COMMISSIONING_H__
#define __LORA_COMMISSIONING_H__

/*!
******************************************************************************
********************************** WARNING ***********************************
******************************************************************************
The crypto-element implementation supports both 1.0.x and 1.1.x LoRaWAN
versions of the specification.
Thus it has been decided to use the 1.1.x keys and EUI name definitions.
The below table shows the names equivalence between versions:
+-------------------+-------------------------+
|       1.0.x       |          1.1.x          |
+===================+=========================+
| LORAWAN_DEVICE_EUI| LORAWAN_DEVICE_EUI      |
+-------------------+-------------------------+
| LORAWAN_APP_EUI   | LORAWAN_JOIN_EUI        |
+-------------------+-------------------------+
| N/A               | LORAWAN_APP_KEY         |
+-------------------+-------------------------+
| LORAWAN_APP_KEY   | LORAWAN_NWK_KEY         |
+-------------------+-------------------------+
| LORAWAN_NWK_S_KEY | LORAWAN_F_NWK_S_INT_KEY |
+-------------------+-------------------------+
| LORAWAN_NWK_S_KEY | LORAWAN_S_NWK_S_INT_KEY |
+-------------------+-------------------------+
| LORAWAN_NWK_S_KEY | LORAWAN_NWK_S_ENC_KEY   |
+-------------------+-------------------------+
| LORAWAN_APP_S_KEY | LORAWAN_APP_S_KEY       |
+-------------------+-------------------------+
******************************************************************************
******************************************************************************
******************************************************************************
*/

/* !!NOTE!!
 *   The following are *defaults* to be applied if no persistent values for these parameters
 *   can be retrieved.
 */

/*!
* When set to 1 the application uses the Over-the-Air activation procedure
* When set to 0 the application uses the Personalization activation procedure
*/
#ifdef LORA_CONF_OVER_THE_AIR_ACTIVATION
#define OVER_THE_AIR_ACTIVATION                            LORA_CONF_OVER_THE_AIR_ACTIVATION
#else
#define OVER_THE_AIR_ACTIVATION                            1
#endif

/*!
* When using ABP activation the MAC layer must know in advance to which server
* version it will be connected.
*/
#define ABP_ACTIVATION_LRWAN_VERSION_V10x                  0x01000300 // 1.0.3.0

#define ABP_ACTIVATION_LRWAN_VERSION                       ABP_ACTIVATION_LRWAN_VERSION_V10x

/*!
* Indicates if the end-device is to be connected to a private or public network
*/
#define LORAWAN_PUBLIC_NETWORK                             true

#ifdef LORA_CONF_IEEE_OUI
#define IEEE_OUI                                           LORA_CONF_IEEE_OUI
#else
/*!
* IEEE Organizationally Unique Identifier ( OUI ) (big endian)
* \remark This is unique to a company or organization
*/
#define IEEE_OUI                                           0x00, 0x00, 0x00
#endif

/*!
* Mote device IEEE EUI (big endian)
*
* \remark In this application the value is automatically generated by calling
*         BoardGetUniqueId function
*/
#ifdef LORA_CONF_LORAWAN_DEVICE_EUI
#define LORAWAN_DEVICE_EUI LORA_CONF_LORAWAN_DEVICE_EUI
#else
#define LORAWAN_DEVICE_EUI                                 { IEEE_OUI, 0x00, 0x00, 0x00, 0x00, 0x00 }
#endif

/*!
* App/Join server IEEE EUI (big endian)
*/
#ifdef LORA_CONF_LORAWAN_JOIN_EUI
#define LORAWAN_JOIN_EUI LORA_CONF_LORAWAN_JOIN_EUI
#else
#define LORAWAN_JOIN_EUI                                   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#endif

/*!
* Application root key
* WARNING: NOT USED FOR 1.0.x DEVICES
*/
#ifdef LORA_CONF_LORAWAN_APP_KEY
#define LORAWAN_APP_KEY LORA_CONF_LORAWAN_APP_KEY
#else
#define LORAWAN_APP_KEY                                    { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#endif

/*!
* Network root key
* WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_APP_KEY
*/
#ifdef LORA_CONF_LORAWAN_NWK_KEY
#define LORAWAN_NWK_KEY LORA_CONF_LORAWAN_NWK_KEY
#else
#define LORAWAN_NWK_KEY                                    { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#endif

/*!
* Current network ID
*/
#ifdef LORA_CONF_LORAWAN_NETWORK_ID
#define LORAWAN_NETWORK_ID LORA_CONF_LORAWAN_NETWORK_ID
#else
#define LORAWAN_NETWORK_ID                                 ( uint32_t )0
#endif

/*!
* Device address on the network (big endian)
*
* \remark In this application the value is automatically generated using
*         a pseudo random generator seeded with a value derived from
*         BoardUniqueId value if LORAWAN_DEVICE_ADDRESS is set to 0
*/
#ifdef LORA_CONF_LORAWAN_DEVICE_ADDRESS
#define LORAWAN_DEVICE_ADDRESS LORA_CONF_LORAWAN_DEVICE_ADDRESS
#else
#define LORAWAN_DEVICE_ADDRESS                             ( uint32_t )0x00000000
#endif

/*!
* Forwarding Network session integrity key
* WARNING: NWK_S_KEY FOR 1.0.x DEVICES
*/
#ifdef LORA_CONF_LORAWAN_F_NWK_S_INT_KEY
#define LORAWAN_F_NWK_S_INT_KEY LORA_CONF_LORAWAN_F_NWK_S_INT_KEY
#else
#define LORAWAN_F_NWK_S_INT_KEY                            { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#endif

/*!
* Serving Network session integrity key
* WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY
*/
#ifdef LORA_CONF_LORAWAN_S_NWK_S_INT_KEY
#define LORAWAN_S_NWK_S_INT_KEY LORA_CONF_LORAWAN_S_NWK_S_INT_KEY
#else
#define LORAWAN_S_NWK_S_INT_KEY                            { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#endif

/*!
* Network session encryption key
* WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY
*/
#ifdef LORA_CONF_LORAWAN_NWK_S_ENC_KEY
#define LORAWAN_NWK_S_ENC_KEY LORA_CONF_LORAWAN_NWK_S_ENC_KEY
#else 
#define LORAWAN_NWK_S_ENC_KEY                              { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#endif

/*!
* Application session key
*/
#ifdef LORA_CONF_LORAWAN_APP_S_KEY
#define LORAWAN_APP_S_KEY LORA_CONF_LORAWAN_APP_S_KEY
#else
#define LORAWAN_APP_S_KEY                                  { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#endif

#endif // __LORA_COMMISSIONING_H__
