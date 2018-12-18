/* 
 * Copyright (c) 2018, THIS.IS.IoT. - https://thisisiot.io
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
 * *  Neither the name of Texas Instruments Incorporated nor the names of
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

/*
 *  Header file with types to adapt Invensense driver library to Contiki.
 *
 *  How to use this library:
 *
 *   Implement void inv_icm20948_sleep_us(int us).
 *     Should be a blocking sleep.
 *     Controls timing in the driver including startup delays.
 *     In vast majority of cases driver uses this in terms of milliseconds (up to 100's).
 *     In one instance it uses a resolution of 100 us (0.1 ms).
 *
 *   Implement uint64_t inv_icm20948_get_time_us(void).
 *     Return a system-dependent time in microseconds.  
 *     Needs to be monotonic - each call produces increasing values.
 *     This needs to be implemented using the most precise timer available - microsecond resolution prefered.
 *
 *   Create instance of invn_lib_host_serif_t called invn_lib_serif.
 *     Needs read_reg and write_reg function implementations for the desired (SPI or I2C) bus.
 *     open, close, register_interrupt_callback  not used can be set to 0.
 *     max read/write size is arbitrary.  Set to a high number, e.g. 32*1024.
 *
 *   Create array of invn_lib_sensor_t structures.  One element per sensor type
 *   to activiate in the motion sensor.  The list of available sensor types is
 *   found in enum INVN_LIB_SENSOR_TYPE.
 *
 */   
 
#ifndef INVN_LIB_H
#define INVN_LIB_H

#include "invn-lib-sensors.h"

/*---------------------------------------------------------------------------*/

enum invn_lib_serif_error {
    INVN_LIB_SERIF_ERROR_SUCCESS = 0,
    INVN_LIB_SERIF_ERROR = -1,
    INVN_LIB_SERIF_ERROR_NOT_FOUND = -2,
    INVN_LIB_SERIF_ERROR_OPEN = -3,
};

enum invn_lib_msg_level {
    INVN_LIB_MSG_LEVEL_OFF = 0,
    INVN_LIB_MSG_LEVEL_ERROR,
    INVN_LIB_MSG_LEVEL_WARNING,
    INVN_LIB_MSG_LEVEL_INFO,
    INVN_LIB_MSG_LEVEL_VERBOSE,
    INVN_LIB_MSG_LEVEL_DEBUG,
    INVN_LIB_MSG_LEVEL_MAX
};

enum invn_lib_serif_type {
    INVN_LIB_SERIF_TYPE_NONE = 0,
    INVN_LIB_SERIF_TYPE_I2C = 1,
    INVN_LIB_SERIF_TYPE_SPI = 2,
};

typedef struct invn_lib_host_serif
{
    int(*open)(void);
    int(*close)(void);
    int(*read_reg)(uint8_t reg, uint8_t * data, uint32_t len);
    int(*write_reg)(uint8_t reg, const uint8_t * data, uint32_t len);
    int(*register_interrupt_callback)(
        void(*interrupt_cb)(void * context, int int_num), void * context);
    uint32_t    max_read_size;
    uint32_t    max_write_size;
    int       serif_type;
} invn_lib_host_serif_t;

typedef struct invn_lib_sensor
{
    uint8_t type; /* one of enum invn_lib_sensor_type */
    uint32_t period_us; /* reporting period.  There are limits on this. */

} invn_lib_sensor_t;

/*---------------------------------------------------------------------------*/


/* This structure must be defined and implemented by the Keystone platform. */
extern const invn_lib_host_serif_t invn_lib_serif;

/*---------------------------------------------------------------------------*/

/*
 * Access a sensor event callback that prints out data received for all supported
 * events to the invn_lib message logger. (msg_level must not be OFF to see output).
 * This can be supplied to invn_lib_init.
 */
invn_lib_sensor_listener_event_cb_t invn_lib_get_demo_callback();

/*
 * Init the driver library.
 *
 * Library won't link unless there is a `const invn_lib_host_serif invn_lib_serif` defined
 * by application.
 *
 * @param msg_level - enable internal debug message printing to console.
 * @param sensor_event_cb - user function to call with sensor event data.
 *
 * @return one of the invn_lib_serif_error codes.
 */
int invn_lib_init(
    enum invn_lib_msg_level msg_level,
    invn_lib_sensor_listener_event_cb_t sensor_event_cb);

/*
 * Poll the driver library on reception of an event (interrupt).
 *
 * This must be called from PROCESS context (not interrupt context).
 * As a result, the sensor listener callback may be invoked.
 *
 */
int invn_lib_poll();

/*
 * Start up one or more virtual sensors on the motion sensor.
 * Data and/or events will be streamed to the application
 * through the sensor callback interface for each enabled virtual sensor.
 */
int invn_lib_start(
    const invn_lib_sensor_t* sensor_list,
    int num_sensors);


#endif /* INVN_LIB_H */
