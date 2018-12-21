/*
 * Copyright (c) 2018, This. Is. IoT. - https://thisisiot.io
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
 *
 * \file
 *        Driver for Invensense ICM-20948 motion sensor.
 * \author
 *        Evan Ross <evan@thisisiot.io>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "sys/ctimer.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include "board-peripherals.h"
#include "motion-sensor.h"

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/dpl/ClockP.h>
/*---------------------------------------------------------------------------*/
#include <Board.h>
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0 
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define INV_MSG_ENABLE
/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG
#else
#define PRINTF(...)
#define MSG_LEVEL INV_MSG_LEVEL_OFF
#endif
/*---------------------------------------------------------------------------*/
#define IMU_SPI_HANDLE  hSpiSensor
/*---------------------------------------------------------------------------*/
typedef struct {
  volatile MOTION_SENSOR_STATUS status;
} MOTION_SENSOR_Object;

static MOTION_SENSOR_Object motion_sensor_obj;


/* PIN driver handle */
static PIN_Handle hMotionPins;
static PIN_State pinState;


/*---------------------------------------------------------------------------*/
/* Invensense library adapter */

#define ODR_NONE       0 /* Asynchronous sensors don't need to have a configured ODR */


PROCESS(motion_sensor_process, "motion_sensor_process");

#define USE_RAW_ACC 0
#define USE_RAW_GYR 0
#define USE_GRV     0
#define USE_CAL_ACC 0
#define USE_CAL_GYR 0
#define USE_CAL_MAG 0
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_RV      0    /* requires COMPASS*/
#define USE_GEORV   0    /* requires COMPASS*/
#define USE_ORI     0    /* requires COMPASS*/
#define USE_STEPC   0
#define USE_STEPD   0
#define USE_SMD     1
#define USE_BAC     1
#define USE_TILT    1
#define USE_PICKUP  1
#define USE_GRAVITY 0
#define USE_LINACC  0
#define USE_B2S     1

/* List of virtual sensors we will support */
static const invn_lib_sensor_t sensor_list[] = {
#if USE_RAW_ACC
    { INVN_LIB_SENSOR_TYPE_RAW_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_RAW_GYR
    { INVN_LIB_SENSOR_TYPE_RAW_GYROSCOPE,     50000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
    { INVN_LIB_SENSOR_TYPE_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_GYR
    { INVN_LIB_SENSOR_TYPE_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_MAG
    { INVN_LIB_SENSOR_TYPE_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_GYR
    { INVN_LIB_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
    { INVN_LIB_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
    { INVN_LIB_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
    { INVN_LIB_SENSOR_TYPE_ROTATION_VECTOR, 1000000 /* 1 Hz */ },
#endif
#if USE_GEORV
    { INVN_LIB_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 1000000 /* 1 Hz */ },
#endif
#if USE_ORI
    { INVN_LIB_SENSOR_TYPE_ORIENTATION, 1000000 /* 1 Hz */ },
#endif
#if USE_STEPC
    { INVN_LIB_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
    { INVN_LIB_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE },
#endif
#if USE_SMD
    { INVN_LIB_SENSOR_TYPE_SMD, ODR_NONE },
#endif
#if USE_BAC
    { INVN_LIB_SENSOR_TYPE_BAC, ODR_NONE },
#endif
#if USE_TILT
    { INVN_LIB_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE },
#endif
#if USE_PICKUP
    { INVN_LIB_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE },
#endif
#if USE_GRAVITY
    { INVN_LIB_SENSOR_TYPE_GRAVITY, 500000 /* 2 Hz */ },
#endif
#if USE_LINACC
    { INVN_LIB_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */ },
#endif
#if USE_B2S
    { INVN_LIB_SENSOR_TYPE_B2S, ODR_NONE },
#endif
};




static void motionSensorIsr(PIN_Handle handle, PIN_Id pinId)
{
    /* Here we are only required to trigger invocation of the 
     * library's polling function.  This must be done in the
     * context of a Contiki process.
     */
    process_poll(&motion_sensor_process);
}


static int
open()
{
    const PIN_Config pinListMotion[] =
    {
        Board_PIN_IMU_INT | PIN_INPUT_EN | PIN_PULLDOWN, /* Enable interrupt later */
        PIN_TERMINATE
    };

    /* Acquire exclusive access to pins (through PIN API), and also
    * install handler.
    */
    hMotionPins = PIN_open(&pinState, pinListMotion);
    if (hMotionPins == NULL) {
        /* some kind of error */
        return -1;
    }

    PIN_registerIntCb(hMotionPins, motionSensorIsr);
    PIN_setInterrupt(hMotionPins, Board_PIN_IMU_INT | PIN_IRQ_POSEDGE);

    return 0;
}

static int
close()
{
    PIN_close(hMotionPins);

    return 0;
}

/*---------------------------------------------------------------------------*/
static int
motion_spi_write(uint8_t reg, const uint8_t * data, uint32_t len)
{
    SPI_Transaction spiTransaction;

    /* SPI mode: register bit 7=0 means write. */
    reg &= 0x7f;
    PRINTF("spi_write: %02x\n", reg);
    PINCC26XX_setOutputValue(Board_SPI_IMU_CS, 0);

    /* Write the register address */
    spiTransaction.count = 1;
    spiTransaction.txBuf = (void*)&reg;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(IMU_SPI_HANDLE, &spiTransaction);

    /* Write the data */
    spiTransaction.count = len;
    spiTransaction.txBuf = (void*)data;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(IMU_SPI_HANDLE, &spiTransaction);

    PINCC26XX_setOutputValue(Board_SPI_IMU_CS, 1); 

    return 0;
}
/*---------------------------------------------------------------------------*/
static int
motion_spi_read(uint8_t reg, uint8_t *data, uint32_t len)
{
    SPI_Transaction spiTransaction;

    /* SPI mode: register bit 7=1 means read. */
    reg |= 0x80;
    PRINTF("spi_read: %02x\n", reg);
    PINCC26XX_setOutputValue(Board_SPI_IMU_CS, 0);

    /* Write the register address */
    spiTransaction.count = 1;
    spiTransaction.txBuf = (void*)&reg;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(IMU_SPI_HANDLE, &spiTransaction);

    /* Read the data */
    spiTransaction.count = len;
    spiTransaction.txBuf = NULL;
    spiTransaction.rxBuf = (void*)data;
    SPI_transfer(IMU_SPI_HANDLE, &spiTransaction);

    PINCC26XX_setOutputValue(Board_SPI_IMU_CS, 1);
    return 0;
}

const invn_lib_host_serif_t invn_lib_serif = {
    0,
    0,
    motion_spi_read,
    motion_spi_write,
    0,
    32*1024, /* max transaction size set to high number to allow single invocation of large transfers. */
    32*1024, /* ditto */
    INVN_LIB_SERIF_TYPE_SPI, /* SPI */
};


void inv_icm20948_sleep_us(int us)
{
    /* NOTES
     *   Should be a blocking sleep.
     *   Controls timing in the driver including startup delays.
     *   In vast majority of cases driver uses this in terms of milliseconds (up to 100's).
     *   In one instance it uses a resolution of 100 us (0.1 ms).
     *   The platform tick counter is fairly coarse 1/128 seconds.
     */

    /*  We will use TI's ClockP sleep for microsecond resolution sleep */
    ClockP_usleep(us);
}

uint64_t inv_icm20948_get_time_us(void)
{
    /* NOTES
     *   Used in driver for timing of interrupt event processing.
     *   More specifically when the driver is actually polled by a Contiki process
     *      that was itself polled by the sensor ISR.
     */

    /* ClockP_getSystemTickPeriod() returns us per tick. On CC26xx and CC13xx
     * platforms it is 10 us per tick. 
     */
    return ((uint64_t)ClockP_getSystemTicks()) * ClockP_getSystemTickPeriod();
}


/*---------------------------------------------------------------------------*/

/* Forward the ISR event to the poll function. */
PROCESS_THREAD(motion_sensor_process, ev, data) {

    PROCESS_BEGIN();

    while (true) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_POLL) {
            /* This will trigger the invocation of the installed user sensor callback. */
            invn_lib_poll();
        }
    }

    PROCESS_END();
}

/*---------------------------------------------------------------------------*/

#if 0 
// This won't work properly.  Multiple events are generated from a single POLL.
/* Handle the events from the motion sensor. */
static void motion_sensor_event_cb(const invn_lib_sensor_event_t * event)
{
    /* notify the processes.
     * The processes are required to get the event structure out. */
    sensors_changed(&motion_sensor);
}
#endif

/*---------------------------------------------------------------------------*/
/**
 * \brief   Initialize the motion sensor driver.
 * \return  true if operation successful; else, return false.
 */
static bool
sensor_init(void)
{
    int rc = 0;

    motion_sensor_obj.status = MOTION_SENSOR_STATUS_DISABLED;

    rc = open();
    if (rc != 0) {
        return false;
    }

    /* Open the library, check return code.  If the sensor could not be found on
     * the bus, returns an error.
     */
    rc = invn_lib_init(
        INVN_LIB_MSG_LEVEL_INFO, /* have library print messages */
        invn_lib_get_demo_callback() /* use libraries demo sensor data printer */
        /*motion_sensor_event_cb*/);

    if (rc != 0) {
        close();
        return false;
    }

    /* Now start the process */
    process_start(&motion_sensor_process, NULL);

    return true;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief         Enable/disable sensor
 * \param enable  Start sensor if true; else, stop sensor.
 * \return        true if operation succeeded, false if error.
 */
static bool
sensor_enable(bool enable)
{
    /* Don't do anything if we couldn't init the sensor through configure(). */
    if (motion_sensor_obj.status == MOTION_SENSOR_STATUS_DISABLED) {
       return false;
    }
    int rc = 0;
    if (enable) {

        rc = invn_lib_start(
                sensor_list,
                sizeof(sensor_list) / sizeof(sensor_list[0]));
        if (rc == 0) {
            return true;
        }
    }
    else {
    }

    return false;

}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/**
 * \brief       
 * \param type  
 * \return      
 */
static int
value(int type)
{
  
  /* return error unless explicilty set successful */
  int rv = MOTION_SENSOR_READING_ERROR;

  if(motion_sensor_obj.status != MOTION_SENSOR_STATUS_DATA_READY) {
      return rv;
  }

  switch(type)
  {
    default:
        break;

  }

  return rv;

}
/*---------------------------------------------------------------------------*/
/**
 * \brief         Configuration function for the OPT3001 sensor.
 * \param type    Activate, enable or disable the sensor. See below.
 * \param enable  Enable or disable sensor.
 *
 *                When type == SENSORS_HW_INIT we open the PDM and I2S drivers.
 *                When type == SENSORS_ACTIVE and enable==1 we enable the sensor.
 *                When type == SENSORS_ACTIVE and enable==0 we disable the sensor.
 *                When type == MOTION_SENSOR_SELF_TEST, enable ignored.
 *               
 */
static int
configure(int type, int enable)
{
  int rv = 0;
  switch(type) {
  case SENSORS_HW_INIT:
    PRINTF("Motion sensor init\n");
    if(sensor_init()) {
      motion_sensor_obj.status = MOTION_SENSOR_STATUS_STANDBY;
      PRINTF("Motion sensor init [success]\n");
    } else {
      motion_sensor_obj.status = MOTION_SENSOR_STATUS_DISABLED;
      rv = MOTION_SENSOR_READING_ERROR;
      PRINTF("Motion sensor init [FAIL]\n");
    }
    break;

  /* Case when SENSORS_ACTIVATE() is called. */
  case SENSORS_ACTIVE:
    if(enable) {
      if (!sensor_enable(true)) {
        rv = MOTION_SENSOR_READING_ERROR;
      }
    } else {
      if (!sensor_enable(false)) {
        rv = AUDIO_SENSOR_READING_ERROR;
      }
    }
    break;
#if 0 /* NOT YET */
  /* Run a self test */
  case MOTION_SENSOR_SELF_TEST:
    /* Don't do anything if we couldn't init the sensor through configure(). */
    if (motion_sensor_obj.status == MOTION_SENSOR_STATUS_STANDBY) {
      rv = invn_lib_self_test();
    }
    else {
      rv = MOTION_SENSOR_READING_ERROR;
    }
    break;
#endif
  default:
    break;
  }

  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns the status of the sensor.
 * \param type  Ignored.
 * \return      The status of the sensor.
 */
static int
status(int type)
{
  /* Unused args */
  (void)type;

  return motion_sensor_obj.status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(motion_sensor, "MOTION", value, configure, status);
/*---------------------------------------------------------------------------*/
