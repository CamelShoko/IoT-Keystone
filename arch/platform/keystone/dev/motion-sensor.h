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
 *        The Invensense ICM-20948 9-axis Motion Sensor offers an incredibly powerful set
 *        of spatial awareness functions in one tiny, low power, sensor package.
 *        
 *        Applications built on the IoT.Keystone platform can leverage the
 *        Motion Sensor functions through this driver interface such as:
 *           
 MOTION_SENSOR_TYPE_ACCELEROMETER Accelerometer.
 MOTION_SENSOR_TYPE_MAGNETOMETER Magnetic field.
 MOTION_SENSOR_TYPE_ORIENTATION Deprecated orientation.
 MOTION_SENSOR_TYPE_GYROSCOPE Gyroscope.
 MOTION_SENSOR_TYPE_GRAVITY Gravity.
 MOTION_SENSOR_TYPE_LINEAR_ACCELERATION Linear acceleration.
 MOTION_SENSOR_TYPE_ROTATION_VECTOR Rotation vector.
 MOTION_SENSOR_TYPE_UNCAL_MAGNETOMETER Uncalibrated magnetic field.
 MOTION_SENSOR_TYPE_GAME_ROTATION_VECTOR Game rotation vector.
 MOTION_SENSOR_TYPE_UNCAL_GYROSCOPE Uncalibrated gyroscope.
 MOTION_SENSOR_TYPE_SMD Significant motion detection.
 MOTION_SENSOR_TYPE_STEP_DETECTOR Step detector.
 MOTION_SENSOR_TYPE_STEP_COUNTER Step counter.
 MOTION_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR Geomagnetic rotation vector.
 MOTION_SENSOR_TYPE_TILT_DETECTOR Tilt detector.
 MOTION_SENSOR_TYPE_WAKE_GESTURE Wake-up gesture.
 MOTION_SENSOR_TYPE_GLANCE_GESTURE Glance gesture.
 MOTION_SENSOR_TYPE_PICK_UP_GESTURE Pick-up gesture.
 MOTION_SENSOR_TYPE_BAC Basic Activity Classifier.
 MOTION_SENSOR_TYPE_PDR Pedestrian Dead Reckoning.
 MOTION_SENSOR_TYPE_B2S Bring to see.
 MOTION_SENSOR_TYPE_RAW_ACCELEROMETER Raw accelerometer.
 MOTION_SENSOR_TYPE_RAW_GYROSCOPE Raw gyroscope.
 MOTION_SENSOR_TYPE_RAW_MAGNETOMETER Raw magnetometer.
 MOTION_SENSOR_TYPE_RAW_TEMPERATURE Raw temperature.
 MOTION_SENSOR_TYPE_BAC_EXTENDED Basic Activity Classifier Extended.
 MOTION_SENSOR_TYPE_BAC_STATISTICS Basic Activity Classifier Statistics.
 MOTION_SENSOR_TYPE_FLOOR_CLIMB_COUNTER Floor Climbed Counter.
 MOTION_SENSOR_TYPE_ENERGY_EXPENDITURE Energy Expenditure.
 MOTION_SENSOR_TYPE_DISTANCE Distance.
 MOTION_SENSOR_TYPE_SHAKE Shake Gesture.
 MOTION_SENSOR_TYPE_DOUBLE_TAP Double Tap.
 MOTION_SENSOR_TYPE_WOM Wake-up on motion.
 MOTION_SENSOR_TYPE_SEDENTARY_REMIND Sedentary Remind.

 * \file
 *        Header file for the Invensense ICM-20948 9-axis Motion Sensor.
 *
 * \author
 *        Evan Ross <evan@thisisiot.io>
 */
/*---------------------------------------------------------------------------*/
#ifndef MOTION_SENSOR_H_
#define MOTION_SENSOR_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include "lib/invn-lib.h"
/*---------------------------------------------------------------------------*/
#define MOTION_SENSOR_READING_ERROR  -1 /* general error */
/*---------------------------------------------------------------------------*/
/* Module configuration */


/*---------------------------------------------------------------------------*/
typedef enum {
  MOTION_SENSOR_STATUS_DISABLED,
  MOTION_SENSOR_STATUS_STANDBY,
  MOTION_SENSOR_STATUS_DATA_READY,
  MOTION_SENSOR_STATUS_BUS_ERROR,
} MOTION_SENSOR_STATUS;


/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor motion_sensor;
/*---------------------------------------------------------------------------*/
#endif /* AUDIO_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
