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
*  Header file with motion sensor virtual sensor interface.
*
*  How to use virtual sensors:
*
*
*/

#ifndef INVN_LIB_SENSORS_H
#define INVN_LIB_SENSORS_H

/*---------------------------------------------------------------------------*/


typedef int invn_lib_bool_t;

/*
 * Types of virtual sensors available to the invn_lib_start() API.
 */
enum invn_lib_sensor_type {
    INVN_LIB_SENSOR_TYPE_RESERVED = 0,  /**< Reserved ID: do not use */
    INVN_LIB_SENSOR_TYPE_ACCELEROMETER = 1,  /**< Accelerometer */
    INVN_LIB_SENSOR_TYPE_MAGNETOMETER = 2,  /**< Magnetic field */
    INVN_LIB_SENSOR_TYPE_ORIENTATION = 3,  /**< Deprecated orientation */
    INVN_LIB_SENSOR_TYPE_GYROSCOPE = 4,  /**< Gyroscope */
    INVN_LIB_SENSOR_TYPE_LIGHT = 5,  /**< Ambient light sensor */
    INVN_LIB_SENSOR_TYPE_PRESSURE = 6,  /**< Barometer */
    INVN_LIB_SENSOR_TYPE_TEMPERATURE = 7,  /**< Temperature */
    INVN_LIB_SENSOR_TYPE_PROXIMITY = 8,  /**< Proximity */
    INVN_LIB_SENSOR_TYPE_GRAVITY = 9,  /**< Gravity */
    INVN_LIB_SENSOR_TYPE_LINEAR_ACCELERATION = 10,  /**< Linear acceleration */
    INVN_LIB_SENSOR_TYPE_ROTATION_VECTOR = 11,  /**< Rotation vector */
    INVN_LIB_SENSOR_TYPE_HUMIDITY = 12,  /**< Relative humidity */
    INVN_LIB_SENSOR_TYPE_AMBIENT_TEMPERATURE = 13,  /**< Ambient temperature */
    INVN_LIB_SENSOR_TYPE_UNCAL_MAGNETOMETER = 14,  /**< Uncalibrated magnetic field */
    INVN_LIB_SENSOR_TYPE_GAME_ROTATION_VECTOR = 15,  /**< Game rotation vector */
    INVN_LIB_SENSOR_TYPE_UNCAL_GYROSCOPE = 16,  /**< Uncalibrated gyroscope */
    INVN_LIB_SENSOR_TYPE_SMD = 17,  /**< Significant motion detection */
    INVN_LIB_SENSOR_TYPE_STEP_DETECTOR = 18,  /**< Step detector */
    INVN_LIB_SENSOR_TYPE_STEP_COUNTER = 19,  /**< Step counter */
    INVN_LIB_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR = 20,  /**< Geomagnetic rotation vector */
    INVN_LIB_SENSOR_TYPE_HEART_RATE = 21,  /**< Heart rate */
    INVN_LIB_SENSOR_TYPE_TILT_DETECTOR = 22,  /**< Tilt detector */
    INVN_LIB_SENSOR_TYPE_WAKE_GESTURE = 23,  /**< Wake-up gesture  */
    INVN_LIB_SENSOR_TYPE_GLANCE_GESTURE = 24,  /**< Glance gesture  */
    INVN_LIB_SENSOR_TYPE_PICK_UP_GESTURE = 25,  /**< Pick-up gesture */
    INVN_LIB_SENSOR_TYPE_BAC = 26,  /**< Basic Activity Classifier */
    INVN_LIB_SENSOR_TYPE_PDR = 27,  /**< Pedestrian Dead Reckoning */
    INVN_LIB_SENSOR_TYPE_B2S = 28,  /**< Bring to see */
    INVN_LIB_SENSOR_TYPE_3AXIS = 29,  /**< 3 Axis sensor */
    INVN_LIB_SENSOR_TYPE_EIS = 30,  /**< Electronic Image Stabilization */
    INVN_LIB_SENSOR_TYPE_OIS = 31,  /**< Optical Image Stabilization */
    INVN_LIB_SENSOR_TYPE_RAW_ACCELEROMETER = 32,  /**< Raw accelerometer */
    INVN_LIB_SENSOR_TYPE_RAW_GYROSCOPE = 33,  /**< Raw gyroscope */
    INVN_LIB_SENSOR_TYPE_RAW_MAGNETOMETER = 34,  /**< Raw magnetometer */
    INVN_LIB_SENSOR_TYPE_RAW_TEMPERATURE = 35,  /**< Raw temperature */
    INVN_LIB_SENSOR_TYPE_CUSTOM_PRESSURE = 36,  /**< Custom Pressure Sensor */
    INVN_LIB_SENSOR_TYPE_MIC = 37,  /**< Stream audio from microphone */
    INVN_LIB_SENSOR_TYPE_TSIMU = 38,  /**< TS-IMU */
    INVN_LIB_SENSOR_TYPE_RAW_PPG = 39,  /**< Raw Photoplethysmogram */
    INVN_LIB_SENSOR_TYPE_HRV = 40,  /**< Heart rate variability */
    INVN_LIB_SENSOR_TYPE_SLEEP_ANALYSIS = 41,  /**< Sleep analysis */
    INVN_LIB_SENSOR_TYPE_BAC_EXTENDED = 42,  /**< Basic Activity Classifier Extended */
    INVN_LIB_SENSOR_TYPE_BAC_STATISTICS = 43,  /**< Basic Activity Classifier Statistics */
    INVN_LIB_SENSOR_TYPE_FLOOR_CLIMB_COUNTER = 44,  /**< Floor Climbed Counter */
    INVN_LIB_SENSOR_TYPE_ENERGY_EXPENDITURE = 45,  /**< Energy Expenditure */
    INVN_LIB_SENSOR_TYPE_DISTANCE = 46,  /**< Distance */
    INVN_LIB_SENSOR_TYPE_SHAKE = 47,  /**< Shake Gesture */
    INVN_LIB_SENSOR_TYPE_DOUBLE_TAP = 48,  /**< Double Tap */
    INVN_LIB_SENSOR_TYPE_CUSTOM0,                            /**< Custom sensor ID 0 */
    INVN_LIB_SENSOR_TYPE_CUSTOM1,                            /**< Custom sensor ID 1 */
    INVN_LIB_SENSOR_TYPE_CUSTOM2,                            /**< Custom sensor ID 2 */
    INVN_LIB_SENSOR_TYPE_CUSTOM3,                            /**< Custom sensor ID 3 */
    INVN_LIB_SENSOR_TYPE_CUSTOM4,                            /**< Custom sensor ID 4 */
    INVN_LIB_SENSOR_TYPE_CUSTOM5,                            /**< Custom sensor ID 5 */
    INVN_LIB_SENSOR_TYPE_CUSTOM6,                            /**< Custom sensor ID 6 */
    INVN_LIB_SENSOR_TYPE_CUSTOM7,                            /**< Custom sensor ID 7 */
    INVN_LIB_SENSOR_TYPE_WOM,                                /**< Wake-up on motion */
    INVN_LIB_SENSOR_TYPE_SEDENTARY_REMIND,                   /**< Sedentary Remind */
    INVN_LIB_SENSOR_TYPE_DATA_ENCRYPTION,                    /**< Data Encryption */
    INVN_LIB_SENSOR_TYPE_FSYNC_EVENT,                        /**< FSYNC event */
    INVN_LIB_SENSOR_TYPE_HIGH_RATE_GYRO,                     /**< High Rate Gyro */
    INVN_LIB_SENSOR_TYPE_CUSTOM_BSCD,                        /**< Custom BAC StepCounter Calorie counter and Distance counter */
    INVN_LIB_SENSOR_TYPE_HRM_LOGGER,                         /**< HRM ouput for logger */
                                                             /* Starting from there, the SensorID is coded with more than 6bits so check that communication protocol is adequate */
                                                             INVN_LIB_SENSOR_TYPE_PREDICTIVE_QUATERNION,              /**< Predictive Quaternion */
                                                             INVN_LIB_SENSOR_TYPE_MAX                                 /**< sentinel value for sensor type */
};


#define INVN_LIB_SENSOR_TYPE_WU_FLAG        (unsigned int)(0x80000000)

enum invn_lib_sensor_status
{
    INVN_LIB_SENSOR_STATUS_DATA_UPDATED = 0,    /**< new sensor data */
    INVN_LIB_SENSOR_STATUS_STATE_CHANGED = 1,    /**< dummy sensor data indicating
                                            to a change in sensor state */
    INVN_LIB_SENSOR_STATUS_FLUSH_COMPLETE = 2,    /**< dummy sensor data indicating
                                             a end of batch after a manual flush */
    INVN_LIB_SENSOR_STATUS_POLLED_DATA = 3,    /**< sensor data value after manual request */
};

/*
 * Event definition for BAC sensor
 */
enum invn_lib_sensor_bac_event {
    INVN_LIB_SENSOR_BAC_EVENT_ACT_UNKNOWN = 0,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN = 1,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END = -1,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN = 2,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_WALKING_END = -2,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN = 3,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_RUNNING_END = -3,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN = 4,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END = -4,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_TILT_BEGIN = 5,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_TILT_END = -5,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_STILL_BEGIN = 6,
    INVN_LIB_SENSOR_BAC_EVENT_ACT_STILL_END = -6,
};

/*
 * Event definition for BAC Ext sensor
 */
enum invn_lib_sensor_bacext_event {
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_UNKNOWN = 0,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_WALKING_START = 1,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_WALKING_END = -1,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_RUNNING_START = 2,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_RUNNING_END = -2,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_ON_BICYCLE_START = 3,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_ON_BICYCLE_END = -3,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_SIT_START = 4,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_SIT_END = -4,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_STAND_START = 5,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_IN_VEHICLE_STAND_END = -5,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_STILL_SIT_START = 6,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_STILL_SIT_END = -6,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_STILL_STAND_START = 7,
    INVN_LIB_SENSOR_BACEXT_EVENT_ACT_STILL_STAND_END = -7
};

/* Maximum size of event data */
#define INVN_LIB_SENSOR_EVENT_DATA_SIZE      64


/* Sensor event definition */
typedef struct invn_lib_sensor_event
{
    unsigned int         sensor;           /**< sensor type */
    int                  status;           /**< sensor data status as of
                                           enum invn_lib_sensor_status */
    uint64_t             timestamp;        /**< sensor data timestamp in us */
    union {
        struct {
            float        vect[3];          /**< x,y,z vector data */
            float        bias[3];          /**< x,y,z bias vector data */
            uint8_t      accuracy_flag;    /**< accuracy flag */
        } acc;                             /**< 3d accelerometer data in g */
        struct {
            float        vect[3];          /**< x,y,z vector data */
            float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
            uint8_t      accuracy_flag;    /**< accuracy flag */
        } mag;                             /**< 3d magnetometer data in uT */
        struct {
            float        vect[3];          /**< x,y,z vector data */
            float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
            uint8_t      accuracy_flag;    /**< accuracy flag */
        } gyr;                             /**< 3d gyroscope data in deg/s */
        struct {
            float        quat[4];          /**< w,x,y,z quaternion data */
            float        accuracy;         /**< heading accuracy in deg */
            uint8_t      accuracy_flag;    /**< accuracy flag specific for GRV*/
        } quaternion;                      /**< quaternion data */
        struct {
            float        x, y, z;            /**< x,y,z angles in deg as defined by Google Orientation sensor */
            uint8_t      accuracy_flag;    /**< heading accuracy in deg */
        } orientation;                     /**< orientation data */
        struct {
            float        bpm;              /**< beat per minute */
            uint8_t      confidence;       /**< confidence level */
            uint8_t      sqi;              /**< signal quality as seen by the the HRM engine */
        } hrm;                             /**< heart rate monitor data */                            /**< heart rate monitor data */
        struct {
            int32_t      acc[3];           /**< accel data used by hrm algorithm */
            int32_t      gyr[3];           /**< gyro data used by hrm algorithm */
            uint32_t     ppg_value;        /**< ppg value read from HRM sensor */
            float        ppm;              /**< beat per minute */
            uint8_t      confidence;       /**< confidence level */
            uint8_t      sqi;              /**< signal quality as seen by the the HRM engine */
            uint8_t      touch_status;     /**< touch status, detected or not by the PPG */
            uint8_t      gyrEnable;        /**< 1 gyro is enable else 0 */
        } hrmlogger;                       /**< heart rate monitor logger data */
        struct {
            uint8_t      rr_count;
            int16_t      rr_interval[4];   /**< beat-to-beat(RR) interval */
        } hrv;                             /**< heart rate variability data */
        struct {
            uint32_t     ppg_value;        /**< ppg value read from HRM sensor */
            uint8_t      touch_status;     /**< touch status, detected or not */
        } rawppg;                          /**< raw heart rate monitor data */
        struct {
            uint8_t      sleep_phase;      /**< state of sleep phases: 0 not defined, 1 restless sleep, 2 light sleep, 3 deep sleep */
            uint32_t     timestamp;        /**< time stamp of the sleep phase transition (seconds)*/
            int32_t      sleep_onset;      /**< time until first period of 20 min sleep without more than 1 min wake */
            int32_t      sleep_latency;    /**< time until first sleep phase */
            uint32_t     time_in_bed;      /**< time in bed (seconds) */
            uint32_t     total_sleep_time; /**< total sleep time (seconds) */
            uint8_t      sleep_efficiency; /**< ratio between total sleep time and time in bed */
        } sleepanalysis;                   /**< sleep analysis data */
        struct {
            int          event;            /**< BAC extended data begin/end event as of
                                           enum invn_lib_sensor_bac_ext_event */
        } bacext;                          /**< activity classifier (BAC) extended data */
        struct {
            uint32_t     durationWalk;          /**< ms */
            uint32_t     durationRun;           /**< ms */
            uint32_t     durationTransportSit;  /**< ms */
            uint32_t     durationTransportStand;/**< ms */
            uint32_t     durationBiking;        /**< ms */
            uint32_t     durationStillSit;      /**< ms */
            uint32_t     durationStillStand;    /**< ms */
            uint32_t     durationTotalSit;      /**< Still-Sit + Transport-Sit + Biking (ms) */
            uint32_t     durationTotalStand;    /**< Still-Stand + Transport-Stand (ms) */
            uint32_t     stepWalk;              /**< walk step count */
            uint32_t     stepRun;               /**< run step count */
        } bacstat;                              /**< activity classifier (BAC) statistics data */
        struct {
            int32_t      floorsUp;         /**< number of floors climbed Up on foot by user. */
            int32_t      floorsDown;       /**< number of floors climbed Down on foot by user. */
        } floorclimb;                      /**< floor climbed data */
        struct {
            int32_t      instantEEkcal;    /**< energy expenditure in kilocalorie/min since last output. Format is q15: 2^15 = 1 kcal/min */
            int32_t      instantEEmets;    /**< energy expenditure in METs(Metabolic Equivalent of Task) since last output. Format is q15: 2^15 = 1 METs */
            int32_t      cumulativeEEkcal; /**< cumulative energy expenditure since the last reset in kilocalorie. Format is q0: 1 = 1 kcal */
            int32_t      cumulativeEEmets; /**< cumulative energy expenditure since the last reset in METs (Metabolic Equivalent of Task). Format is q0: 1 = 1 METs */
        } energyexp;                       /**< energy expenditure data */
        struct {
            int32_t      distanceWalk;     /**< distance in meters */
            int32_t      distanceRun;      /**< distance in meters */
        } distance;                        /**< distance data */
        struct {
            float        tmp;              /**< temperature in deg celcius */
        } temperature;                     /**< temperature data */
        struct {
            uint64_t     count;            /**< number of steps */
        } step;                            /**< step-counter data */
        struct {
            uint32_t     level;            /**< light level in lux */
        } light;                           /**< light data */
        struct {
            uint32_t     distance;         /**< distance in mm */
        } proximity;                       /**< proximity data */
        struct {
            uint32_t     pressure;         /**< pressure in Pa */
        } pressure;                        /**< pressure data */
        struct {
            int          event;            /**< BAC data begin/end event as of
                                           enum invn_lib_sensor_bac_event */
        } bac;                             /**< BAC data */
        struct {
            uint32_t     fxdata[12];       /**< PDR data in fixpoint*/
        } pdr;                             /**< PDR data */
        struct {
            float        vect[3];          /**< x,y,z vector data */
            float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
            int16_t      delta_ts;         /**< timestamp delta between standard gyro and EIS gyro */
        } eis;                             /**< EIS data
                                           @warning experimental: structure is likely to change in near future */
        struct {
            int32_t      vect[3];          /**< x,y,z vector data */
            uint32_t     fsr;              /**< full scale range */
        } raw3d;                           /**< 3d raw acc, mag or gyr*/
        struct {
            int32_t      raw;              /**< raw temperature value */
        } rawtemp;                         /**< Raw temperature data*/
        struct {
            uint8_t      status[6];        /**< raw temperature value */
        } tsimu_status;                    /**< TSIMU status data*/
        invn_lib_bool_t       event;            /**< event state for gesture-like sensor
                                           (SMD, B2S, Step-detector, Tilt-detector, Wake, Glance, Pick-Up, Shake, Double-tap, ...) */
        struct {
            int16_t delay_count;           /**< delay counter in us between FSYNC tag and previous gyro data */
        } fsync_event;                     /** < FSYNC tag (EIS sensor) */
        struct {
            unsigned     flags;             /** WOM status flags: non-zero value - motion detected
                                            bit0 - motion detected around X axis
                                            bit1 - motion detected around Y axis
                                            bit2 - motion detected around Z axis
                                            */
        } wom;                              /** Wake-up on motion data */
        struct {
            struct {
                int        event;          /**< BAC data begin/end event as of  enum invn_lib_sensor_bac_event */
            } bac;                         /**< BAC data */
            struct {
                uint64_t   count;          /**< number of steps */
            } step;                        /**< step-counter data */
            int32_t      cumulativeEEkcal; /**< cumulative energy expenditure since the last reset in kilocalorie. Format is q0: 1 = 1 kcal */
            int32_t      distance;         /**< sum of walk and run distance in meters */
        } bscd;                            /**< buffer of custom BSCD */
        uint8_t          reserved[INVN_LIB_SENSOR_EVENT_DATA_SIZE];     /**< reserved sensor data for future sensor */
    } data;                                /**< sensor data */
} invn_lib_sensor_event_t;


/* Retrieve sensor type (without wake-up flag) from a sensor id.
*/
#define INVN_LIB_SENSOR_ID_TO_TYPE(sensor) \
	((unsigned int)(sensor) & ~INVN_LIB_SENSOR_TYPE_WU_FLAG)

/* Check if given sensor is of known type
*/
#define INVN_LIB_SENSOR_IS_VALID(sensor) \
	(INVN_LIB_SENSOR_ID_TO_TYPE(sensor) < INVN_LIB_SENSOR_TYPE_MAX)

/* Check if given sensor is a wake-up sensor
*/
#define INVN_LIB_SENSOR_IS_WU(sensor) \
 	(((int)(sensor) & INVN_LIB_SENSOR_TYPE_WU_FLAG) != 0)

/* 
 * User callback to receive the sensor events.
 * This callback is supplied to invn_lib_init, and is invoked in the context of invn_lib_poll
 *
 */
typedef void(*invn_lib_sensor_listener_event_cb_t)(const invn_lib_sensor_event_t * event);


#endif /* INVN_LIB_SENSORS_H */


