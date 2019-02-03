// Copyright (c) 2017,2018,2019 Peter Budd. All rights reserved
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//     * The above copyright notice and this permission notice shall be included in all copies or substantial
//       portions of the Software.
//     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//       BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//       IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//       WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//       SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER,
//       ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY
//       FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW


#include <linux/types.h>
#include <stdint.h>

void setup(void);
int32_t collectPacket(void);
int32_t sendPacket(uint32_t channelNumber, uint32_t dataLength);
void start(uint32_t dataRate);
void handleEvent(void);
void parseGameRotationVector(void);
void parseStabilisedRotationVector(void);
void parseStabilisedGameRotationVector(void);
void parseStability(void);
void parseCalibratedGyroscope(void);
void parseLinearAccelerometer(void);
void parseAccelerometer(void);
void parseGyroIntegratedRotationVector(void);
void parseCalibratedMagneticField(void);
void setStandardOrientation(void);
void reportFeatureResponse(void);
void reportCommandResponse(void);
void saveCalibration(void);
int reorient(float w, float x, float y, float z);
int runtimeReorient(float w, float x, float y, float z);
void parseEvent(void);
void setCalibration(char accel, char gyro, char mag);
void tare(void);
void tareZ(void);
int readFrsRecord(uint16_t recordType);
int writeFrsWord(uint16_t recordType, uint32_t offset, uint32_t data);
int readFrsWord(uint16_t recordType, uint32_t offset, uint32_t* result);
void configureStability(uint32_t reportPeriod);
void alignFIFOs(void);
int configureFeatureReport(uint8_t report, uint32_t reportPeriod);
void pushData(void);
float savGol(unsigned int startPosition);
void pushTestData(void);
void clearPersistentTare(void);
void setupStabilityClassifierFrs(float threshold);
int startRun(void);
int eraseFrsRecord(uint16_t recordType);
void printGyroIntegratedRotationVectorFRS(void);
void setGyroIntegratedRotationVectorFRS(uint32_t axes, uint32_t rate, uint32_t maxError);
int writeFrsRecord(uint16_t recordType, uint32_t length);
int writeFrsRecordWords(uint32_t offset,uint32_t data0, uint32_t data1);

const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_FRS_WRITE_RESPONSE 0XF5
#define SHTP_REPORT_FRS_WRITE_REQUEST 0xF7
#define SHTP_REPORT_FRS_WRITE_DATA_REQUEST 0xF6
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_GET_FEATURE_REQUEST 0xFE

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR 0x29
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A


//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 32768 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

#define STATIC_CALIBRATION_AGM                   (0x7979)
#define NOMINAL_CALIBRATION                      (0x4D4D)
#define STATIC_CALIBRATION_SRA                   (0x8A8A)
#define NOMINAL_CALIBRATION_SRA                  (0x4E4E)
#define DYNAMIC_CALIBRATION                      (0x1F1F)
#define ME_POWER_MGMT                            (0xD3E2)
#define SYSTEM_ORIENTATION                       (0x2D3E)
#define ACCEL_ORIENTATION                        (0x2D41)
#define SCREEN_ACCEL_ORIENTATION                 (0x2D43)
#define GYROSCOPE_ORIENTATION                    (0x2D46)
#define MAGNETOMETER_ORIENTATION                 (0x2D4C)
#define ARVR_STABILIZATION_RV                    (0x3E2D)
#define ARVR_STABILIZATION_GRV                   (0x3E2E)
#define TAP_DETECT_CONFIG                        (0xC269)
#define SIG_MOTION_DETECT_CONFIG                 (0xC274)
#define SHAKE_DETECT_CONFIG                      (0x7D7D)
#define MAX_FUSION_PERIOD                        (0xD7D7)
#define SERIAL_NUMBER                            (0x4B4B)
#define ES_PRESSURE_CAL                          (0x39AF)
#define ES_TEMPERATURE_CAL                       (0x4D20)
#define ES_HUMIDITY_CAL                          (0x1AC9)
#define ES_AMBIENT_LIGHT_CAL                     (0x39B1)
#define ES_PROXIMITY_CAL                         (0x4DA2)
#define ALS_CAL                                  (0xD401)
#define PROXIMITY_SENSOR_CAL                     (0xD402)
#define PICKUP_DETECTOR_CONFIG                   (0x1B2A)
#define FLIP_DETECTOR_CONFIG                     (0xFC94)
#define STABILITY_DETECTOR_CONFIG                (0xED85)
#define ACTIVITY_TRACKER_CONFIG                  (0xED88)
#define SLEEP_DETECTOR_CONFIG                    (0xED87)
#define TILT_DETECTOR_CONFIG                     (0xED89)
#define POCKET_DETECTOR_CONFIG                   (0xEF27)
#define CIRCLE_DETECTOR_CONFIG                   (0xEE51)
#define USER_RECORD                              (0x74B4)
#define ME_TIME_SOURCE_SELECT                    (0xD403)
#define UART_FORMAT                              (0xA1A1)
#define GYRO_INTEGRATED_RV_CONFIG                (0xA1A2)
#define FRS_ID_META_RAW_ACCELEROMETER            (0xE301)
#define FRS_ID_META_ACCELEROMETER                (0xE302)
#define FRS_ID_META_LINEAR_ACCELERATION          (0xE303)
#define FRS_ID_META_GRAVITY                      (0xE304)
#define FRS_ID_META_RAW_GYROSCOPE                (0xE305)
#define FRS_ID_META_GYROSCOPE_CALIBRATED         (0xE306)
#define FRS_ID_META_GYROSCOPE_UNCALIBRATED       (0xE307)
#define FRS_ID_META_RAW_MAGNETOMETER             (0xE308)
#define FRS_ID_META_MAGNETIC_FIELD_CALIBRATED    (0xE309)
#define FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED  (0xE30A)
#define FRS_ID_META_ROTATION_VECTOR              (0xE30B)
#define FRS_ID_META_GAME_ROTATION_VECTOR         (0xE30C)
#define FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR  (0xE30D)
#define FRS_ID_META_PRESSURE                     (0xE30E)
#define FRS_ID_META_AMBIENT_LIGHT                (0xE30F)
#define FRS_ID_META_HUMIDITY                     (0xE310)
#define FRS_ID_META_PROXIMITY                    (0xE311)
#define FRS_ID_META_TEMPERATURE                  (0xE312)
#define FRS_ID_META_TAP_DETECTOR                 (0xE313)
#define FRS_ID_META_STEP_DETECTOR                (0xE314)
#define FRS_ID_META_STEP_COUNTER                 (0xE315)
#define FRS_ID_META_SIGNIFICANT_MOTION           (0xE316)
#define FRS_ID_META_STABILITY_CLASSIFIER         (0xE317)
#define FRS_ID_META_SHAKE_DETECTOR               (0xE318)
#define FRS_ID_META_FLIP_DETECTOR                (0xE319)
#define FRS_ID_META_PICKUP_DETECTOR              (0xE31A)
#define FRS_ID_META_STABILITY_DETECTOR           (0xE31B)
#define FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER (0xE31C)
#define FRS_ID_META_SLEEP_DETECTOR               (0xE31D)
#define FRS_ID_META_TILT_DETECTOR                (0xE31E)
#define FRS_ID_META_POCKET_DETECTOR              (0xE31F)
#define FRS_ID_META_CIRCLE_DETECTOR              (0xE320)
#define FRS_ID_META_HEART_RATE_MONITOR           (0xE321)
#define FRS_ID_META_ARVR_STABILIZED_RV           (0xE322)
#define FRS_ID_META_ARVR_STABILIZED_GRV          (0xE323)
#define FRS_ID_META_GYRO_INTEGRATED_RV           (0xE324)
