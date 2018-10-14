//gcc BNO080.c -o BNO080 -lm -lbcm2835


// Copyright (c) 2017,2018 Peter Budd. All rights reserved
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

#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <linux/types.h>
#include <sys/types.h>
#include <linux/spi/spidev.h>
#include <dirent.h>
#include "BNO080.h"
#include <bcm2835.h>

#ifndef NULL
#define NULL 0
#endif

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 
#define INTPIN 23
#define RESETPIN 24
#define WAKPS0PIN 25
#define QP(n) (1.0 / (1 << n))  
#define ODR 50

FILE *fd_write_out;

struct {
    unsigned char buffer[MAX_PACKET_SIZE];
    uint8_t sequence;
    uint16_t dataLength;
    uint8_t channel;
} spiRead;

struct {
    unsigned char buffer[256];
} spiWrite;

struct {
    float rateBuffer[16];
    float accnBuffer[16];
    int head;
    int tail;
    unsigned int available;
    unsigned int status;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
    float lastRate;
} gyroData={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0, .lastRate=0 };

struct {
    float angleBuffer[16];
    float rateBuffer[16];
    float accnBuffer[16];
    int head;
    int tail;
    unsigned int available;
    float lastReportedAngle;
    float direction;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
    float lastRate;
} gyroIntegratedRotationVectorData ={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };


struct {
    float angleBuffer[16];
    int head;
    int tail;
    unsigned int available;
    float lastReportedAngle;
    float direction;
    unsigned int status;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} gameRotationVectorData ={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };

struct {
    float XBuffer[16];
    float YBuffer[16];
    float ZBuffer[16];
    int head;
    int tail;
    unsigned int available;
    unsigned int status;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} linearAccelerometerData={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };

struct {
    unsigned int status;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} stabilityData={ .lastReportTimestamp=0 };

struct timeval current;

char READ_OUTBUF[1500];

uint32_t FRS_READ_BUFFER[64];

uint8_t SEQUENCENUMBER[7];
unsigned int LOOPSLEEP = 2000;  // in microseconds
unsigned int LOOPCOUNT = 0;
char FILENAME[50];
int RUNNING = 0;


volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

int main(int argc, char const *argv[]){
    char linein[50];
    char command[6];
    char details[42];
    char oscommand[90];
    float shutdowncount = 0.0;
    FILE *shutdowncheck;

    for(uint32_t i=0; i<sizeof(SEQUENCENUMBER); i++) SEQUENCENUMBER[i] = 0;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
    
    setupBNO080();
    setupStabilityClassifierFrs(1.5);
    configureBNO080();
    
//    reorient(0,0,0,0);
//    clearPersistentTare();
//    reorient(0.0,0.0,-1.0,0.0);

    startBNO080(ODR);
   
    while(!sig_exit){
        usleep(LOOPSLEEP);
        LOOPCOUNT += 1;
        while(bcm2835_gpio_lev(INTPIN) == 0) handleEvent();
        if(LOOPCOUNT == 5000) startRun();
    }
    startBNO080(0);
    collectPacket();// collect the three feature status reports and dump them
    collectPacket();
    collectPacket();
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}

int startRun(void){

// reset fifos
    gameRotationVectorData.head = 1;
    gameRotationVectorData.tail = 0;
    gameRotationVectorData.available = 1;  // one is added by this routine
//    gyroIntegratedRotationVectorData.head = 1;
//    gyroIntegratedRotationVectorData.tail = 0;
//    gyroIntegratedRotationVectorData.available = 1;
    linearAccelerometerData.head = 0;
    linearAccelerometerData.tail = 0;
    linearAccelerometerData.available = 0;
    gyroData.head = 0;
    gyroData.tail = 0;
    gyroData.available = 0;
    
    if(stabilityData.status != 1 && stabilityData.status != 2) {
        printf("Bell is moving\n");
        return(0);
    }
    if(abs(gameRotationVectorData.lastReportedAngle) < 160 || abs(gameRotationVectorData.lastReportedAngle) > 178){
        printf("Bell not at stand\n");
        return(0);
    }
    if(gameRotationVectorData.lastReportedAngle < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        gameRotationVectorData.direction = -1;
        gameRotationVectorData.angleBuffer[0] = -180.0-gameRotationVectorData.lastReportedAngle;
    } else {
        gameRotationVectorData.direction = +1;
        gameRotationVectorData.angleBuffer[0] = gameRotationVectorData.lastReportedAngle-180.0;
    }
    RUNNING = 1;
    return(1);
}

void handleEvent(void){
    if(!collectPacket()) return;
    if(spiRead.dataLength != 0) {
        parseEvent();
        alignFIFOs();
        pushData();
    }
}

void alignFIFOs(void){
    if(gyroData.available >= 3){
        if ((gyroData.available - 3) > gameRotationVectorData.available ||
            (gyroData.available - 3) > linearAccelerometerData.available){
                gyroData.tail = (gyroData.tail + 1) & 0x0F; // ditch a sample don't forget that there is a gyro accelerometer reading here too
                gyroData.available -= 1;
                printf("Ditched rate sample\n");
        }
    }

    if(gameRotationVectorData.available >= 3){
        if ((gameRotationVectorData.available - 3) > gyroData.available ||
            (gameRotationVectorData.available - 3) > linearAccelerometerData.available){
                gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) & 0x0F; // ditch a sample
                gameRotationVectorData.available -= 1;
                printf("Ditched angle sample\n");
        }
    }
    
    if(linearAccelerometerData.available >= 3){
        if ((linearAccelerometerData.available - 3) > gyroData.available ||
            (linearAccelerometerData.available - 3) > gameRotationVectorData.available){
                linearAccelerometerData.tail = (linearAccelerometerData.tail + 1) & 0x0F; // ditch a sample
                linearAccelerometerData.available -= 1;
                printf("Ditched linacc sample\n");
        }
    }
}

void pushData(void){
    uint16_t statuses = gyroData.status | (linearAccelerometerData.status << 4) | (gameRotationVectorData.status << 8);

//    if (gyroIntegratedRotationVectorData.available >=10){

    if (gyroData.available >=10 && linearAccelerometerData.available >=10 && gameRotationVectorData.available >=10){
//        printf("Gyro: %d, LinAcc: %d, Angle: %d",gyroData.available, linearAccelerometerData.available, gameRotationVectorData.available);
        for(int counter = 0; counter < 10; ++counter){
//            printf("A:%+07.1f,R:%+07.1f,C:%+07.1f\n", gyroIntegratedRotationVectorData.angleBuffer[gyroIntegratedRotationVectorData.tail], gyroIntegratedRotationVectorData.rateBuffer[gyroIntegratedRotationVectorData.tail], gyroIntegratedRotationVectorData.accnBuffer[gyroIntegratedRotationVectorData.tail]);
            printf("A:%+07.1f,R:%+07.1f,C:%+07.1f,LA:%+07.1f,S:%04X\n", gameRotationVectorData.angleBuffer[gameRotationVectorData.tail], gyroData.rateBuffer[gyroData.tail], gyroData.accnBuffer[gyroData.tail], 25*linearAccelerometerData.YBuffer[linearAccelerometerData.tail],statuses);
//            gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData + 1) & 0x0F
            gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) & 0x0F;
            gyroData.tail = (gyroData.tail + 1) & 0x0F;
            linearAccelerometerData.tail = (linearAccelerometerData.tail + 1) & 0x0F;
        }
//        gyroIntegratedRotationVectorData.available -=10;
        gameRotationVectorData.available -= 10;
        gyroData.available -= 10;
        linearAccelerometerData.available -= 10;
    }
}

void parseEvent(void){
    if(spiRead.channel == CHANNEL_REPORTS) {
        switch(spiRead.buffer[5]) {
//            case SENSOR_REPORTID_ROTATION_VECTOR:                       parseRotationVector(); break;
            case SENSOR_REPORTID_GAME_ROTATION_VECTOR:                  parseGameRotationVector(); break;
            case SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR:       parseStabilisedRotationVector(); break;
            case SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR:  parseStabilisedGameRotationVector(); break;
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:                  parseCalibratedGyroscope(); break;
//            case SENSOR_REPORTID_ACCELEROMETER:                         parseAccelerometer(); break;
            case SENSOR_REPORTID_LINEAR_ACCELERATION:                   parseLinearAccelerometer(); break;
            case SENSOR_REPORTID_STABILITY_CLASSIFIER:                  parseStability(); break;
            default: printf("Unhandled report event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);

        }
    }

    if(spiRead.channel == CHANNEL_CONTROL) {
        switch(spiRead.buffer[0]) {
            case SHTP_REPORT_COMMAND_RESPONSE:      reportCommandResponse(); break;
            case SHTP_REPORT_GET_FEATURE_RESPONSE:  reportFeatureResponse(); break;
            case SHTP_REPORT_PRODUCT_ID_RESPONSE:   break; // ignore this
            case SHTP_REPORT_FRS_WRITE_RESPONSE:    break; // sometimes get a few of these for each write - ignore them
            default: printf("Unhandled control event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
        }
    }
    
    if(spiRead.channel == CHANNEL_GYRO) parseGyroIntegratedRotationVector();
}

void reportFeatureResponse(void){
    float reportInterval = (float)((spiRead.buffer[8 ] << 24) + (spiRead.buffer[7 ] << 16) + (spiRead.buffer[6 ] << 8) + spiRead.buffer[5])/1000.0;
    float batchInterval =  (float)((spiRead.buffer[12] << 24) + (spiRead.buffer[11] << 16) + (spiRead.buffer[10] << 8) + spiRead.buffer[9])/1000.0;
    switch(spiRead.buffer[1]){
//        case SENSOR_REPORTID_ACCELEROMETER:         break;
        case SENSOR_REPORTID_GAME_ROTATION_VECTOR:  gameRotationVectorData.reportInterval = reportInterval; break;
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:  gyroData.reportInterval = reportInterval; break;
        case SENSOR_REPORTID_LINEAR_ACCELERATION:   linearAccelerometerData.reportInterval = reportInterval; break;
        case SENSOR_REPORTID_STABILITY_CLASSIFIER:  stabilityData.reportInterval = reportInterval; break;
        case SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR: gyroIntegratedRotationVectorData.reportInterval = reportInterval; break;
        default: printf("Unhandled FR event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
    }
}

void reportCommandResponse(void){
    switch(spiRead.buffer[2]){
        case 0x84: printf("Error: device has reinitialised!\n"); break;  // bit 7 set indicates autonomous response 0x04 = Initialisation.  Sometimes caused by too much data being pushed.
        case 0x06: printf("Save DCD.  Success: %d", spiRead.buffer[5] ); break;
        case 0xFD: printf("FRS Write response\n"); break;  // is this correct???
        case COMMAND_ME_CALIBRATE: break;
        default: printf("Unhandled CR event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
    }
}

void parseGyroIntegratedRotationVector(void){

    double Qx = (double)((spiRead.buffer[1] << 8) + spiRead.buffer[0]);
    double Qy = (double)((spiRead.buffer[3] << 8) + spiRead.buffer[2]);
    double Qz = (double)((spiRead.buffer[5] << 8) + spiRead.buffer[4]);
    double Qw = (double)((spiRead.buffer[7] << 8) + spiRead.buffer[6]);
    
    if(Qx >= 0x8000) Qx -= 0x10000;
    if(Qy >= 0x8000) Qy -= 0x10000;
    if(Qz >= 0x8000) Qz -= 0x10000;
    if(Qw >= 0x8000) Qw -= 0x10000;

    Qx *= QP(14);
    Qy *= QP(14);
    Qz *= QP(14);
    Qw *= QP(14);

    float yaw   =  atan2((Qx * Qy + Qw * Qz), ((Qw * Qw + Qx * Qx) - 0.5f)) * RADIANS_TO_DEGREES_MULTIPLIER;   
    float pitch = -asin(2.0f * (Qx * Qz - Qw * Qy))* RADIANS_TO_DEGREES_MULTIPLIER;
    float roll  =  atan2((Qw * Qx + Qy * Qz), ((Qw * Qw + Qz * Qz) - 0.5f))* RADIANS_TO_DEGREES_MULTIPLIER;

    float Gx = (float)((spiRead.buffer[9 ] << 8) + spiRead.buffer[8]);
    float Gy = (float)((spiRead.buffer[11] << 8) + spiRead.buffer[10]);
    float Gz = (float)((spiRead.buffer[13] << 8) + spiRead.buffer[12]);
    
    if(Gx >= 0x8000) Gx -= 0x10000;
    if(Gy >= 0x8000) Gy -= 0x10000;
    if(Gz >= 0x8000) Gz -= 0x10000;

    Gx *= QP(10) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gy *= QP(10) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gz *= QP(10) * RADIANS_TO_DEGREES_MULTIPLIER;

    gettimeofday(&current, NULL);
    gyroIntegratedRotationVectorData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);
 
    float rolldiff = roll - gyroIntegratedRotationVectorData.lastReportedAngle;
    if(rolldiff > 250.0) {
        rolldiff -= 360.0;
    } else if (rolldiff < -250.0) {
        rolldiff += 360.0;
    }
 
    gyroIntegratedRotationVectorData.lastReportedAngle = roll;

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gyroIntegratedRotationVectorData.available == 15) {
        printf("Rate buffer full.  Ditching oldest sample\n");
        gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData.tail + 1) & 0x0F;
        gyroIntegratedRotationVectorData.available -= 1;
    }
    
        
    gyroIntegratedRotationVectorData.rateBuffer[gyroIntegratedRotationVectorData.head] = Gx;
    gyroIntegratedRotationVectorData.accnBuffer[gyroIntegratedRotationVectorData.head] = Gx-gyroIntegratedRotationVectorData.lastRate;
    gyroIntegratedRotationVectorData.lastRate = Gx;
    gyroIntegratedRotationVectorData.angleBuffer[gyroIntegratedRotationVectorData.head] = gameRotationVectorData.angleBuffer[(gyroIntegratedRotationVectorData.head -1) & 0x0F] + gyroIntegratedRotationVectorData.direction*rolldiff;
    gyroIntegratedRotationVectorData.head = (gyroIntegratedRotationVectorData.head + 1) & 0x0F;
    gyroIntegratedRotationVectorData.available += 1;
}

void parseGameRotationVector(void){
    uint8_t status = spiRead.buffer[5 + 2] & 0x03;
   
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);
    double Qx = (double)((spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4]);
    double Qy = (double)((spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6]);
    double Qz = (double)((spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8]);
    double Qw = (double)((spiRead.buffer[5 + 11] << 8) + spiRead.buffer[5 + 10]);
    
    if(Qx >= 0x8000) Qx -= 0x10000;
    if(Qy >= 0x8000) Qy -= 0x10000;
    if(Qz >= 0x8000) Qz -= 0x10000;
    if(Qw >= 0x8000) Qw -= 0x10000;

    Qx *= QP(14);
    Qy *= QP(14);
    Qz *= QP(14);
    Qw *= QP(14);

    // remap axes
//        Qw = Qw * Q_remap.w - Qx * Q_remap.x - Qy * Q_remap.y - Qz * Q_remap.z; 
//        Qx = Qw * Q_remap.x + Qx * Q_remap.w + Qy * Q_remap.z - Qz * Q_remap.y; 
//        Qy = Qw * Q_remap.y - Qx * Q_remap.z + Qy * Q_remap.w + Qz * Q_remap.x;
//        Qz = Qw * Q_remap.z + Qx * Q_remap.y - Qy * Q_remap.x + Qz * Q_remap.w;

// https://forums.adafruit.com/viewtopic.php?t=131484
// https://mbientlab.com/community/discussion/2036/taring-quaternion-attitude
/*    if (torn < 100) {
        torn += 1;
        Q_tare.x = Qx;  // note that normally the conjugate is calculated the functions below alreasy have the sign swaps
        Q_tare.y = Qy;
        Q_tare.z = Qz;
        Q_tare.w = Qw;
        double magnit = sqrt(Q_tare.w * Q_tare.w + Q_tare.x * Q_tare.x + Q_tare.y * Q_tare.y + Q_tare.z * Q_tare.z);
        Q_tare.w /= magnit;
        Q_tare.x /= magnit;
        Q_tare.y /= magnit;
        Q_tare.z /= magnit;
    } else {
        Qw =  Qw * Q_tare.w + Qx * Q_tare.x + Qy * Q_tare.y + Qz * Q_tare.z; 
        Qx = -Qw * Q_tare.x + Qx * Q_tare.w - Qy * Q_tare.z + Qz * Q_tare.y; 
        Qy = -Qw * Q_tare.y + Qx * Q_tare.z + Qy * Q_tare.w - Qz * Q_tare.x;
        Qz = -Qw * Q_tare.z - Qx * Q_tare.y + Qy * Q_tare.x + Qz * Q_tare.w;
    }
//    float norm = sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz);

*/

    float yaw   =  atan2((Qx * Qy + Qw * Qz), ((Qw * Qw + Qx * Qx) - 0.5f)) * RADIANS_TO_DEGREES_MULTIPLIER;   
    float pitch = -asin(2.0f * (Qx * Qz - Qw * Qy))* RADIANS_TO_DEGREES_MULTIPLIER;
    float roll  =  atan2((Qw * Qx + Qy * Qz), ((Qw * Qw + Qz * Qz) - 0.5f))* RADIANS_TO_DEGREES_MULTIPLIER;


    gameRotationVectorData.status = status;
    gettimeofday(&current, NULL);
    gameRotationVectorData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    float rolldiff = roll - gameRotationVectorData.lastReportedAngle;
    if(rolldiff > 250.0) {
        rolldiff -= 360.0;
    } else if (rolldiff < -250.0) {
        rolldiff += 360.0;
    }    
    gameRotationVectorData.lastReportedAngle = roll;

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gameRotationVectorData.available == 15) {
        printf("Angle buffer full.  Ditching oldest sample\n");
        gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) & 0x0F;
        gameRotationVectorData.available -= 1;
    }

    
    gameRotationVectorData.angleBuffer[gameRotationVectorData.head] =  gameRotationVectorData.angleBuffer[(gameRotationVectorData.head -1) & 0x0F] + gameRotationVectorData.direction*rolldiff;
    gameRotationVectorData.head = (gameRotationVectorData.head + 1) & 0x0F;
    gameRotationVectorData.available += 1;

//    printf("Game Rotation Vector.  S: %d R:%+07.1f, P:%+07.1f, Y:%+07.1f, Sequence: %02X, Buff: %d\n", status, roll, pitch, yaw, spiRead.sequence, gameRotationVectorData.available);
}

void parseLinearAccelerometer(void){
    uint8_t status = spiRead.buffer[5 + 2] & 0x03;
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);
    float Ax = (float)((spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4]);
    float Ay = (float)((spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6]);
    float Az = (float)((spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8]);
    
    if(Ax >= 0x8000) Ax -= 0x10000;
    if(Ay >= 0x8000) Ay -= 0x10000;
    if(Az >= 0x8000) Az -= 0x10000;

    Ax *= QP(8);
    Ay *= QP(8);
    Az *= QP(8);

    linearAccelerometerData.status = status;
    gettimeofday(&current, NULL);
    linearAccelerometerData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (linearAccelerometerData.available == 15) {
        printf("Lin Acc buffer full.  Ditching oldest sample\n");
        linearAccelerometerData.tail = (linearAccelerometerData.tail + 1) & 0x0F;
        linearAccelerometerData.available -= 1;
    }
    linearAccelerometerData.XBuffer[linearAccelerometerData.head] = Ax;
    linearAccelerometerData.YBuffer[linearAccelerometerData.head] = Ay;
    linearAccelerometerData.ZBuffer[linearAccelerometerData.head] = Az;
    linearAccelerometerData.head = (linearAccelerometerData.head + 1) & 0x0F;
    linearAccelerometerData.available += 1;
    
//    printf("Lin. Accel.  S: %d X:%+07.2f, Y:%+07.2f, Z:%+07.2f, Sequence: %02X, Buff: %d\n", status, Ax, Ay, Az, spiRead.sequence, CB_linaccn.available);
}

void parseCalibratedGyroscope(void){
    uint8_t status = spiRead.buffer[5 + 2] & 0x03;
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);
    float Gx = (float)((spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4]);
    float Gy = (float)((spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6]);
    float Gz = (float)((spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8]);
    
    if(Gx >= 0x8000) Gx -= 0x10000;
    if(Gy >= 0x8000) Gy -= 0x10000;
    if(Gz >= 0x8000) Gz -= 0x10000;

    Gx *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gy *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gz *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;

    gyroData.status = status;
    gettimeofday(&current, NULL);
    gyroData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gyroData.available == 15) {
        printf("Rate buffer full.  Ditching oldest sample\n");
        gyroData.tail = (gyroData.tail + 1) & 0x0F;
        gyroData.available -= 1;
    }
    gyroData.rateBuffer[gyroData.head] = Gx;
    gyroData.accnBuffer[gyroData.head] = Gx-gyroData.lastRate;
    gyroData.lastRate = Gx;
    gyroData.head = (gyroData.head + 1) & 0x0F;
    gyroData.available += 1;
}

void parseStability(void){
    stabilityData.status = spiRead.buffer[5+4];
    gettimeofday(&current, NULL);
    stabilityData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);
}

void parseStabilisedRotationVector(void){
    printf("Not yet\n");
}

void parseStabilisedGameRotationVector(void){
    printf("Not yet\n");
}

void clearPersistentTare(void){
    writeFrsWord(SYSTEM_ORIENTATION,0,0);
    writeFrsWord(SYSTEM_ORIENTATION,1,0);
    writeFrsWord(SYSTEM_ORIENTATION,2,0);
    writeFrsWord(SYSTEM_ORIENTATION,3,0);
//    eraseFrsRecord(SYSTEM_ORIENTATION);   
}

void tareBNO080(void){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; // 0x05=rotation vector, 0x08=game rotation vector, 0x28=ARVR-stabilized rotation vector, 0x29=ARVR-stabilised game rotation vector
    spiWrite.buffer[2] = 0x03; // Tare command
    spiWrite.buffer[3] = 0x00; // perform Tare now
    spiWrite.buffer[4] = 0x07; // all axes
    spiWrite.buffer[5] = 0x01; // use gaming rotation vector
    spiWrite.buffer[6] = 0x00; // reserved
    spiWrite.buffer[7] = 0x00;
    spiWrite.buffer[8] = 0x00;
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    sendPacket(CHANNEL_CONTROL, 12);
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; // 0x05=rotation vector, 0x08=game rotation vector, 0x28=ARVR-stabilized rotation vector, 0x29=ARVR-stabilised game rotation vector
    spiWrite.buffer[2] = 0x03; // Tare command
    spiWrite.buffer[3] = 0x01; // persist tare
    spiWrite.buffer[4] = 0x00; 
    spiWrite.buffer[5] = 0x00;
    spiWrite.buffer[6] = 0x00;
    spiWrite.buffer[7] = 0x00;
    spiWrite.buffer[8] = 0x00;
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    sendPacket(CHANNEL_CONTROL, 12);
}

void calibrationSetup(char accel, char gyro, char mag){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; // sequence number
    spiWrite.buffer[2] = COMMAND_ME_CALIBRATE; // ME calibration
    spiWrite.buffer[3] = accel; // accel 1=enabled 0=disabled
    spiWrite.buffer[4] = gyro; // gyro 1=enabled 0=disabled
    spiWrite.buffer[5] = mag; // mag 1=enabled 0=disabled
    spiWrite.buffer[6] = 0x00; 
    spiWrite.buffer[7] = 0x00; // planar calibration
    spiWrite.buffer[8] = 0x00; // reserved
    spiWrite.buffer[9] = 0x00; // reserved
    spiWrite.buffer[10]= 0x00; // reserved
    spiWrite.buffer[11]= 0x00; // reserved
    sendPacket(CHANNEL_CONTROL, 12);
}

void saveCalibration(void){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; 
    spiWrite.buffer[2] = 0x06; // Save DCD
    spiWrite.buffer[3] = 0x00; // reserved
    spiWrite.buffer[4] = 0x00;  
    spiWrite.buffer[5] = 0x00; 
    spiWrite.buffer[6] = 0x00; 
    spiWrite.buffer[7] = 0x00;
    spiWrite.buffer[8] = 0x00;
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    sendPacket(CHANNEL_CONTROL, 12);
}

int reorient(float w, float x, float y, float z){
    uint16_t W = (int16_t)(w/QP(14));
    uint16_t X = (int16_t)(x/QP(14));
    uint16_t Y = (int16_t)(y/QP(14));
    uint16_t Z = (int16_t)(z/QP(14));

    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++;
    spiWrite.buffer[2] = 0x03; // Tare command
    spiWrite.buffer[3] = 0x02; // reorient
    spiWrite.buffer[4] = X & 0x00FF; // X LSB
    spiWrite.buffer[5] = X >> 8; // X MSB
    spiWrite.buffer[6] = Y & 0x00FF; // Y LSB
    spiWrite.buffer[7] = Y >> 8; // Y MSB
    spiWrite.buffer[8] = Z & 0x00FF; // Z LSB
    spiWrite.buffer[9] = Z >> 8; // Z MSB
    spiWrite.buffer[10]= W & 0x00FF; // W LSB
    spiWrite.buffer[11]= W >> 8; // W MSB
    return(sendPacket(CHANNEL_CONTROL, 12));
}

int configureFeatureReport(uint8_t report, uint32_t reportPeriod){
    spiWrite.buffer[0] = SHTP_REPORT_SET_FEATURE_COMMAND; // set feature
    spiWrite.buffer[1] = report;
    spiWrite.buffer[2] = 0x00; // feature flags
    spiWrite.buffer[3] = 0x00; // change sensitivity LSB
    spiWrite.buffer[4] = 0x00; // change sensitivity MSB
    spiWrite.buffer[5] =  reportPeriod & 0x000000FF;       // report interval (LSB)
    spiWrite.buffer[6] = (reportPeriod & 0x0000FF00) >> 8;
    spiWrite.buffer[7] = (reportPeriod & 0x00FF0000) >> 16;
    spiWrite.buffer[8] = (reportPeriod & 0xFF000000) >> 24;// report interval (MSB)
    spiWrite.buffer[9] = 0x00;  // batch interval (LSB)
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    spiWrite.buffer[12]= 0x00;  // batch interval (MSB)
    spiWrite.buffer[13]= 0x00;  // sensor specific config word (LSB)
    spiWrite.buffer[14]= 0x00;
    spiWrite.buffer[15]= 0x00;
    spiWrite.buffer[16]= 0x00;  // sensor specific config word (MSB)
    return(sendPacket(CHANNEL_CONTROL, 17));
}

int readFrsRecord(uint16_t recordType){
    spiWrite.buffer[0] = SHTP_REPORT_FRS_READ_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = 0x00; 
    spiWrite.buffer[3] = 0x00; 
    spiWrite.buffer[4] = (char)(recordType & 0x00FF);
    spiWrite.buffer[5] = (char)(recordType >> 8);
    spiWrite.buffer[6] = 0x00; 
    spiWrite.buffer[7] = 0x00; 
    sendPacket(CHANNEL_CONTROL, 8);

    int timeout = 0;
    int counter = 0;
    while(timeout < 1000){
        timeout +=1;
        usleep(50);
        while(bcm2835_gpio_lev(INTPIN) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_READ_RESPONSE) {parseEvent(); break;}
            if(((spiRead.buffer[1] & 0x0F) == 1) || ((spiRead.buffer[1] & 0x0F) == 2) || ((spiRead.buffer[1] & 0x0F) == 4) || \
                ((spiRead.buffer[1] & 0x0F) == 5) || ((spiRead.buffer[1] & 0x0F) == 8)) {
                printf("FRS Read error, %02X\n",spiRead.buffer[1] & 0x0F);
                return(-1);
            }
            FRS_READ_BUFFER[counter] = (uint32_t)spiRead.buffer[7] << 24 | (uint32_t)spiRead.buffer[6] << 16 | (uint32_t)spiRead.buffer[5] << 8 | (uint32_t)spiRead.buffer[4];
            counter += 1;
            FRS_READ_BUFFER[counter] = (uint32_t)spiRead.buffer[11] << 24 | (uint32_t)spiRead.buffer[10] << 16 | (uint32_t)spiRead.buffer[9] << 8 | (uint32_t)spiRead.buffer[8];
            counter += 1;
            if(((spiRead.buffer[1] & 0x0F) == 3) || ((spiRead.buffer[1] & 0x0F) == 6) || ((spiRead.buffer[1] & 0x0F) == 7)) return(counter);
        }
    }
    printf("FRS Read timeout");
    return(-1);
}

int eraseFrsRecord(uint16_t recordType){
    spiWrite.buffer[0] = SHTP_REPORT_FRS_WRITE_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = 0x00;//length lsb 
    spiWrite.buffer[3] = 0x00;//length msb
    spiWrite.buffer[4] = recordType & 0x00FF;
    spiWrite.buffer[5] = recordType >> 8;
    if (!sendPacket(CHANNEL_CONTROL, 6)) return(0);
    return(1);
}


int writeFrsWord(uint16_t recordType, uint32_t offset, uint32_t data){
    spiWrite.buffer[0] = SHTP_REPORT_FRS_WRITE_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = 0x01;//length lsb 
    spiWrite.buffer[3] = 0x00;//length msb
    spiWrite.buffer[4] = recordType & 0x00FF;
    spiWrite.buffer[5] = recordType >> 8;
    if (!sendPacket(CHANNEL_CONTROL, 6)) return(0);

    spiWrite.buffer[0] = SHTP_REPORT_FRS_WRITE_DATA_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = offset & 0x00FF;  //offset lsb
    spiWrite.buffer[3] = offset >> 8;//offset msb
    spiWrite.buffer[4] =  data & 0x000000FF;       // word 1 (LSB)
    spiWrite.buffer[5] = (data & 0x0000FF00) >> 8;
    spiWrite.buffer[6] = (data & 0x00FF0000) >> 16;
    spiWrite.buffer[7] = data >> 24; // word 1 (MSB)
    spiWrite.buffer[8] = 0x00;  // word 2 lsb
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    if(!sendPacket(CHANNEL_CONTROL, 12)) return(0);

    for(int i = 0; i<1000; i++){
        usleep(100);
        while(bcm2835_gpio_lev(INTPIN) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_WRITE_RESPONSE) {parseEvent(); continue;}
            if(spiRead.buffer[1] != 3) {
//                printf("FRS Write, %02X\n",spiRead.buffer[1]);
                continue;
            } else {
//                printf("Write OK\n");
                return(1);
            }
        }
    }
    printf("FRS Write timeout\n");
    return(0);
}


int readFrsWord(uint16_t recordType, uint32_t offset, uint32_t* result){
    spiWrite.buffer[0] = SHTP_REPORT_FRS_READ_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = offset & 0x00FF; 
    spiWrite.buffer[3] = offset >> 8; 
    spiWrite.buffer[4] = recordType & 0x00FF;
    spiWrite.buffer[5] = recordType >> 8;
    spiWrite.buffer[6] = 0x01; 
    spiWrite.buffer[7] = 0x00; 
    if (!sendPacket(CHANNEL_CONTROL, 6)) return(0);

    int timeout = 0;
    while(timeout < 1000){
        timeout +=1;
        usleep(50);
        while(bcm2835_gpio_lev(INTPIN) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_READ_RESPONSE) {parseEvent(); continue;}
            if(((spiRead.buffer[1] & 0x0F) == 1) || ((spiRead.buffer[1] & 0x0F) == 2) || ((spiRead.buffer[1] & 0x0F) == 4) || \
                ((spiRead.buffer[1] & 0x0F) == 5) || ((spiRead.buffer[1] & 0x0F) == 8)) {
                printf("FRS Read error, %02X\n",spiRead.buffer[1] & 0x0F);
                return(0);
            }
            *result = ((uint32_t)spiRead.buffer[7] << 24) | ((uint32_t)spiRead.buffer[6] << 16) | ((uint32_t)spiRead.buffer[5] << 8) | (uint32_t)spiRead.buffer[4];
            if(((spiRead.buffer[1] & 0x0F) == 3) || ((spiRead.buffer[1] & 0x0F) == 6) || ((spiRead.buffer[1] & 0x0F) == 7)) return(1);
        }
    }
    printf("FRS Read timeout\n");
    return(0);
}

void startBNO080(uint32_t dataRate){ // set datarate to zero to stop sensors
// setup rotation vector quaternion report
    uint32_t odrPeriodMicrosecs = 0;
    if(dataRate != 0) odrPeriodMicrosecs = 1000000/dataRate; 

// disable calibration (we are about to start)
    calibrationSetup(0,0,0);

    configureFeatureReport(SENSOR_REPORTID_GAME_ROTATION_VECTOR, odrPeriodMicrosecs);
    gameRotationVectorData.requestedInterval=odrPeriodMicrosecs;
    gameRotationVectorData.direction=0;
    
    configureFeatureReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, odrPeriodMicrosecs);
    gyroData.requestedInterval=odrPeriodMicrosecs;

    configureFeatureReport(SENSOR_REPORTID_LINEAR_ACCELERATION, odrPeriodMicrosecs);
    linearAccelerometerData.requestedInterval=odrPeriodMicrosecs;
    
    configureFeatureReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, 500000);// one report per second
    stabilityData.requestedInterval=500000;
    
//    configureFeatureReport(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, odrPeriodMicrosecs);
//    gyroIntegratedRotationVectorData.requestedInterval=odrPeriodMicrosecs;
//    gyroIntegratedRotationVectorData.direction = 0;
}

void configureBNO080(void){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; 
    spiWrite.buffer[2] = 0x09; // Periodic save DCD (calibration data)
    spiWrite.buffer[3] = 0x00; // 0x00=disable 0x01=enable
    spiWrite.buffer[4] = 0x00;  
    spiWrite.buffer[5] = 0x00; 
    spiWrite.buffer[6] = 0x00; 
    spiWrite.buffer[7] = 0x00;
    spiWrite.buffer[8] = 0x00;
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    sendPacket(CHANNEL_CONTROL, 12);

}

void setupStabilityClassifierFrs(float threshold){
    threshold /= QP(24);
    uint32_t result;
    readFrsWord(STABILITY_DETECTOR_CONFIG,0,&result);
    if (threshold != result) writeFrsWord(STABILITY_DETECTOR_CONFIG,0,threshold);

// for some reason the duration word does not seem to want to take
//    readFrsWord(STABILITY_DETECTOR_CONFIG,1,&result);
//    if (duration != result) writeFrsWord(STABILITY_DETECTOR_CONFIG,1,duration);
}

void setupBNO080(void){
    if (!bcm2835_init()){
        printf("Unable to inititalise bcm2835\n");
        exit(1);
    }
    bcm2835_gpio_fsel(RESETPIN, BCM2835_GPIO_FSEL_OUTP); // reset
    bcm2835_gpio_set(RESETPIN);
    bcm2835_gpio_fsel(WAKPS0PIN, BCM2835_GPIO_FSEL_OUTP); // WAKPS0
    bcm2835_gpio_set(WAKPS0PIN);
    bcm2835_gpio_fsel(INTPIN, BCM2835_GPIO_FSEL_INPT); // INT
    bcm2835_gpio_set_pud(INTPIN, BCM2835_GPIO_PUD_UP);
    
// setup SPI
    if (!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. \n");
      exit(1);
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    bcm2835_spi_set_speed_hz(3000000);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

//    reset device
    bcm2835_gpio_clr(RESETPIN);
    usleep(20000);
    bcm2835_gpio_set(RESETPIN);
    int waitCount = 0;
    while ((bcm2835_gpio_lev(INTPIN) == 1) && (waitCount < 1000)){ // should only be called when int is low so not really needed
        usleep(500);
        waitCount += 1;
    }
    if(waitCount == 1000)  {printf("Device did not wake on reset\n"); exit(1);}
    if(!collectPacket()) {printf("Reset packet not received\n"); exit(1);}

    usleep(5000);
    if(!collectPacket()) {printf("Unsolicited packet not received\n"); exit(1);}

    spiWrite.buffer[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    spiWrite.buffer[1] = 0; //Reserved
    if(!sendPacket(CHANNEL_CONTROL, 2)) {printf("Cannot send ID request\n"); exit(1);}

    if(!collectPacket()) {printf("Cannot receive ID response"); exit(1);}
    
    if(spiRead.buffer[0] != SHTP_REPORT_PRODUCT_ID_RESPONSE){
        printf("Cannot communicate with BNO080: got %02X DATALENGTH %04X \n",spiRead.buffer[0],spiRead.dataLength);
        exit(1);
    }
/*    usleep(20000);
    spiWrite.buffer[0] = 0x01; //reset
    printf("Sending reset request\n");
    sendPacket(CHANNEL_EXECUTABLE, 1);
    usleep(500000);*/

//    spiWrite.buffer[0] = 0x02; //sensors on
//    sendPacket(CHANNEL_EXECUTABLE, 1);
}

int32_t collectPacket(void){
    int waitCount = 0;
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;
    uint32_t RXCnt=0;
    uint32_t TXCnt=0;
    spiRead.dataLength = 0;
    spiRead.sequence = 0;
    spiRead.channel = 9;

    while ((bcm2835_gpio_lev(INTPIN) == 1) && (waitCount < 100)){ // should only be called when int is low so not really needed
        usleep(500);
        waitCount += 1;
    }
    if(waitCount == 100) return(0);  // timeout

    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);
    while((TXCnt < 4)||(RXCnt < 4)){
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))&&(TXCnt < 4 )){
            bcm2835_peri_write_nb(fifo, 0);
            TXCnt++;
        }
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < 4 )){
            spiRead.buffer[RXCnt] = bcm2835_peri_read_nb(fifo);
            RXCnt++;
        }
    }
    
    spiRead.dataLength = (uint16_t)spiRead.buffer[1] << 8 | (uint16_t)spiRead.buffer[0];
    spiRead.sequence = spiRead.buffer[3];
    spiRead.channel = spiRead.buffer[2];
    if(spiRead.dataLength == 0) return(-1); // nothing collected
//    if(spiRead.dataLength > 0x7FFF) printf("Continuation\n"); 
    spiRead.dataLength = spiRead.dataLength & ~(1 << 15); //Clear the MSbit - no idea what to do with a continuation (yuk)
    while((TXCnt < spiRead.dataLength)||(RXCnt < spiRead.dataLength)){
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))&&(TXCnt < spiRead.dataLength )){
            bcm2835_peri_write_nb(fifo, 0);
            TXCnt++;
        }
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < spiRead.dataLength )){
            spiRead.buffer[RXCnt - 4] = bcm2835_peri_read_nb(fifo); // -4 as we don't need to keep the header
            RXCnt++;
        }
    }
    
    while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE));
    bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);
    spiRead.dataLength -= 4;
    return(1);
}

int32_t sendPacket(uint32_t channelNumber, uint32_t dataLength){
    unsigned char header[4];
    int waitCount = 0;
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;
    uint32_t TXCnt = 0;
    uint32_t RXCnt = 0;
    spiRead.dataLength = 0;
    spiRead.sequence = 0;
    spiRead.channel = 9;

    bcm2835_gpio_clr(WAKPS0PIN);  // wake the thing up
    dataLength += 4;    
    header[0] = dataLength & 0xFF;
    header[1] = dataLength >> 8;
    header[2] = (char)channelNumber;
    header[3] = (char)SEQUENCENUMBER[channelNumber]++;
    usleep(2000);  // makes things much more reliable - thankfully we are only sending commands infrequently

    while ((bcm2835_gpio_lev(INTPIN) == 1) && (waitCount < 200)){
        usleep(500);
        waitCount += 1;
    }
    
    if(waitCount == 200) return(0);

    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);
    bcm2835_peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);

    // send header
    while((TXCnt < 4)||(RXCnt < 4)){
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))&&(TXCnt < 4 )){
            bcm2835_peri_write_nb(fifo, header[TXCnt]);
            TXCnt++;
        }
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < 4 )) {
            spiRead.buffer[RXCnt] = bcm2835_peri_read_nb(fifo);
            RXCnt++;
        }
    }
    // physically possible that data will be received at the same time so have a look
    spiRead.sequence = spiRead.buffer[3];
    spiRead.channel = spiRead.buffer[2];
    spiRead.dataLength = ((uint16_t)spiRead.buffer[1] << 8 | (uint16_t)spiRead.buffer[0]);
    spiRead.dataLength = spiRead.dataLength & ~(1 << 15); //Clear the MSbit

    bcm2835_gpio_set(WAKPS0PIN);  // probably OK to remove wake signal now

    // now send rest of TX packet
    while((TXCnt < dataLength)||(RXCnt < dataLength))
    {
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))&&(TXCnt < dataLength )){
            bcm2835_peri_write_nb(fifo, spiWrite.buffer[TXCnt-4]);
            TXCnt++;
        }
        while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < dataLength )){
            spiRead.buffer[RXCnt-4] = bcm2835_peri_read_nb(fifo);  // -4 as we don't need header
            RXCnt++;
        }
    }
    // check to see if we are reading a longer packet than sent by us

    if(spiRead.dataLength > dataLength){
        while((TXCnt < spiRead.dataLength)||(RXCnt < spiRead.dataLength)){
            while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_TXD))&&(TXCnt < spiRead.dataLength )){
                bcm2835_peri_write_nb(fifo, 0); // send padding zeros
                TXCnt++;
            }
            while(((bcm2835_peri_read(paddr) & BCM2835_SPI0_CS_RXD))&&( RXCnt < spiRead.dataLength )){
                spiRead.buffer[RXCnt-4] = bcm2835_peri_read_nb(fifo);  // -4 as we don't need header
                RXCnt++;
            }
        }
    }
    while (!(bcm2835_peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE));
    bcm2835_peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);

    if(spiRead.dataLength != 0) { spiRead.dataLength -= 4; parseEvent(); }

    return(1);

}