//gcc BNOgrabber.c -o BNOgrabber -lm -lbcm2835

/*
 * Copyright (c) 2017,2018 Peter Budd. All rights reserved
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
    * The above copyright notice and this permission notice shall be included in all copies or substantial
      portions of the Software.
    * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
      BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
      IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
      WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
      SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER,
      ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY
      FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW

* This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
* bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
* hardware is currently a Pi Zero running Arch Linux.

* I pulled snippets of code from loads of places so if any of you recognise anything you wrote - thanks!

* This program is a trial of a BNO080 device.  It pulls data from the device and pushes it
* to the javascript running on the users browser
* 
* Will "work" standalone (from the command line) but just communicates with the terminal.  It is
* intended to be interfaced with websocketd https://github.com/joewalnes/websocketd to connect to 
* browsers

*/
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
#define QP(n) (pow(2,-n)) 
#define ODR 100
#define BUFFERSIZE 32
#define PUSHBATCH 20

struct timeval current;

char READ_OUTBUF[1500];

uint32_t FRS_READ_BUFFER[64];

uint8_t SEQUENCENUMBER[7];
unsigned int LOOPSLEEP = 2000;  // in microseconds
unsigned int LOOPCOUNT = 0;
char FILENAME[50];
int RUNNING = 0;
int OUT_COUNT = 0;

char READ_OUTBUF[1500];
char READ_OUTBUF_LINE[150];
int READ_OUTBUF_COUNT;

FILE *fd_write_out;

/*
# There are nine possible command sent by the user's browser:
# (a)   STRT:[filename] Start recording a session with a bell.  The bell
#       must be at stand and (reasonably) stationary otherwise this
#       will return ESTD: or EMOV: either of which signals the browser 
#       to stop expecting data.  If Filename is not provided then a
#       default filename of "CurrentRecording [date]" is used.
# (b)   STOP: stops a recording in progress.  This also saves off the
#       collected data into a file in /data/samples/
# (c)   LDAT: a request for data from the broswer, only works when there
#       is a recording in progress - deprecated.  Does nothing.
# (d)   FILE: get a listing of the previous recordings stored in
#       /data/samples/ .  Used to display the selection box to the user.
# (e)   LOAD:[filename] used to transmit a previous recording to the browser
# (f)   DATE:[string] sets the date on the device to the date on the browser 
#             string is unixtime (microsecs since 1/1/70)
# (g)   SAMP: requests the current sample period
# (h)   SHDN: tells the device to shutdown
# (i)   CALI: tells the device to provide calibration and tare data
# (j)   TARE: tares the device in the current position (bell down)
# (k)   CLTA: clears the current tare
# (l)   DOTA: marks tare as already done - useful when tare was done but you need
#               to reset things with the bell at stand
# 
# 
# There are the following responses back to the user's browser:
# (i)   STPD: tells the browser that the device has sucessfully stopped sampling
# (ii)  ESTP: signals an error in the stop process (an attempt to stopped when not started)
# (iii) LIVE:[string] the string contains the data recorded by the device in the format
#             "A:[angle]R:[rate]C:[acceleration]".  Angle, rate and acceleration are 
#             ordinary floating point numbers.  Angle is in degrees, rate is in degrees/sec
#             and acceleration is in degrees/sec/sec.
# (iv)  NDAT: indicates that there is no current data to send to the browser (deprecated)
# (v)   SAMP: returns the sample period
# (vi)  EIMU: IMU not detected or some IMU related failure
# (vii) ESTD: bell not at stand (aborts started session)
# (viii)EMOV: bell moving when session started (the bell has to be stationary at start)
# (ix)  ESAM: sample period needs to be measured first (deprecated)
# (x)   ESTR: internal error related to data / filenames (shouldn't happen)!
# (xi)  DATA:[string]  chunk of data from a previously stored file.  In same format as LIVE:
# (xii) EOVF: (sent by dcmimu) fifo overflow flagged.
# (xiii)FILE:[filename] in response to FILE: a list of the previous recordings in /data/samples
# (xiv) STRT: in response to STRT: indicates a sucessful start.
# (xv)  LFIN:[number] indicates the end of a file download and the number of samples sent
* (xvi) CALI:returns certain calibration/tare data

* Commands (see above) are received on stdin and output is on stdout.
*/

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
    float rateBuffer[BUFFERSIZE];
    float accnBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    unsigned int status;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
    float lastRate;
} gyroData={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0, .lastRate=0 };

struct {
    float angleBuffer[BUFFERSIZE];
    float rateBuffer[BUFFERSIZE];
    float accnBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    float direction;
    float lastYaw;
    float lastPitch;
    float lastRoll;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
    float lastRate;
} gyroIntegratedRotationVectorData ={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };

struct {
    float angleBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    float direction;
    unsigned int status;
    float lastYaw;
    float lastPitch;
    float lastRoll;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} gameRotationVectorData ={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };

struct {
    float XBuffer[BUFFERSIZE];
    float YBuffer[BUFFERSIZE];
    float ZBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    unsigned int status;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} linearAccelerometerData={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };

struct {
    float XBuffer[BUFFERSIZE];
    float YBuffer[BUFFERSIZE];
    float ZBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    unsigned int status;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} accelerometerData={ .head=0, .tail=0, .available=0, .lastReportTimestamp=0 };

struct {
    unsigned int status;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
    unsigned int lastReportTimestamp;
} stabilityData={ .lastReportTimestamp=0 };


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
    int entries;
    int torn;
    float shutdowncount = 0.0;
    FILE *shutdowncheck;

    for(uint32_t i=0; i<sizeof(SEQUENCENUMBER); i++) SEQUENCENUMBER[i] = 0;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);

    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    
    setup();
    setupStabilityClassifierFrs(1.5);
//    
//    reorient(0,0,0,0);
//    
//    reorient(0.0,0.0,-1.0,0.0);

    start(ODR);
    while(!sig_exit){
        usleep(LOOPSLEEP);
        LOOPCOUNT += 1;        
        while(bcm2835_gpio_lev(INTPIN) == 0) handleEvent();

        if((LOOPCOUNT > 5000) && ((LOOPCOUNT % 6000) == 0)  && (torn == 0)){ // if we already haven't torn the device then try doing it now
            if(stabilityData.status == 1 || stabilityData.status == 2) { // check if bell is stable
                if(abs(gameRotationVectorData.lastPitch) < 10.0){
                    if(abs(gameRotationVectorData.lastPitch) < 10.0) {  // device is mounted on top of headstock and roughly level  TODO: side mounting
                        if(gameRotationVectorData.status == 3 && gyroData.status == 3){ // we have good calibrations
                            printf("CALI:%d %d %d P:%+07.1f R:%+07.1f\n",torn,gameRotationVectorData.status,gyroData.status,gameRotationVectorData.lastPitch,gameRotationVectorData.lastRoll);
                        }
                    }
                }
            }
        }

        shutdowncount += LOOPSLEEP/1000;
        if(shutdowncount >= 20000){ // check every 20 seconds
            shutdowncheck = fopen("/run/nologin","r");  // 5 minute warning
            if(shutdowncheck != NULL){
                puts("Low battery warning.\n");
                fclose(shutdowncheck);
            }
            shutdowncount = 0.0;
        }
        
        while(fgets(linein, sizeof(linein), stdin ) != NULL) {
            entries = sscanf(linein, "%5s%[^\n]", command, details);
            if(entries < 1) {
                fprintf( stderr, "\nError reading: %s \n", linein );
                continue;
            }
            if(strcmp("DATE:", command) == 0 && entries == 2  && strlen(details) < 12){
                sprintf(oscommand,"/usr/bin/date --set=\"$(date --date='@%s')\" && /usr/bin/touch /var/lib/systemd/clock", details);
                system(oscommand);
                continue;
            }
            if(strcmp("SAMP:", command) == 0) {
                printf("SAMP:%f\n",1.0/ODR);
                continue;
            }
            if(strcmp("CALI:", command) == 0) {
                printf("CALI:%d %d %d\n",torn,gameRotationVectorData.status,gyroData.status);
                continue;
            }
            if(strcmp("TARE:", command) == 0) {
                saveCalibration();
                torn = 1;
                tare();
                start(ODR); // occasional hang found unless we did this
                continue;
            }
            if(strcmp("CLTA:", command) == 0) {
                clearPersistentTare();
                continue;
            }

            if(strcmp("DOTA:", command) == 0) {
                torn=1;
                continue;
            }            
            if(strcmp("FILE:", command) == 0) {
                DIR *d;
                struct dirent *dir;
                d = opendir("/data/samples");
                if(d){
                    while ((dir = readdir(d)) != NULL) {
                        if(dir->d_type == DT_REG) printf("FILE:%s\n", dir->d_name);
                    }
                closedir(d);
                }
                continue;
            }
            if(strcmp("STRT:", command) == 0) {
                if(entries == 2  && strlen(details) < 34){
                    sprintf(FILENAME, "/data/samples/%s", details);
                } else {
                    struct tm *timenow;
                    time_t now = time(NULL);
                    timenow = gmtime(&now);
                    strftime(FILENAME, sizeof(FILENAME), "/data/samples/Unnamed_%d-%m-%y_%H%M", timenow);
                }
                if(!startRun()) continue;
                fd_write_out = fopen(FILENAME,"w");
                if(fd_write_out == NULL) {
                    fprintf(stderr, "Could not open file for writing\n");
                    printf("ESTR:\n");
                    printf("STPD:\n");
                    continue;
                } else {
                    printf("STRT:\n"); 
                }
                continue;
            }
            if(strcmp("STOP:", command) == 0) {
                if(fd_write_out != NULL){
                    fflush(fd_write_out);
                    fclose(fd_write_out);
                    fd_write_out = NULL;
                }
                RUNNING = 0;
//                calibrationSetup(1,1,1);
                printf("STPD:%d\n",OUT_COUNT);
                continue;
            }
            if(strcmp("LOAD:", command) == 0 && RUNNING == 0) {
                if(entries == 2  && strlen(details) < 34){
                    sprintf(FILENAME, "/data/samples/%s", details);
                } else {
                    printf("ESTR:\n");
                    continue;
                }
                FILE *fd_read_in;
                fd_read_in = fopen(FILENAME,"r");
                if(fd_read_in == NULL) {
                    fprintf(stderr, "Could not open file for reading\n");
                    printf("ESTR:\n");
                    continue;
                } else {
                    int counter = 0;
                    READ_OUTBUF_COUNT = 0;
                    while(fgets(READ_OUTBUF_LINE, sizeof(READ_OUTBUF_LINE), fd_read_in ) != NULL) {
                        size_t ln = strlen(READ_OUTBUF_LINE);
                        if(ln > 0 && READ_OUTBUF_LINE[ln -1] == '\n') READ_OUTBUF_LINE[ln - 1] = '\0';   // get rid of /n
//                        READ_OUTBUF_LINE[strcspn(READ_OUTBUF_LINE, "\n")] = 0;
                        if (READ_OUTBUF_COUNT + strlen(READ_OUTBUF_LINE) + 7 > (sizeof(READ_OUTBUF) -2)) {
                            printf("%s\n", READ_OUTBUF);
                            READ_OUTBUF_COUNT = 0;
                        }
                        READ_OUTBUF_COUNT += sprintf(&READ_OUTBUF[READ_OUTBUF_COUNT],"DATA:%s",READ_OUTBUF_LINE);
                        counter += 1;
                    }
                    if(READ_OUTBUF_COUNT != 0) printf("%s\n",READ_OUTBUF);
                    printf("LFIN:%d\n", counter);
                }
                fclose(fd_read_in);
                continue;
            }
            if(strcmp("SHDN:", command) == 0) {
                system("/usr/bin/sync && /usr/bin/shutdown -P now");
                continue;
            }
            printf("ESTR:\n");
            fprintf(stderr, "Unrecognised command: %s \n", command);
        }
    }
    if(fd_write_out != NULL) {
        fflush(fd_write_out);
        fclose(fd_write_out);
    }
    start(0);
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
    gameRotationVectorData.tail = 1;  // this function puts the starting angle in the fifo
    gameRotationVectorData.available = 0;
    gyroIntegratedRotationVectorData.head = 1;
    gyroIntegratedRotationVectorData.tail = 1;
    gyroIntegratedRotationVectorData.available = 0;
    linearAccelerometerData.head = 0;
    linearAccelerometerData.tail = 0;
    linearAccelerometerData.available = 0;
    gyroData.head = 0;
    gyroData.tail = 0;
    gyroData.available = 0;
    accelerometerData.head = 0;
    accelerometerData.tail = 0;
    accelerometerData.available = 0;
    
    if(stabilityData.status != 1 && stabilityData.status != 2) {
        printf("EMOV:\n"); // bell is moving
        return(0);
    }
    if(abs(gameRotationVectorData.lastRoll) < 160 || abs(gameRotationVectorData.lastRoll) > 178){
        printf("ESTD:\n"); // bell not at stand
        return(0);
    }
    
    calibrationSetup(0,0,0); // disable auto calibration
    if(gameRotationVectorData.lastRoll < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        gameRotationVectorData.direction = -1;
        gameRotationVectorData.angleBuffer[0] = -180.0-gameRotationVectorData.lastRoll;
        
    } else {
        gameRotationVectorData.direction = +1;
        gameRotationVectorData.angleBuffer[0] = gameRotationVectorData.lastRoll-180.0;
    }

    if(gyroIntegratedRotationVectorData.lastRoll < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        gyroIntegratedRotationVectorData.direction = -1;
        gyroIntegratedRotationVectorData.angleBuffer[0] = -180.0-gyroIntegratedRotationVectorData.lastRoll;
    } else {
        gyroIntegratedRotationVectorData.direction = +1;
        gyroIntegratedRotationVectorData.angleBuffer[0] = gyroIntegratedRotationVectorData.lastRoll-180.0;
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

// this may not be needed
void alignFIFOs(void){
    if(gyroData.available >= 3){
        if ((gyroData.available - 3) > gameRotationVectorData.available ||
//            (gyroData.available - 3) > gyroIntegratedRotationVectorData.available ||
            (gyroData.available - 3) > accelerometerData.available) {
//            (gyroData.available - 3) > linearAccelerometerData.available){
                gyroData.tail = (gyroData.tail + 1) % BUFFERSIZE; // ditch a sample don't forget that there is a gyro accelerometer reading here too
                gyroData.available -= 1;
                printf("Ditched rate sample\n");
        }
    }

    if(gameRotationVectorData.available >= 3){
        if ((gameRotationVectorData.available - 3) > gyroData.available ||
//            (gameRotationVectorData.available - 3) > gyroIntegratedRotationVectorData.available ||
            (gameRotationVectorData.available - 3) > accelerometerData.available){
//            (gameRotationVectorData.available - 3) > linearAccelerometerData.available){
                gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) % BUFFERSIZE; // ditch a sample
                gameRotationVectorData.available -= 1;
                printf("Ditched angle sample\n");
        }
    }

/*
    if(gyroIntegratedRotationVectorData.available >= 3){
        if ((gyroIntegratedRotationVectorData.available - 3) > gyroData.available ||
            (gyroIntegratedRotationVectorData.available - 3) > gameRotationVectorData.available ||
            (gyroIntegratedRotationVectorData.available - 3) > accelerometerData.available ||
            (gyroIntegratedRotationVectorData.available - 3) > linearAccelerometerData.available){
                gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData.tail + 1) % BUFFERSIZE; // ditch a sample
                gyroIntegratedRotationVectorData.available -= 1;
                printf("Ditched GI sample\n");
        }
    }

    if(linearAccelerometerData.available >= 3){
        if ((linearAccelerometerData.available - 3) > gyroData.available ||
            (linearAccelerometerData.available - 3) > gyroIntegratedRotationVectorData.available ||
            (linearAccelerometerData.available - 3) > accelerometerData.available ||
            (linearAccelerometerData.available - 3) > gameRotationVectorData.available){
                linearAccelerometerData.tail = (linearAccelerometerData.tail + 1) % BUFFERSIZE; // ditch a sample
                linearAccelerometerData.available -= 1;
                printf("Ditched linacc sample\n");
        }
    }
*/
    
    if(accelerometerData.available >= 3){
        if ((accelerometerData.available - 3) > gyroData.available ||
//            (accelerometerData.available - 3) > gyroIntegratedRotationVectorData.available ||
//            (accelerometerData.available - 3) > linearAccelerometerData.available ||
            (accelerometerData.available - 3) > gameRotationVectorData.available){
                accelerometerData.tail = (accelerometerData.tail + 1) % BUFFERSIZE; // ditch a sample
                accelerometerData.available -= 1;
                printf("Ditched acc sample\n");
        }
    }

}

void pushData(void){
//    uint16_t statuses = gyroData.status | (linearAccelerometerData.status << 4) | (gameRotationVectorData.status << 8);

    uint16_t statuses = gyroData.status | (gameRotationVectorData.status << 4);

    char output[2048];
    int outputCount = 0;

//    if (){

//    if (gyroData.available >=10 && linearAccelerometerData.available >=10 && gameRotationVectorData.available >=10 && gyroIntegratedRotationVectorData.available >=10  && accelerometerData.available >= 10){
//        printf("Gyro: %d, LinAcc: %d, Angle: %d",gyroData.available, linearAccelerometerData.available, gameRotationVectorData.available);
    if (gyroData.available >= PUSHBATCH && gameRotationVectorData.available >= PUSHBATCH && accelerometerData.available >= PUSHBATCH){
        for(int counter = 0; counter < PUSHBATCH; ++counter){
//            printf("A:%+07.1f,R:%+07.1f,C:%+07.1f\n", gyroIntegratedRotationVectorData.angleBuffer[gyroIntegratedRotationVectorData.tail], gyroIntegratedRotationVectorData.rateBuffer[gyroIntegratedRotationVectorData.tail], gyroIntegratedRotationVectorData.accnBuffer[gyroIntegratedRotationVectorData.tail]);
//            outputCount += sprintf(&output[outputCount],"LIVE:A:%+07.1f,R:%+07.1f,C:%+07.1f,LA:%+07.1f,AC:%+07.1f,GI:%+07.1f,S:%04X\n", gameRotationVectorData.angleBuffer[gameRotationVectorData.tail], gyroData.rateBuffer[gyroData.tail]*gameRotationVectorData.direction, gyroData.accnBuffer[gyroData.tail]*gameRotationVectorData.direction, linearAccelerometerData.YBuffer[linearAccelerometerData.tail],accelerometerData.YBuffer[accelerometerData.tail],gyroIntegratedRotationVectorData.angleBuffer[gyroIntegratedRotationVectorData.tail],statuses);
            outputCount += sprintf(&output[outputCount],"LIVE:A:%+07.1f,R:%+07.1f,C:%+07.1f,AC:%+07.1f,S:%04X\n", gameRotationVectorData.angleBuffer[gameRotationVectorData.tail], gyroData.rateBuffer[gyroData.tail]*gameRotationVectorData.direction, gyroData.accnBuffer[gyroData.tail]*gameRotationVectorData.direction,accelerometerData.YBuffer[accelerometerData.tail]*gameRotationVectorData.direction,statuses);
//            outputCount += sprintf(&output[outputCount],"LIVE:A:%+07.1f,R:%+07.1f,C:%+07.1f,S:%04X\n", gameRotationVectorData.angleBuffer[gameRotationVectorData.tail], gyroData.rateBuffer[gyroData.tail]*gameRotationVectorData.direction, gyroData.accnBuffer[gyroData.tail]*gameRotationVectorData.direction,statuses);



//            gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData + 1) % BUFFERSIZE
            gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) % BUFFERSIZE;
            gyroData.tail = (gyroData.tail + 1) % BUFFERSIZE;
//            linearAccelerometerData.tail = (linearAccelerometerData.tail + 1) % BUFFERSIZE;
//            gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData.tail + 1) % BUFFERSIZE;
            accelerometerData.tail = (accelerometerData.tail + 1) % BUFFERSIZE;
        }
//        gyroIntegratedRotationVectorData.available -= PUSHBATCH;
        gameRotationVectorData.available -= PUSHBATCH;
        gyroData.available -= PUSHBATCH;
//        linearAccelerometerData.available -= PUSHBATCH;
        accelerometerData.available -= PUSHBATCH;
        OUT_COUNT += PUSHBATCH;
        
        printf("%s",output);
        fputs(output,fd_write_out);
        fflush(fd_write_out);
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
            case SENSOR_REPORTID_ACCELEROMETER:                         parseAccelerometer(); break;
            case SENSOR_REPORTID_LINEAR_ACCELERATION:                   parseLinearAccelerometer(); break;
            case SENSOR_REPORTID_STABILITY_CLASSIFIER:                  parseStability(); break;
            default: printf("Unhandled report event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
        }
        return;
    }

    if(spiRead.channel == CHANNEL_CONTROL) {
        switch(spiRead.buffer[0]) {
            case SHTP_REPORT_COMMAND_RESPONSE:      reportCommandResponse(); break;
            case SHTP_REPORT_GET_FEATURE_RESPONSE:  reportFeatureResponse(); break;
            case SHTP_REPORT_PRODUCT_ID_RESPONSE:   break; // ignore this
            case SHTP_REPORT_FRS_WRITE_RESPONSE:    break; // sometimes get a few of these for each write - ignore them
            default: printf("Unhandled control event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
        }
        return;
    }
    
    if(spiRead.channel == CHANNEL_GYRO) parseGyroIntegratedRotationVector();
}

void reportFeatureResponse(void){
    unsigned int reportInterval = (spiRead.buffer[8 ] << 24) + (spiRead.buffer[7 ] << 16) + (spiRead.buffer[6 ] << 8) + spiRead.buffer[5];
    unsigned int batchInterval =  (spiRead.buffer[12] << 24) + (spiRead.buffer[11] << 16) + (spiRead.buffer[10] << 8) + spiRead.buffer[9];
    switch(spiRead.buffer[1]){
        case SENSOR_REPORTID_GAME_ROTATION_VECTOR:  
            gameRotationVectorData.reportInterval = reportInterval; 
            if(gameRotationVectorData.reportInterval != gameRotationVectorData.requestedInterval) printf("Warning: GRV data rate mismatch. Req: %d, Rep: %d\n",gameRotationVectorData.requestedInterval,reportInterval);
            break;
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:  
            gyroData.reportInterval = reportInterval;
            if(gyroData.reportInterval != gyroData.requestedInterval) printf("Warning: Gyro data rate mismatch. Req: %d, Rep: %d\n",gyroData.requestedInterval,reportInterval);
            break;
        case SENSOR_REPORTID_LINEAR_ACCELERATION:   
            linearAccelerometerData.reportInterval = reportInterval;
            if(linearAccelerometerData.reportInterval != linearAccelerometerData.requestedInterval) printf("Warning: LinAcc data rate mismatch. Req: %d, Rep: %d\n", linearAccelerometerData.requestedInterval,reportInterval);
            break;
        case SENSOR_REPORTID_ACCELEROMETER:         
            accelerometerData.reportInterval = reportInterval; 
            if(accelerometerData.reportInterval != accelerometerData.requestedInterval) printf("Warning: Acc data rate mismatch. Req: %d, Rep: %d\n",accelerometerData.requestedInterval,reportInterval);
            break;
        case SENSOR_REPORTID_STABILITY_CLASSIFIER:  
            stabilityData.reportInterval = reportInterval; 
            if(stabilityData.reportInterval != stabilityData.requestedInterval) printf("Warning: Stability data rate mismatch. Req: %d, Rep: %d\n",stabilityData.requestedInterval,reportInterval);
            break;
        case SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR: 
            gyroIntegratedRotationVectorData.reportInterval = reportInterval;
            if(gyroIntegratedRotationVectorData.reportInterval != gyroIntegratedRotationVectorData.requestedInterval) printf("Warning: GIRV data rate mismatch. Req: %d, Rep: %d\n",gyroIntegratedRotationVectorData.requestedInterval,reportInterval);
            break;
        default: printf("Unhandled FR event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
    }
}

void reportCommandResponse(void){
    switch(spiRead.buffer[2]){
        case 0x84: printf("Error: device has reinitialised!\n"); break;  // bit 7 set indicates autonomous response 0x04 = Initialisation.  Sometimes caused by too much data being pushed.
        case 0x06: printf("Save DCD.  Success: %d\n", spiRead.buffer[5] ); break;
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
 
    float rolldiff = roll - gyroIntegratedRotationVectorData.lastRoll;
    if(rolldiff > 250.0) {
        rolldiff -= 360.0;
    } else if (rolldiff < -250.0) {
        rolldiff += 360.0;
    }
    
    gyroIntegratedRotationVectorData.lastRoll = roll;
    gyroIntegratedRotationVectorData.lastPitch = pitch;
    gyroIntegratedRotationVectorData.lastYaw = yaw;

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gyroIntegratedRotationVectorData.available == (BUFFERSIZE - 2)) {
        printf("Rate buffer full.  Ditching oldest sample\n");
        gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData.tail + 1) % BUFFERSIZE;
        gyroIntegratedRotationVectorData.available -= 1;
    }
            
    gyroIntegratedRotationVectorData.rateBuffer[gyroIntegratedRotationVectorData.head] = Gx;
    gyroIntegratedRotationVectorData.accnBuffer[gyroIntegratedRotationVectorData.head] = (Gx-gyroIntegratedRotationVectorData.lastRate)*ODR;
    gyroIntegratedRotationVectorData.lastRate = Gx;
    unsigned int lastHead = (gyroIntegratedRotationVectorData.head == 0) ? BUFFERSIZE-1 : gyroIntegratedRotationVectorData.head -1;
    gyroIntegratedRotationVectorData.angleBuffer[gyroIntegratedRotationVectorData.head] = gyroIntegratedRotationVectorData.angleBuffer[lastHead] + gyroIntegratedRotationVectorData.direction*rolldiff;
    gyroIntegratedRotationVectorData.head = (gyroIntegratedRotationVectorData.head + 1) % BUFFERSIZE;
    gyroIntegratedRotationVectorData.available += 1;
}

void parseGameRotationVector(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == gameRotationVectorData.lastSequence) return; // sometimes HINT is not brought low quickly, this checks to see if we are reading a report we have already seen.
    gameRotationVectorData.lastSequence = newSequence;

    uint8_t status = spiRead.buffer[5 + 2] & 0x03;
   
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);
    float Qx = (float)((spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4]);
    float Qy = (float)((spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6]);
    float Qz = (float)((spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8]);
    float Qw = (float)((spiRead.buffer[5 + 11] << 8) + spiRead.buffer[5 + 10]);
    
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

// https://math.stackexchange.com/questions/687964/getting-euler-tait-bryan-angles-from-quaternion-representation
//    float yaw   =  atan2(Qx * Qy + Qw * Qz, 0.5 - (Qw * Qw + Qx * Qx)) * RADIANS_TO_DEGREES_MULTIPLIER;   
//    float pitch = asin(-2.0 * (Qx * Qz - Qw * Qy))* RADIANS_TO_DEGREES_MULTIPLIER;
//    float roll  =  atan2(Qw * Qx + Qy * Qz, 0.5 - (Qw * Qw + Qz * Qz))* RADIANS_TO_DEGREES_MULTIPLIER;


    float yaw   =  atan2(Qx * Qy + Qw * Qz, (Qw * Qw + Qx * Qx) - 0.5) * RADIANS_TO_DEGREES_MULTIPLIER;   
    float pitch = -asin(2.0 * (Qx * Qz - Qw * Qy)) * RADIANS_TO_DEGREES_MULTIPLIER;
    float roll  =  atan2(Qw * Qx + Qy * Qz, (Qw * Qw + Qz * Qz) - 0.5) * RADIANS_TO_DEGREES_MULTIPLIER;


    gameRotationVectorData.status = status;
    gettimeofday(&current, NULL);
    gameRotationVectorData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    float rolldiff = roll - gameRotationVectorData.lastRoll;
    if(rolldiff > 250.0) {
        rolldiff -= 360.0;
    } else if (rolldiff < -250.0) {
        rolldiff += 360.0;
    }    

    gameRotationVectorData.lastRoll = roll;
    gameRotationVectorData.lastPitch = pitch;
    gameRotationVectorData.lastYaw = yaw;

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gameRotationVectorData.available == (BUFFERSIZE - 2)) {
        printf("Angle buffer full.  Ditching oldest sample\n");
        gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) % BUFFERSIZE;
        gameRotationVectorData.available -= 1;
    }

    unsigned int lastHead = (gameRotationVectorData.head == 0) ? BUFFERSIZE-1 : gameRotationVectorData.head -1;
    gameRotationVectorData.angleBuffer[gameRotationVectorData.head] =  gameRotationVectorData.angleBuffer[lastHead] + gameRotationVectorData.direction*rolldiff;
    gameRotationVectorData.head = (gameRotationVectorData.head + 1) % BUFFERSIZE;
    gameRotationVectorData.available += 1;

//    printf("Game Rotation Vector.  S: %d R:%+07.1f, P:%+07.1f, Y:%+07.1f, Sequence: %02X, Buff: %d\n", status, roll, pitch, yaw, spiRead.sequence, gameRotationVectorData.available);
}

void parseLinearAccelerometer(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == linearAccelerometerData.lastSequence) return; // sometimes HINT is not brought low quickly, this checks to see if we are reading a report we have already seen.
    linearAccelerometerData.lastSequence = newSequence;

    uint8_t status = spiRead.buffer[5 + 2] & 0x03;
    
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);
    float Ax = (float)((spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4]);
    float Ay = (float)((spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6]);
    float Az = (float)((spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8]);
    
    if(Ax >= 0x8000) Ax -= 0x10000;
    if(Ay >= 0x8000) Ay -= 0x10000;
    if(Az >= 0x8000) Az -= 0x10000;

    Ax *= QP(8) * 100;
    Ay *= QP(8) * 100;
    Az *= QP(8) * 100;

    linearAccelerometerData.status = status;
    gettimeofday(&current, NULL);
    linearAccelerometerData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (linearAccelerometerData.available == (BUFFERSIZE - 2)) {
        printf("Lin Acc buffer full.  Ditching oldest sample\n");
        linearAccelerometerData.tail = (linearAccelerometerData.tail + 1) % BUFFERSIZE;
        linearAccelerometerData.available -= 1;
    }
    linearAccelerometerData.XBuffer[linearAccelerometerData.head] = Ax;
    linearAccelerometerData.YBuffer[linearAccelerometerData.head] = Ay;
    linearAccelerometerData.ZBuffer[linearAccelerometerData.head] = Az;
    linearAccelerometerData.head = (linearAccelerometerData.head + 1) % BUFFERSIZE;
    linearAccelerometerData.available += 1;
    
//    printf("Lin. Accel.  S: %d X:%+07.2f, Y:%+07.2f, Z:%+07.2f, Sequence: %02X, Buff: %d\n", status, Ax, Ay, Az, spiRead.sequence, CB_linaccn.available);
}

void parseAccelerometer(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == accelerometerData.lastSequence) return; // sometimes HINT is not brought low quickly, this checks to see if we are reading a report we have already seen.
    accelerometerData.lastSequence = newSequence;

    uint8_t status = spiRead.buffer[5 + 2] & 0x03;
    
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);
    float Ax = (float)((spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4]);
    float Ay = (float)((spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6]);
    float Az = (float)((spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8]);
    
    if(Ax >= 0x8000) Ax -= 0x10000;
    if(Ay >= 0x8000) Ay -= 0x10000;
    if(Az >= 0x8000) Az -= 0x10000;

    Ax *= QP(8) * 100;
    Ay *= QP(8) * 100;
    Az *= QP(8) * 100;

    accelerometerData.status = status;
    gettimeofday(&current, NULL);
    accelerometerData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (accelerometerData.available == (BUFFERSIZE - 2)) {
        printf("Lin Acc buffer full.  Ditching oldest sample\n");
        accelerometerData.tail = (accelerometerData.tail + 1) % BUFFERSIZE;
        accelerometerData.available -= 1;
    }
    accelerometerData.XBuffer[accelerometerData.head] = Ax;
    accelerometerData.YBuffer[accelerometerData.head] = Ay;
    accelerometerData.ZBuffer[accelerometerData.head] = Az;
    accelerometerData.head = (accelerometerData.head + 1) % BUFFERSIZE;
    accelerometerData.available += 1;
    
//    printf("Accel.  S: %d X:%+07.2f, Y:%+07.2f, Z:%+07.2f, Sequence: %02X, Buff: %d\n", status, Ax, Ay, Az, spiRead.sequence, CB_linaccn.available);
}


void parseCalibratedGyroscope(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == gyroData.lastSequence) return; // sometimes HINT is not brought low quickly, this checks to see if we are reading a report we have already seen.
    gyroData.lastSequence = newSequence;
    float Gx, Gy, Gz;
//    uint32_t timebase = (uint32_t)((spiRead.buffer[4] << 24) + (spiRead.buffer[3] << 16 ) + (spiRead.buffer[2] << 8) + spiRead.buffer[1]);

    uint8_t status = spiRead.buffer[5 + 2] & 0x03;

    uint32_t Gxi = (spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4];
    uint32_t Gyi = (spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6];
    uint32_t Gzi = (spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8];
    
    Gx = Gxi;
    Gy = Gyi;
    Gz = Gzi;
    
    if(Gxi >= 0x8000) Gx -= 0x10000;
    if(Gyi >= 0x8000) Gy -= 0x10000;
    if(Gzi >= 0x8000) Gz -= 0x10000;

    Gx *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gy *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gz *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;

    gyroData.status = status;
    gettimeofday(&current, NULL);
    gyroData.lastReportTimestamp= (unsigned int)((current.tv_sec)*1000 +(current.tv_usec)/1000);

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gyroData.available == (BUFFERSIZE - 2)) {
        printf("Rate buffer full.  Ditching oldest sample\n");
        gyroData.tail = (gyroData.tail + 1) % BUFFERSIZE;
        gyroData.available -= 1;
    }
    gyroData.rateBuffer[gyroData.head] = Gx;
    gyroData.accnBuffer[gyroData.head] = (Gx-gyroData.lastRate)*ODR;
    gyroData.lastRate = Gx;
    gyroData.head = (gyroData.head + 1) % BUFFERSIZE;
    gyroData.available += 1;
}

void parseStability(void){    
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == stabilityData.lastSequence) return; // sometimes HINT is not brought low quickly, this checks to see if we are reading a report we have already seen.
    stabilityData.lastSequence = newSequence;

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
//    writeFrsWord(SYSTEM_ORIENTATION,0,0);
//    writeFrsWord(SYSTEM_ORIENTATION,1,0);
//    writeFrsWord(SYSTEM_ORIENTATION,2,0);
//    writeFrsWord(SYSTEM_ORIENTATION,3,0);
    reorient(0,0,0,0);
    eraseFrsRecord(SYSTEM_ORIENTATION);
      
}

void tare(void){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; 
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
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++;
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

    int timeout = 0;
    int counter = 0;
    while(timeout < 1000){
        timeout +=1;
        usleep(50);
        while(bcm2835_gpio_lev(INTPIN) == 0) {
            collectPacket();
            if(spiRead.buffer[2] != 0x06) {parseEvent(); break;}
            if(spiRead.buffer[5] != 0) printf("Calibration save error\n");
            return;
        }
    }
    printf("Calibration save timeout\n");
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
            FRS_READ_BUFFER[counter] = (uint32_t)spiRead.buffer[11] << 24 | (uint32_t)spiRead.buffer[10] << 16 | (uint32_t)spiRead.buffer[9] << 8 | (uint32_t)spiRead.buffer[8];
            counter += 2;
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

    for(int i = 0; i<2000; i++){
        usleep(100);
        while(bcm2835_gpio_lev(INTPIN) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_WRITE_RESPONSE) {parseEvent(); continue;}
            if(spiRead.buffer[1] != 3) {
                printf("FRS Write, %02X\n",spiRead.buffer[1]);
                continue;
            } else {
                printf("Write OK\n");
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

void start(uint32_t dataRate){ // set datarate to zero to stop sensors
// setup rotation vector quaternion report
    uint32_t odrPeriodMicrosecs = 0;
    if(dataRate != 0) odrPeriodMicrosecs = 1000000/dataRate; 

// we are going to arrange things so that calibration data is only persisted when we ask for it (saveCalibration() does this)
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

// enable calibration (we should be stationary)  // TODO: check stationary
    calibrationSetup(1,1,1);

    configureFeatureReport(SENSOR_REPORTID_GAME_ROTATION_VECTOR, odrPeriodMicrosecs);
    gameRotationVectorData.requestedInterval=odrPeriodMicrosecs;
    gameRotationVectorData.direction=0;
    
    configureFeatureReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, odrPeriodMicrosecs);
    gyroData.requestedInterval=odrPeriodMicrosecs;

//    configureFeatureReport(SENSOR_REPORTID_LINEAR_ACCELERATION, odrPeriodMicrosecs);
//    linearAccelerometerData.requestedInterval=odrPeriodMicrosecs;
    
    configureFeatureReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, 500000);// two reports per second
    stabilityData.requestedInterval=500000;

    configureFeatureReport(SENSOR_REPORTID_ACCELEROMETER, odrPeriodMicrosecs);
    accelerometerData.requestedInterval=odrPeriodMicrosecs;

//    configureFeatureReport(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, odrPeriodMicrosecs);
//    gyroIntegratedRotationVectorData.requestedInterval=odrPeriodMicrosecs;
//    gyroIntegratedRotationVectorData.direction = 0;
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

void setup(void){
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
