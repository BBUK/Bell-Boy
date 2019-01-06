//gcc grabber.c -o grabber -lm -lbcm2835

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

* This program uses a BNO080 device.  It pulls data from the device and pushes it
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

// These are Broadcom GPIO numbers NOT Raspberry Pi pin numbers
#define INTGPIO 23
#define RESETGPIO 24
#define WAKPS0GPIO 25

#define QP(n) (pow(2,-n)) 

// interesting effect, the gyro integrated rotation vector is unstable
// at ODR values in which the game rotation vector is "uncomfortable"
// (The game rotation vector is used for stabilisation of the GIRV.) 
// An ODR of 400 is quite noisy but 200 works fine.
#define ODR 200 
#define PUSHBATCH 20 // number of samples taken before pushing out
#define SAVGOLLENGTH  31
#define SAVGOLHALF 15 // = math.floor(SAVGOLLENGTH/2.0)
#define BUFFERSIZE 80 // must be bigger than PUSHBATCH + SAVGOLLENGTH
#define R2O2 (sqrt(2)/2.0)

#define SMOOTHFACTOR 0.75

char READ_OUTBUF[1500];

// savgol_coeffs(31,1,deriv=1,use="dot")
const float savGolCoefficients[] = {
    -6.04838710e-03, -5.64516129e-03, -5.24193548e-03, -4.83870968e-03,
    -4.43548387e-03, -4.03225806e-03, -3.62903226e-03, -3.22580645e-03,
    -2.82258065e-03, -2.41935484e-03, -2.01612903e-03, -1.61290323e-03,
    -1.20967742e-03, -8.06451613e-04, -4.03225806e-04, -1.02877021e-17,
    4.03225806e-04,   8.06451613e-04,  1.20967742e-03,  1.61290323e-03,
    2.01612903e-03,   2.41935484e-03,  2.82258065e-03,  3.22580645e-03,
    3.62903226e-03,   4.03225806e-03,  4.43548387e-03,  4.83870968e-03,
    5.24193548e-03,   5.64516129e-03,  6.04838710e-03};

uint32_t FRS_READ_BUFFER[64];
uint32_t FRS_WRITE_BUFFER[64];

uint8_t SEQUENCENUMBER[7];
unsigned int LOOPSLEEP = 1000;  // in microseconds
unsigned int LOOPCOUNT = 0;
char FILENAME[50];
int RUNNING = 0;
int CALIBRATING = 0;
int OUT_COUNT = 0;

char READ_OUTBUF[1500];
char READ_OUTBUF_LINE[150];
int READ_OUTBUF_COUNT;

FILE *fd_write_out;

/*
# There are the following possible command sent by the user's browser:
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
# (k)   CLTA: clears the current tare (back to standard orientation - which may not be quite correct)
# (l)   SAVE: save current calibration to flash
# (m)   STCA: stops the current calibration
# (n)   PFRS: (for testing - prints the current GIRV FRS config record)
# (m)   TEST: (for testing - prints current orientation
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
# (xvi) CALI:returns certain calibration/tare data

* Commands (see above) are received on stdin and output is on stdout.
*/

struct {
    uint32_t buffer[MAX_PACKET_SIZE];
    unsigned int sequence;
    unsigned int dataLength;
    uint8_t channel;
} spiRead;

struct {
    uint32_t buffer[256];
} spiWrite;

struct {
    float rateBuffer[BUFFERSIZE];
    float accnBuffer[BUFFERSIZE];// this is just raw acceleration (for testing) the used acceleration is calculated using a Savgol filter
    unsigned int status;
    unsigned int lastSequence;
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    int direction;
    float lastX;
    float lastY;
    float lastZ;
    unsigned int requestedInterval;
    unsigned int reportInterval;    
} gyroData = {.head=0, .tail=0, .available=0};

struct {
    float angleBuffer[BUFFERSIZE];
    float rateBuffer[BUFFERSIZE];
    float accnBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    int direction;
    float lastYaw;
    float lastPitch;
    float lastRoll;
    float lastX;
    float lastY;
    float lastZ;
    float tareValue;
    float lastAngle;
    float smoothRoll;
    float smoothAngle;
    float smoothRate;
    float smoothAccn;
    float angleXV[3];
    float angleYV[3];
    float rateXV[3];
    float rateYV[3];
    float accnXV[3];
    float accnYV[3];
    unsigned int requestedInterval;
    unsigned int reportInterval;
} gyroIntegratedRotationVectorData ={ .smoothAngle=0.0, .smoothRate=0.0, .smoothRoll =0.0, .smoothAccn =0.0, .head=0, .tail=0, .available=0, .tareValue=0.0 };

struct {
    float angleBuffer[BUFFERSIZE];
    unsigned int head;
    unsigned int tail;
    unsigned int available;
    unsigned int status;
    unsigned int lastSequence;
    int direction;
    float lastYaw;
    float lastPitch;
    float lastRoll;
    float tareValue;
    unsigned int calibrationCount;
    unsigned int requestedInterval;
    unsigned int reportInterval;
} gameRotationVectorData  = {.head=0, .tail=0, .available=0, .lastRoll = 0.0, .tareValue = 0.0};

struct {
    unsigned int status;
    unsigned int lastSequence;
    float lastX;
    float lastY;
    float lastZ;
    unsigned int requestedInterval;
    unsigned int reportInterval;
} linearAccelerometerData;

struct {
    unsigned int status;
    unsigned int lastSequence;
    float lastX;
    float lastY;
    float lastZ;
    int direction;
    unsigned int requestedInterval;
    unsigned int reportInterval;
} calibratedMagneticFieldData;

struct {
    unsigned int status;
    unsigned int lastSequence;
    float lastX;
    float lastY;
    float lastZ;
    unsigned int requestedInterval;
    unsigned int reportInterval;
} accelerometerData;

struct {
    unsigned int status;
    unsigned int lastSequence;
    unsigned int requestedInterval;
    unsigned int reportInterval;
} stabilityData;

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
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);

    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    setup();
    start(ODR);
    
    FILE *fdTare;
    fdTare = fopen("/tmp/bb_tare","r");
    if(fdTare != NULL) {
        if (fgets(linein, sizeof(linein), fdTare) != NULL) {
            gameRotationVectorData.tareValue = atof(linein);
        }
        fclose(fdTare);
    }
    
    while(!sig_exit){
        usleep(LOOPSLEEP);
        LOOPCOUNT += 1;        
        while(bcm2835_gpio_lev(INTGPIO) == 0) handleEvent();

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
                if(RUNNING) continue;
                eraseFrsRecord(SYSTEM_ORIENTATION);
                tareZ();
                CALIBRATING = 1;
                gameRotationVectorData.calibrationCount = 0;
                continue;
            }
            if(strcmp("STCA:", command) == 0) {
                CALIBRATING = 0;
                continue;
            }
            if(strcmp("PFRS:", command) == 0) {
                printGyroIntegratedRotationVectorFRS();
                continue;
            }
            if(strcmp("TEST:", command) == 0) {
                printf("TEST:%f, %+07.1f, %+07.1f, %+07.1f\n", gameRotationVectorData.tareValue, gameRotationVectorData.lastRoll, gameRotationVectorData.lastPitch, gameRotationVectorData.lastYaw);
                continue;
            }
            if(strcmp("TARE:", command) == 0) {
                gameRotationVectorData.tareValue = (gameRotationVectorData.lastRoll + gameRotationVectorData.tareValue);
                fdTare = fopen("/tmp/bb_tare","w");
                if(fdTare == NULL) {
                    printf("EIMU: Could not open tare file for writing\n");
                    continue;
                } else {
                    fprintf(fdTare,"%+07.4f\n", gameRotationVectorData.tareValue);
                    fclose(fdTare);
                }
                continue;
            }
            if(strcmp("CLTA:", command) == 0) { // consider removing file too
                gameRotationVectorData.tareValue = 0.0;
                fdTare = fopen("/tmp/bb_tare","w");
                if(fdTare == NULL) {
                    printf("EIMU: Could not open tare file for writing\n");
                    continue;
                } else {
                    fprintf(fdTare,"0.0000\n");
                    fclose(fdTare);
                }
                continue;
            }
            if(strcmp("SAVE:", command) == 0) {
                saveCalibration();
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
                    OUT_COUNT = 0;
                }
                CALIBRATING = 0;
                continue;
            }
            if(strcmp("STOP:", command) == 0) {
                if(fd_write_out != NULL){
                    fflush(fd_write_out);
                    fclose(fd_write_out);
                    fd_write_out = NULL;
                }
                RUNNING = 0;
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
                        if(ln > 0 && READ_OUTBUF_LINE[ln -1] == '\n') READ_OUTBUF_LINE[ln - 1] = '\0';   // get rid of newline
//                        READ_OUTBUF_LINE[strcspn(READ_OUTBUF_LINE, "\n")] = 0;
                        if (READ_OUTBUF_COUNT + strlen(READ_OUTBUF_LINE) + 7 > (sizeof(READ_OUTBUF) -2)) {
                            printf("%s\n", READ_OUTBUF);
                            READ_OUTBUF_COUNT = 0;
                        }
                        READ_OUTBUF_COUNT += sprintf(&READ_OUTBUF[READ_OUTBUF_COUNT],"DATA:%s",READ_OUTBUF_LINE);
                        counter += 1;
                        while(bcm2835_gpio_lev(INTGPIO) == 0) handleEvent();  // pick up and deal with any reports
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

// reset fifo
    gameRotationVectorData.head = SAVGOLHALF;
    gameRotationVectorData.tail = 0;  // this function puts the starting angle in the fifo
    gameRotationVectorData.available = SAVGOLHALF;

    gyroData.head = SAVGOLHALF;
    gyroData.tail = 0;
    gyroData.available = SAVGOLHALF;

    gyroIntegratedRotationVectorData.head = SAVGOLHALF;
    gyroIntegratedRotationVectorData.tail = 0;
    gyroIntegratedRotationVectorData.available = SAVGOLHALF;

    if(stabilityData.status != 1 && stabilityData.status != 2) {
        printf("EMOV:%+07.1f\n",gameRotationVectorData.lastRoll); // bell is moving
        return(0);
    }

    if(abs(gameRotationVectorData.lastRoll) < 160 || abs(gameRotationVectorData.lastRoll) > 178){
        printf("ESTD:%+07.1f\n",gameRotationVectorData.lastRoll); // bell not at stand
        return(0);
    }
    
    if(gameRotationVectorData.lastRoll < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        gameRotationVectorData.direction = -1;
        gameRotationVectorData.angleBuffer[0] = -180.0-gameRotationVectorData.lastRoll;
        gyroData.direction = -1;
        
    } else {
        gameRotationVectorData.direction = +1;
        gameRotationVectorData.angleBuffer[0] = gameRotationVectorData.lastRoll-180.0;
        gyroData.direction = +1;
    }
    
    for(int i = 0; i< BUFFERSIZE; ++i){ // initialise with dummy data for savgol alignment
        gyroData.rateBuffer[i] = 0;
        gameRotationVectorData.angleBuffer[i] = gameRotationVectorData.angleBuffer[0];
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

// We don't actually know that the reports will be sent at the same rate
// This is some basic fifo alignment to stop things getting too far out
// of whack
void alignFIFOs(void){
    if(gyroData.available >= 3){
        if ((gyroData.available - 3) > gameRotationVectorData.available){
                gyroData.tail = (gyroData.tail + 1) % BUFFERSIZE; // ditch a sample
                gyroData.available -= 1;
                printf("Ditched rate sample\n");
        }
    }

    if(gameRotationVectorData.available >= 3){
        if ((gameRotationVectorData.available - 3) > gyroData.available){
                gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) % BUFFERSIZE; // ditch a sample
                gameRotationVectorData.available -= 1;
                printf("Ditched angle sample\n");
        }
    }
}

void pushData(void){
    static char outputLocal[2048];
    static char outputRemote[2048];
    static char outputLine[100];
    int outputCountLocal = 0;
    int outputCountRemote = 0;
    if(gyroData.available < PUSHBATCH + SAVGOLLENGTH) return;
    if(gameRotationVectorData.available < PUSHBATCH + SAVGOLLENGTH) return;

    for(int counter = 0; counter < PUSHBATCH; ++counter){
        sprintf(outputLine,"A:%+07.1f,R:%+07.1f,C:%+07.1f,W:%+07.1f\n", 
            gameRotationVectorData.angleBuffer[gameRotationVectorData.tail], 
            gyroData.rateBuffer[gyroData.tail],
            savGol(gyroData.tail),
            gyroData.accnBuffer[gyroData.tail]);
        outputCountLocal += sprintf(&outputLocal[outputCountLocal],outputLine);
        outputCountRemote += sprintf(&outputRemote[outputCountRemote],"LIVE:%s", outputLine);
        OUT_COUNT += 1;
        gameRotationVectorData.tail = (gameRotationVectorData.tail + 1) % BUFFERSIZE;
        gyroData.tail = (gyroData.tail + 1) % BUFFERSIZE;
    }
    gameRotationVectorData.available -= PUSHBATCH;
    gyroData.available -= PUSHBATCH;
    
    printf("%s",outputRemote);
    fputs(outputLocal,fd_write_out);
    fflush(fd_write_out);
}

// smooths and differenciates gryo readings to make smoothed accelerations
float savGol(unsigned int startPosition){
    unsigned int windowStartPosition = (startPosition < SAVGOLHALF) ? (startPosition + BUFFERSIZE) - SAVGOLHALF: startPosition - SAVGOLHALF;
    float savGolResult=0;

    for(int i = 0; i < SAVGOLLENGTH; ++i){
        savGolResult += ODR*(savGolCoefficients[i] * gyroData.rateBuffer[(windowStartPosition +i) % BUFFERSIZE]);
    }
    return(savGolResult);
}

void parseEvent(void){
    if(spiRead.channel == CHANNEL_REPORTS) {
        switch(spiRead.buffer[5]) {
            case SENSOR_REPORTID_GAME_ROTATION_VECTOR:                  parseGameRotationVector(); break;
            case SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR:       parseStabilisedRotationVector(); break;
            case SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR:  parseStabilisedGameRotationVector(); break;
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:                  parseCalibratedGyroscope(); break;
            case SENSOR_REPORTID_MAGNETIC_FIELD:                        parseCalibratedMagneticField(); break;
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
        case SENSOR_REPORTID_MAGNETIC_FIELD:   
            calibratedMagneticFieldData.reportInterval = reportInterval;
            if(calibratedMagneticFieldData.reportInterval != calibratedMagneticFieldData.requestedInterval) printf("Warning: Magfield data rate mismatch. Req: %d, Rep: %d\n", calibratedMagneticFieldData.requestedInterval,reportInterval);
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
        case 0x84: 
          printf("Error: device has reinitialised!\n");
          CALIBRATING = 0;
          runtimeReorient(0,0,0,0);
          setStandardOrientation();
          for(uint32_t i=0; i<sizeof(SEQUENCENUMBER); i++) SEQUENCENUMBER[i] = 0;
          start(ODR);
          break;  // bit 7 set indicates autonomous response 0x04 = Initialisation.  Sometimes caused by too much data being pushed.
        case 0x06: printf("Save DCD.  Success: %d\n", spiRead.buffer[5] ); break;
        case 0xFD: printf("FRS Write response\n"); break;  // is this correct???
        case COMMAND_ME_CALIBRATE: break;
        default: printf("Unhandled CR event: CHANNEL: %02X, SEQUENCE: %02X BYTES: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n", spiRead.channel, spiRead.sequence, spiRead.buffer[0], spiRead.buffer[1], spiRead.buffer[2], spiRead.buffer[3], spiRead.buffer[4], spiRead.buffer[5], spiRead.buffer[6], spiRead.buffer[7], spiRead.buffer[8]);
    }
}

void parseGyroIntegratedRotationVector(void){
    // https://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
    // 200Hz sample rate 10Hz low pass == 400hz 20Hz low pass
    static const double butterworthMagicNumber1 = 49.79245121;
    static const double butterworthMagicNumber2 = -0.6413515381;
    static const double butterworthMagicNumber3 = 1.5610180758;

    // 400Hz sample rate 10Hz low pass
//    static const double butterworthMagicNumber1 = 180.4169259;
//    static const double butterworthMagicNumber2 = -0.8008026467;
//    static const double butterworthMagicNumber3 = 1.7786317778;
    
    // 400Hz sample rate 5Hz low pass
//    static const double butterworthMagicNumber1 = 180.4169259;
//    static const double butterworthMagicNumber2 = -0.8008026467;
//    static const double butterworthMagicNumber3 = 1.7786317778;
    

    float Qx = (spiRead.buffer[1] << 8) + spiRead.buffer[0];
    float Qy = (spiRead.buffer[3] << 8) + spiRead.buffer[2];
    float Qz = (spiRead.buffer[5] << 8) + spiRead.buffer[4];
    float Qw = (spiRead.buffer[7] << 8) + spiRead.buffer[6];
    
    if(Qx >= 0x8000) Qx -= 0x10000;
    if(Qy >= 0x8000) Qy -= 0x10000;
    if(Qz >= 0x8000) Qz -= 0x10000;
    if(Qw >= 0x8000) Qw -= 0x10000;

    Qx *= QP(14);
    Qy *= QP(14);
    Qz *= QP(14);
    Qw *= QP(14);

// possibly did not transcribe this quaternion to angle routine well - to be checked

    float r31 = 2.0 * (Qx * Qz - Qw * Qy);
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    if(abs(r31) != 1.0){
        pitch = -asin(r31);
        roll = atan2((2.0*(Qy * Qz - Qw * Qx)) / cos(pitch), (1.0 - (2.0*(Qx * Qx + Qy * Qy))) / cos(pitch));
        yaw =  atan2((2.0*(Qx * Qy - Qw * Qz)) / cos(pitch), (1.0 - (2.0*(Qy * Qy + Qz * Qz))) / cos(pitch));
    } else {
        yaw = 0;
        if(r31 == -1){
            pitch = 3.1415927/2.0;
            roll = atan2(2.0*(Qx * Qy + Qw * Qz),2.0*(Qx * Qz - Qw * Qy));
        } else {
            pitch = -3.1415927/2.0;
            roll = atan2(-2.0*(Qx * Qy + Qw * Qz),-2.0*(Qx * Qz - Qw * Qy));            
        }
    }

    roll *= RADIANS_TO_DEGREES_MULTIPLIER;
    pitch *= RADIANS_TO_DEGREES_MULTIPLIER;
    yaw *= RADIANS_TO_DEGREES_MULTIPLIER;

    roll -= gyroIntegratedRotationVectorData.tareValue; 

    float Gx = (float)((spiRead.buffer[9 ] << 8) + spiRead.buffer[8]);
    float Gy = (float)((spiRead.buffer[11] << 8) + spiRead.buffer[10]);
    float Gz = (float)((spiRead.buffer[13] << 8) + spiRead.buffer[12]);
    
    if(Gx >= 0x8000) Gx -= 0x10000;
    if(Gy >= 0x8000) Gy -= 0x10000;
    if(Gz >= 0x8000) Gz -= 0x10000;

    Gx *= QP(10) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gy *= QP(10) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gz *= QP(10) * RADIANS_TO_DEGREES_MULTIPLIER;
 
    float rolldiff = roll - gyroIntegratedRotationVectorData.lastRoll;
    if(rolldiff > 250.0) {
        rolldiff -= 360.0;
    } else if (rolldiff < -250.0) {
        rolldiff += 360.0;
    }
    gyroIntegratedRotationVectorData.smoothRoll = 0.95*gyroIntegratedRotationVectorData.smoothRoll + 0.05*roll;    
    gyroIntegratedRotationVectorData.smoothAngle = SMOOTHFACTOR*gyroIntegratedRotationVectorData.smoothAngle + (1-SMOOTHFACTOR)*(gyroIntegratedRotationVectorData.lastAngle + gyroIntegratedRotationVectorData.direction*rolldiff);
    gyroIntegratedRotationVectorData.lastAngle = gyroIntegratedRotationVectorData.lastAngle + gyroIntegratedRotationVectorData.direction*rolldiff;
    gyroIntegratedRotationVectorData.lastRoll = roll;
    gyroIntegratedRotationVectorData.lastPitch = pitch;
    gyroIntegratedRotationVectorData.lastYaw = yaw;

    float noPeakAccn = (Gx-gyroIntegratedRotationVectorData.lastX)*ODR*gyroIntegratedRotationVectorData.direction;
    if(noPeakAccn > 1000.0) noPeakAccn = 1000.0;
    if(noPeakAccn < -1000.0) noPeakAccn = -1000.0;

    gyroIntegratedRotationVectorData.smoothAccn = SMOOTHFACTOR*gyroIntegratedRotationVectorData.smoothAccn + (1-SMOOTHFACTOR)*noPeakAccn;
    
    gyroIntegratedRotationVectorData.lastX = Gx;
    gyroIntegratedRotationVectorData.lastY = Gy;
    gyroIntegratedRotationVectorData.lastZ = Gz;
    gyroIntegratedRotationVectorData.smoothRate = SMOOTHFACTOR*gyroIntegratedRotationVectorData.smoothRate + (1-SMOOTHFACTOR)*Gx*gyroIntegratedRotationVectorData.direction;

    gyroIntegratedRotationVectorData.angleXV[0] = gyroIntegratedRotationVectorData.angleXV[1];
    gyroIntegratedRotationVectorData.angleXV[1] = gyroIntegratedRotationVectorData.angleXV[2];
    gyroIntegratedRotationVectorData.angleXV[2] = gyroIntegratedRotationVectorData.smoothAngle / butterworthMagicNumber1;
    gyroIntegratedRotationVectorData.angleYV[0] = gyroIntegratedRotationVectorData.angleYV[1];
    gyroIntegratedRotationVectorData.angleYV[1] = gyroIntegratedRotationVectorData.angleYV[2];
    gyroIntegratedRotationVectorData.angleYV[2] = (gyroIntegratedRotationVectorData.angleXV[0] + gyroIntegratedRotationVectorData.angleXV[2]) +
        2 * gyroIntegratedRotationVectorData.angleXV[1] + ( butterworthMagicNumber2 * gyroIntegratedRotationVectorData.angleYV[0]) + 
        (  butterworthMagicNumber3 * gyroIntegratedRotationVectorData.angleYV[1]);
    
    gyroIntegratedRotationVectorData.rateXV[0] = gyroIntegratedRotationVectorData.rateXV[1];
    gyroIntegratedRotationVectorData.rateXV[1] = gyroIntegratedRotationVectorData.rateXV[2];
    gyroIntegratedRotationVectorData.rateXV[2] = gyroIntegratedRotationVectorData.smoothRate / butterworthMagicNumber1;
    gyroIntegratedRotationVectorData.rateYV[0] = gyroIntegratedRotationVectorData.rateYV[1];
    gyroIntegratedRotationVectorData.rateYV[1] = gyroIntegratedRotationVectorData.rateYV[2];
    gyroIntegratedRotationVectorData.rateYV[2] = (gyroIntegratedRotationVectorData.rateXV[0] + gyroIntegratedRotationVectorData.rateXV[2]) +
        2 * gyroIntegratedRotationVectorData.rateXV[1] + ( butterworthMagicNumber2 * gyroIntegratedRotationVectorData.rateYV[0]) + 
        (  butterworthMagicNumber3 * gyroIntegratedRotationVectorData.rateYV[1]);

    gyroIntegratedRotationVectorData.accnXV[0] = gyroIntegratedRotationVectorData.accnXV[1];
    gyroIntegratedRotationVectorData.accnXV[1] = gyroIntegratedRotationVectorData.accnXV[2];
    gyroIntegratedRotationVectorData.accnXV[2] = gyroIntegratedRotationVectorData.smoothAccn / butterworthMagicNumber1;
    gyroIntegratedRotationVectorData.accnYV[0] = gyroIntegratedRotationVectorData.accnYV[1];
    gyroIntegratedRotationVectorData.accnYV[1] = gyroIntegratedRotationVectorData.accnYV[2];
    gyroIntegratedRotationVectorData.accnYV[2] = (gyroIntegratedRotationVectorData.accnXV[0] + gyroIntegratedRotationVectorData.accnXV[2]) +
        2 * gyroIntegratedRotationVectorData.accnXV[1] + ( butterworthMagicNumber2 * gyroIntegratedRotationVectorData.accnYV[0]) + 
        (  butterworthMagicNumber3 * gyroIntegratedRotationVectorData.accnYV[1]);

    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gyroIntegratedRotationVectorData.available == (BUFFERSIZE - 2)) {
        printf("GIRV buffer full.  Ditching oldest sample\n");
        gyroIntegratedRotationVectorData.tail = (gyroIntegratedRotationVectorData.tail + 1) % BUFFERSIZE;
        gyroIntegratedRotationVectorData.available -= 1;
    }
            
    gyroIntegratedRotationVectorData.rateBuffer[gyroIntegratedRotationVectorData.head] = gyroIntegratedRotationVectorData.rateYV[2];
    gyroIntegratedRotationVectorData.accnBuffer[gyroIntegratedRotationVectorData.head] = gyroIntegratedRotationVectorData.accnYV[2];
    gyroIntegratedRotationVectorData.angleBuffer[gyroIntegratedRotationVectorData.head] = gyroIntegratedRotationVectorData.angleYV[2];
    gyroIntegratedRotationVectorData.head = (gyroIntegratedRotationVectorData.head + 1) % BUFFERSIZE;
    gyroIntegratedRotationVectorData.available += 1;
}

void parseGameRotationVector(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == gameRotationVectorData.lastSequence) return; // sometimes HINT is not brought high quickly, this checks to see if we are reading a report we have already seen.
    gameRotationVectorData.lastSequence = newSequence;

    gameRotationVectorData.status = spiRead.buffer[5 + 2] & 0x03;
   
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

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
// https://forums.adafruit.com/viewtopic.php?t=131484
// https://mbientlab.com/community/discussion/2036/taring-quaternion-attitude
// http://www.gregslabaugh.net/publications/euler.pdf
// https://math.stackexchange.com/questions/687964/getting-euler-tait-bryan-angles-from-quaternion-representation
/*
// possibly did not transcribe this well - to be checked
    float r31 = 2.0 * (Qx * Qz - Qw * Qy);
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    if(abs(r31) != 1.0){
        pitch = -asin(r31);
        roll = atan2((2.0*(Qy * Qz - Qw * Qx)) / cos(pitch), (1.0 - (2.0*(Qx * Qx + Qy * Qy))) / cos(pitch));
        yaw =  atan2((2.0*(Qx * Qy - Qw * Qz)) / cos(pitch), (1.0 - (2.0*(Qy * Qy + Qz * Qz))) / cos(pitch));
    } else {
        yaw = 0;
        if(r31 == -1){
            pitch = 3.1415927/2.0;
            roll = atan2(2.0*(Qx * Qy + Qw * Qz),2.0*(Qx * Qz - Qw * Qy));
        } else {
            pitch = -3.1415927/2.0;
            roll = atan2(-2.0*(Qx * Qy + Qw * Qz),-2.0*(Qx * Qz - Qw * Qy));            
        }
    }

    roll *= RADIANS_TO_DEGREES_MULTIPLIER;
    pitch *= RADIANS_TO_DEGREES_MULTIPLIER;
    yaw *= RADIANS_TO_DEGREES_MULTIPLIER;
*/
    float yaw   =  atan2(Qx * Qy + Qw * Qz, (Qw * Qw + Qx * Qx) - 0.5) * RADIANS_TO_DEGREES_MULTIPLIER;   
    float pitch = -asin(2.0 * (Qx * Qz - Qw * Qy)) * RADIANS_TO_DEGREES_MULTIPLIER;
    float roll  =  atan2(Qw * Qx + Qy * Qz, (Qw * Qw + Qz * Qz) - 0.5) * RADIANS_TO_DEGREES_MULTIPLIER;

    roll -= gameRotationVectorData.tareValue; 

    float rolldiff = roll - gameRotationVectorData.lastRoll;
    if(rolldiff > 250.0) {
        rolldiff -= 360.0;
    } else if (rolldiff < -250.0) {
        rolldiff += 360.0;
    }

    gameRotationVectorData.lastRoll = roll;
    gameRotationVectorData.lastPitch = pitch;
    gameRotationVectorData.lastYaw = yaw;
    
    if(CALIBRATING) {
        gameRotationVectorData.calibrationCount += 1;
        if((gameRotationVectorData.calibrationCount % PUSHBATCH) == 0) {
            printf("CALI:%d, %+07.1f, %+07.1f, %+07.1f\n", gameRotationVectorData.status, gameRotationVectorData.lastRoll, gameRotationVectorData.lastPitch, gameRotationVectorData.lastYaw);
        }
    }

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
}

void parseLinearAccelerometer(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == linearAccelerometerData.lastSequence) return; // sometimes HINT is not brought high quickly, this checks to see if we are reading a report we have already seen.
    linearAccelerometerData.lastSequence = newSequence;

    linearAccelerometerData.status = spiRead.buffer[5 + 2] & 0x03;
    
    float Ax = (spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4];
         float Ay = (spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6];
    float Az = (spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8];
    
    if(Ax >= 0x8000) Ax -= 0x10000;
    if(Ay >= 0x8000) Ay -= 0x10000;
    if(Az >= 0x8000) Az -= 0x10000;

    Ax *= QP(8);
    Ay *= QP(8);
    Az *= QP(8);

    linearAccelerometerData.lastX = Ax;
    linearAccelerometerData.lastY = Ay;
    linearAccelerometerData.lastZ = Az;
}

void parseAccelerometer(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == accelerometerData.lastSequence) return; // sometimes HINT is not brought high quickly, this checks to see if we are reading a report we have already seen.
    accelerometerData.lastSequence = newSequence;

    accelerometerData.status = spiRead.buffer[5 + 2] & 0x03;
    
    float Ax = (spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4];
    float Ay = (spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6];
    float Az = (spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8];
    
    if(Ax >= 0x8000) Ax -= 0x10000;
    if(Ay >= 0x8000) Ay -= 0x10000;
    if(Az >= 0x8000) Az -= 0x10000;

    Ax *= QP(8);
    Ay *= QP(8);
    Az *= QP(8);

    accelerometerData.lastX = Ax;
    accelerometerData.lastY = Ay;
    accelerometerData.lastZ = Az;
}

void parseCalibratedGyroscope(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == gyroData.lastSequence) return;
    gyroData.lastSequence = newSequence;

    gyroData.status = spiRead.buffer[5 + 2] & 0x03;

    float Gx = (spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4];
    float Gy = (spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6];
    float Gz = (spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8];
    
    if(Gx >= 0x8000) Gx -= 0x10000;
    if(Gy >= 0x8000) Gy -= 0x10000;
    if(Gz >= 0x8000) Gz -= 0x10000;

    Gx *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gy *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;
    Gz *= QP(9) * RADIANS_TO_DEGREES_MULTIPLIER;

    float rawAccn = (Gx-gyroData.lastX)*ODR*gyroData.direction;

    gyroData.lastX = Gx;
    gyroData.lastY = Gy;
    gyroData.lastZ = Gz;
    
    if(!RUNNING) return;  // only push data to fifo if we are on a run

    if (gyroData.available == (BUFFERSIZE - 2)) {
        printf("Rate buffer full.  Ditching oldest sample\n");
        gyroData.tail = (gyroData.tail + 1) % BUFFERSIZE;
        gyroData.available -= 1;
    }
    
    gyroData.rateBuffer[gyroData.head] = Gx * gyroData.direction;
    gyroData.accnBuffer[gyroData.head] = rawAccn;
    gyroData.head = (gyroData.head + 1) % BUFFERSIZE;
    gyroData.available += 1;
    
}

void parseStability(void){    
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == stabilityData.lastSequence) return;
    stabilityData.lastSequence = newSequence;

    stabilityData.status = spiRead.buffer[5+4];
}

void parseCalibratedMagneticField(void){
    uint8_t newSequence = spiRead.buffer[5 + 1];
    if(newSequence == calibratedMagneticFieldData.lastSequence) return; 
    calibratedMagneticFieldData.lastSequence = newSequence;

    calibratedMagneticFieldData.status = spiRead.buffer[5 + 2] & 0x03;
    
    float Mx = (spiRead.buffer[5 + 5] << 8) + spiRead.buffer[5 + 4];
    float My = (spiRead.buffer[5 + 7] << 8) + spiRead.buffer[5 + 6];
    float Mz = (spiRead.buffer[5 + 9] << 8) + spiRead.buffer[5 + 8];
    
    if(Mx >= 0x8000) Mx -= 0x10000;
    if(My >= 0x8000) My -= 0x10000;
    if(Mz >= 0x8000) Mz -= 0x10000;

    Mx *= QP(8) * 100;
    My *= QP(8) * 100;
    Mz *= QP(8) * 100;

    calibratedMagneticFieldData.lastX = Mx;
    calibratedMagneticFieldData.lastY = My;
    calibratedMagneticFieldData.lastZ = Mz;
}

void parseStabilisedRotationVector(void){
    printf("Not yet\n");
}

void parseStabilisedGameRotationVector(void){
    printf("Not yet\n");
}

void setStandardOrientation(void){
//    reorient(0,0,0,0);
    reorient(R2O2,0,-R2O2,0);  // side mount connectors down
//    reorient(0,-R2O2,0,-R2O2);  // side mount connectors up
}

int reorient(float w, float x, float y, float z){
    uint32_t W = (int32_t)round(w/QP(30));
    uint32_t X = (int32_t)round(x/QP(30));
    uint32_t Y = (int32_t)round(y/QP(30));
    uint32_t Z = (int32_t)round(z/QP(30));

    if(readFrsRecord(SYSTEM_ORIENTATION) && FRS_READ_BUFFER[0] == X && FRS_READ_BUFFER[1] == Y && FRS_READ_BUFFER[2] == Z && FRS_READ_BUFFER[3] == W ) return(1);
    FRS_WRITE_BUFFER[0]=X;
    FRS_WRITE_BUFFER[1]=Y;
    FRS_WRITE_BUFFER[2]=Z;
    FRS_WRITE_BUFFER[3]=W;
    writeFrsRecord(SYSTEM_ORIENTATION,4);
// if we write the record then reinitialise the sensors
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++;
    spiWrite.buffer[2] = 0x04; // Initialize command
    spiWrite.buffer[3] = 0x01; // reinitialise everything
    spiWrite.buffer[4] = 0x00;
    spiWrite.buffer[5] = 0x00;
    spiWrite.buffer[6] = 0x00;
    spiWrite.buffer[7] = 0x00;
    spiWrite.buffer[8] = 0x00;
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    sendPacket(CHANNEL_CONTROL, 12);
    for(int i = 0; i<2000; i++){
        usleep(100);
        while(bcm2835_gpio_lev(INTGPIO) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_COMMAND_RESPONSE) {parseEvent(); continue;}
            if(spiRead.buffer[2] != COMMAND_INITIALIZE) {parseEvent(); continue;}
            if(spiRead.buffer[5] != 0) {
                return(0);
            } else {
                return(1);
            }
        }
    }
    printf("Initialise timeout\n");
    return(0);
}


int runtimeReorient(float w, float x, float y, float z){
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

void clearPersistentTare(void){
    eraseFrsRecord(SYSTEM_ORIENTATION);
    runtimeReorient(0,0,0,0);
    setStandardOrientation();
}

void tareZ(void){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; 
    spiWrite.buffer[2] = 0x03; // Tare command
    spiWrite.buffer[3] = 0x00; // perform Tare now
    spiWrite.buffer[4] = 0x04; // Z axis
    spiWrite.buffer[5] = 0x01; // use gaming rotation vector
    spiWrite.buffer[6] = 0x00; // reserved
    spiWrite.buffer[7] = 0x00;
    spiWrite.buffer[8] = 0x00;
    spiWrite.buffer[9] = 0x00;
    spiWrite.buffer[10]= 0x00;
    spiWrite.buffer[11]= 0x00;
    sendPacket(CHANNEL_CONTROL, 12);
}

void tare(void){
    spiWrite.buffer[0] = SHTP_REPORT_COMMAND_REQUEST; // set feature
    spiWrite.buffer[1] = SEQUENCENUMBER[6]++; 
    spiWrite.buffer[2] = 0x03; // Tare command
    spiWrite.buffer[3] = 0x00; // perform Tare now
    spiWrite.buffer[4] = 0x07; // all axes
    spiWrite.buffer[5] = 0x01; // use game rotation vector
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

void setupCalibration(char accel, char gyro, char mag){
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
        while(bcm2835_gpio_lev(INTGPIO) == 0) {
            collectPacket();
            if(spiRead.buffer[2] != 0x06) {parseEvent(); continue;}
            if(spiRead.buffer[5] != 0) printf("Calibration save error\n");
            return;
        }
    }
    printf("Calibration save timeout\n");
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
    while(timeout < 2000){
        timeout +=1;
        usleep(50);
        while(bcm2835_gpio_lev(INTGPIO) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_READ_RESPONSE) {parseEvent(); continue;}
            if(((spiRead.buffer[1] & 0x0F) == 1) || ((spiRead.buffer[1] & 0x0F) == 2) || ((spiRead.buffer[1] & 0x0F) == 4) || \
                ((spiRead.buffer[1] & 0x0F) == 5) || ((spiRead.buffer[1] & 0x0F) == 8)) {
                printf("FRS record read error, %02X\n",spiRead.buffer[1] & 0x0F);
                return(0);
            }
            FRS_READ_BUFFER[counter]   = (uint32_t)spiRead.buffer[7] << 24 | (uint32_t)spiRead.buffer[6] << 16 | (uint32_t)spiRead.buffer[5] << 8 | (uint32_t)spiRead.buffer[4];
            FRS_READ_BUFFER[counter+1] = (uint32_t)spiRead.buffer[11] << 24 | (uint32_t)spiRead.buffer[10] << 16 | (uint32_t)spiRead.buffer[9] << 8 | (uint32_t)spiRead.buffer[8];
            counter += 2;
            if(((spiRead.buffer[1] & 0x0F) == 3) || ((spiRead.buffer[1] & 0x0F) == 6) || ((spiRead.buffer[1] & 0x0F) == 7)) return(counter);
        }
    }
    printf("FRS Read timeout");
    return(0);
}

void printGyroIntegratedRotationVectorFRS(void){
    if(!readFrsRecord(GYRO_INTEGRATED_RV_CONFIG)){
        printf("Read error\n");
        return;
    }
    printf("Reference data type (0x0204=9 Axis, 0x0207=6 Axis): %08X\n", FRS_READ_BUFFER[0]);
    if(FRS_READ_BUFFER[1] != 0) printf("Synchronisation interval (Hz): %d\n", 1000000/FRS_READ_BUFFER[1]);
    printf("Maximum Error (Degrees): %03.1f\n", FRS_READ_BUFFER[2]*QP(29)*RADIANS_TO_DEGREES_MULTIPLIER);
    printf("Prediction amount: %f\n", FRS_READ_BUFFER[3]*QP(10));
    printf("Alpha: %f\n", FRS_READ_BUFFER[4]*QP(20));
    printf("Beta: %f\n", FRS_READ_BUFFER[5]*QP(20));
    printf("Gamma: %f\n", FRS_READ_BUFFER[6]*QP(20));
}

// axes is one of 0x0207 (for 6 axis) or 0x0204 (for 9 axis).  9 Axis is just used for calibration
// rate is the rate used to update the GIRV from the more accurate rotaton vector.  It is in Hz 
void setGyroIntegratedRotationVectorFRS(uint32_t axes, uint32_t rate, uint32_t maxError){
    if((axes != 0x0207) && (axes !=  0x0204)) return;
    if (rate == 0) return;
    readFrsRecord(GYRO_INTEGRATED_RV_CONFIG);
    if((FRS_READ_BUFFER[0] == axes) && (FRS_READ_BUFFER[1] == 1000000/rate) && (round(FRS_READ_BUFFER[2]*QP(29)*RADIANS_TO_DEGREES_MULTIPLIER) == maxError)) return; // nothing to do
    FRS_WRITE_BUFFER[0]=axes;
    FRS_WRITE_BUFFER[1]=1000000/rate;
    FRS_WRITE_BUFFER[2]=maxError*DEGREES_TO_RADIANS_MULTIPLIER/QP(29);
    FRS_WRITE_BUFFER[3]=0; // prediction parameter
    FRS_WRITE_BUFFER[4]=round(0.303072543909142/QP(20));  // alpha
    FRS_WRITE_BUFFER[5]=round(0.113295896384921/QP(20));  // beta
    FRS_WRITE_BUFFER[6]=round(0.002776219713054/QP(20));  //gamma
    FRS_WRITE_BUFFER[7]=0; // padding
    writeFrsRecord(GYRO_INTEGRATED_RV_CONFIG,8);
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

    writeFrsRecordWords(offset, data, 0);
    
    for(int i = 0; i<2000; i++){
        usleep(100);
        while(bcm2835_gpio_lev(INTGPIO) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_WRITE_RESPONSE) {parseEvent(); continue;}
            if(spiRead.buffer[1] != 3) {
                continue;
            } else {
                return(1);
            }
        }
    }
    printf("FRS Write timeout\n");
    return(0);
}

int writeFrsRecord(uint16_t recordType, uint32_t length){
    spiWrite.buffer[0] = SHTP_REPORT_FRS_WRITE_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = length & 0x00FF;//length lsb 
    spiWrite.buffer[3] = length >> 8;    //length msb
    spiWrite.buffer[4] = recordType & 0x00FF;
    spiWrite.buffer[5] = recordType >> 8;
    if (!sendPacket(CHANNEL_CONTROL, 6)) return(0);

    for(int i=0; i<length; i+=2){
        writeFrsRecordWords(i,FRS_WRITE_BUFFER[i], FRS_WRITE_BUFFER[i+1]);
    }
    
    for(int i = 0; i<2000; i++){
        usleep(100);
        while(bcm2835_gpio_lev(INTGPIO) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_WRITE_RESPONSE) {parseEvent(); continue;}
            if(spiRead.buffer[1] != 3) {
                continue;
            } else {
                return(1);
            }
        }
    }
    printf("FRS Write timeout\n");
    return(0);
}

int writeFrsRecordWords(uint32_t offset, uint32_t data0, uint32_t data1){
    spiWrite.buffer[0] = SHTP_REPORT_FRS_WRITE_DATA_REQUEST;
    spiWrite.buffer[1] = 0x00;
    spiWrite.buffer[2] = offset & 0x00FF;  //offset lsb
    spiWrite.buffer[3] = offset >> 8;//offset msb
    spiWrite.buffer[4] =  data0 & 0x000000FF;       // word 1 (LSB)
    spiWrite.buffer[5] = (data0 & 0x0000FF00) >> 8;
    spiWrite.buffer[6] = (data0 & 0x00FF0000) >> 16;
    spiWrite.buffer[7] = data0 >> 24; // word 1 (MSB)
    spiWrite.buffer[8] = data1 & 0x000000FF;  // word 2 lsb
    spiWrite.buffer[9] = (data1 & 0x0000FF00) >> 8;
    spiWrite.buffer[10]= (data1 & 0x00FF0000) >> 16;
    spiWrite.buffer[11]= data1 >> 24;
    if(!sendPacket(CHANNEL_CONTROL, 12)) return(0);
    return(1);
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
    if (!sendPacket(CHANNEL_CONTROL, 8)) return(0);

    int timeout = 0;
    while(timeout < 2000){
        timeout +=1;
        usleep(50);
        while(bcm2835_gpio_lev(INTGPIO) == 0) {
            collectPacket();
            if(spiRead.buffer[0] != SHTP_REPORT_FRS_READ_RESPONSE) {parseEvent(); continue;}
            if(((spiRead.buffer[1] & 0x0F) == 1) || ((spiRead.buffer[1] & 0x0F) == 2) || ((spiRead.buffer[1] & 0x0F) == 4) || \
                ((spiRead.buffer[1] & 0x0F) == 5) || ((spiRead.buffer[1] & 0x0F) == 8)) {
                printf("FRS Read error, %02X\n",spiRead.buffer[1] & 0x0F);
                *result = 0;
                return(0);
            }
            *result = ((uint32_t)spiRead.buffer[7] << 24) | ((uint32_t)spiRead.buffer[6] << 16) | ((uint32_t)spiRead.buffer[5] << 8) | (uint32_t)spiRead.buffer[4];
            if(((spiRead.buffer[1] & 0x0F) == 3) || ((spiRead.buffer[1] & 0x0F) == 6) || ((spiRead.buffer[1] & 0x0F) == 7)) return(1);
        }
    }
    printf("FRS Read timeout\n");
    *result = 0;
    return(0);
}

void start(uint32_t dataRate){ // set datarate to zero to stop sensors
    uint32_t odrPeriodMicrosecs = 0;
    if(dataRate != 0) odrPeriodMicrosecs = 1000000/dataRate; 

// enable calibration (we should be stationary)  // TODO: check stationary
    setupCalibration(1,1,1);

//    configureFeatureReport(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, odrPeriodMicrosecs);
//    gyroIntegratedRotationVectorData.requestedInterval=odrPeriodMicrosecs;
//    gyroIntegratedRotationVectorData.direction = 0;

    configureFeatureReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, 500000);// two reports per second
    stabilityData.requestedInterval=500000;

    configureFeatureReport(SENSOR_REPORTID_GAME_ROTATION_VECTOR, odrPeriodMicrosecs);
    gameRotationVectorData.requestedInterval=odrPeriodMicrosecs;
    gameRotationVectorData.direction=0;
    
    configureFeatureReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, odrPeriodMicrosecs);
    gyroData.requestedInterval=odrPeriodMicrosecs;

/*
    configureFeatureReport(SENSOR_REPORTID_ACCELEROMETER, odrPeriodMicrosecs);
    accelerometerData.requestedInterval=odrPeriodMicrosecs;

    configureFeatureReport(SENSOR_REPORTID_LINEAR_ACCELERATION, odrPeriodMicrosecs);
    linearAccelerometerData.requestedInterval=odrPeriodMicrosecs;
*/
}

void setupStabilityClassifierFrs(float threshold){
    uint32_t result = (int32_t)round(threshold/QP(24));
    readFrsRecord(STABILITY_DETECTOR_CONFIG);
    if((FRS_READ_BUFFER[0] == result) && (FRS_READ_BUFFER[1] == 500000)) return; // nothing to do
    FRS_WRITE_BUFFER[0]=threshold;
    FRS_WRITE_BUFFER[1]=500000;
    writeFrsRecord(STABILITY_DETECTOR_CONFIG,2);
}

void setup(void){
    if (!bcm2835_init()){
        printf("Unable to inititalise bcm2835\n");
        exit(1);
    }
    bcm2835_gpio_fsel(RESETGPIO, BCM2835_GPIO_FSEL_OUTP); // reset
    bcm2835_gpio_set(RESETGPIO);
    bcm2835_gpio_fsel(WAKPS0GPIO, BCM2835_GPIO_FSEL_OUTP); // WAKPS0
    bcm2835_gpio_set(WAKPS0GPIO);
    bcm2835_gpio_fsel(INTGPIO, BCM2835_GPIO_FSEL_INPT); // INT
    bcm2835_gpio_set_pud(INTGPIO, BCM2835_GPIO_PUD_UP);
//  setup SPI
    if (!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. \n");
      exit(1);
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    bcm2835_spi_set_speed_hz(3000000);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
//  reset device
    bcm2835_gpio_clr(RESETGPIO);
    usleep(20000);
    bcm2835_gpio_set(RESETGPIO);
    int waitCount = 0;
    while ((bcm2835_gpio_lev(INTGPIO) == 1) && (waitCount < 1000)){ // should only be called when int is low so not really needed
        usleep(500);
        waitCount += 1;
    }
    if(waitCount == 1000)  {printf("EIMU: Device did not wake on reset\n");}
    if(!collectPacket()) {printf("EIMU: Reset packet not received\n");}

    usleep(5000);
    if(!collectPacket()) {printf("EIMU:Unsolicited packet not received\n");}

    spiWrite.buffer[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    spiWrite.buffer[1] = 0; //Reserved
    if(!sendPacket(CHANNEL_CONTROL, 2)) {printf("EIMU:Cannot send ID request\n");}

    if(!collectPacket()) {printf("EIMU:Cannot receive ID response");}
    
    if(spiRead.buffer[0] != SHTP_REPORT_PRODUCT_ID_RESPONSE){
        printf("EIMU: Cannot communicate with BNO080: got %02X DATALENGTH %04X \n",spiRead.buffer[0],spiRead.dataLength);
        exit(1);
    }
    for(uint32_t i=0; i<sizeof(SEQUENCENUMBER); i++) SEQUENCENUMBER[i] = 0;
    setStandardOrientation(); 
    setupStabilityClassifierFrs(2.5);  // used to check if bell is moving.  2.5m/s2 allows some small movement

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

    while ((bcm2835_gpio_lev(INTGPIO) == 1) && (waitCount < 100)){ // should only be called when int is low so not really needed
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
    if(spiRead.dataLength == 0) return(0); // nothing collected
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
    unsigned int header[4];
    int waitCount = 0;
    volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;
    volatile uint32_t* fifo = bcm2835_spi0 + BCM2835_SPI0_FIFO/4;
    uint32_t TXCnt = 0;
    uint32_t RXCnt = 0;
    spiRead.dataLength = 0;
    spiRead.sequence = 0;
    spiRead.channel = 9;

    bcm2835_gpio_clr(WAKPS0GPIO);  // wake the thing up
    dataLength += 4;    
    header[0] = dataLength & 0xFF;
    header[1] = dataLength >> 8;
    header[2] = channelNumber;
    header[3] = SEQUENCENUMBER[channelNumber]++;
    usleep(2000);  // makes things much more reliable - thankfully we are only sending commands infrequently

    while ((bcm2835_gpio_lev(INTGPIO) == 1) && (waitCount < 200)){
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

    bcm2835_gpio_set(WAKPS0GPIO);  // probably OK to remove wake signal now

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
