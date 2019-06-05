//gcc grabber.c -o grabber -lm -lbcm2835

/*
 * Copyright (c) 2017,2018,2019 Peter Budd. All rights reserved
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
*
* This program forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
* bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
* hardware is currently a Pi Zero running Arch Linux.
*
* The results of the "engineering" and "experimentation" work on the bell system are described here.
* In essence, although it will work straight from the command line, this program is intended to be started 
* by Websocketd https://github.com/joewalnes/websocketd
* On startup, it initialises the two IMUs (setup function) and starts pulling data
* from them (pullData function).  This data is not pushed to the browser until the user signals that it is starting a run.
* The data is always pulled as it allows for some gyro calibration (pullAndTransform function) and for the filters to settle down.
*  
* Two IMUs are used and their outputs are averaged.  Some time during the early development of this project, 
* this seemed to produce better results but given other improvements here, it may not be needed any longer.
* 
* The main loop:
*       *  checks the Arduino for battery life and reports this to the browser
*       *  checks stdin for commends from the browser (see below) and processes these commands
*       *  calls pushData to see if any data needs pushing to the browser (pushData calls pullData to pull data from the IMUs
* setup:
*       * configures the IMU device over SPI
*       * pulls calibration data from the Arduino over I2C
* pushData
*       * calls pullData
*       * if there is enough data in the circular buffer populated by pullData, it calculates angular acceleration using a Savitsky-Golay filter
*       * pushes angle, rate and acceleration to the browser
* savgol
*       * processes angular rate data to create angular acceleration data
* pullData is called by pushData
*       * checks to see if enough data is in the two IMUs' FIFOs to be pulled and processed
*       * checks to see if data from the two IMUs is aligned and if not ditches a sample from the faster IMU
*       * calls pullAndTransform to actually pull the data and process it into angles
*       * puts processed data (angle and angular rate) into a circular buffer
* pullAndTransform
*       * called by pullData, takes data from the IMUs (over SPI) averages the data from each of them and then adjusts for bias,scale and misalignment  
*       * checks to see if the bell has been stationary for a while and if it has it takes the opportunity to recalculate the gyro biasses
*       * calls calculate function to update rotation quaternion and Euler angles
* calculate
*       * called by pullAndTransform
*       * straightforward Mahony complementary filter as wriiten in quaternion form by Madgwick
* startRun
*       * called when the user has indicated that a recording should be started
*       * works out which way the bell went up (the device can be mounted either side of the headstock)
*       * initialises the circular buffer
*       * sets the RUNNING global to true (signals to pullData and pushData) 
* 
* There are the following possible command sent by the user's browser.  
* Commands are received on stdin and output is on stdout (which is what Websocketd expects):
* (a)   STRT:[filename] Start recording a session with a bell.  The bell
*       must be at stand and (reasonably) stationary otherwise this
*       will return ESTD: or EMOV: either of which signals the browser 
*       to stop expecting data.  If Filename is not provided then a
*       default filename of "CurrentRecording [date]" is used.
* (b)   STOP: stops a recording in progress.  This also saves off the
*       collected data into a file in /data/samples/
* (c)   LDAT: a request for data from the browser, only works when there
*       is a recording in progress - deprecated.  Does nothing.
* (d)   FILE: get a listing of the previous recordings stored in
*       /data/samples/ .  Used to display the selection box to the user.
* (e)   LOAD:[filename] used to transmit a previous recording to the browser
* (f)   DATE:[string] sets the date on the device to the date on the browser 
*             string is unixtime (microsecs since 1/1/70)
* (g)   SAMP: requests the current sample period
* (h)   SHDN: tells the device to shutdown
* (i)   TEST: returns the current basic orientation (for testing)
* (j)   EYEC: starts eye candy demo
* (k)   STEC: stops eye candy demo
* (l)   STND: standalone (playback-only) operation.
* (m)   PLRD: play and record operation.
* (n)   BELS: bell data. Format: numberofbells(int),CPM(float),openHandstroke(float), faketimefromBDC(float)
* (o)   EFIL: end sending of FILE: data
*
* There are the following responses back to the user's browser:
* (i)    STPD: tells the browser that the device has sucessfully stopped sampling
* (ii)   ESTP: signals an error in the stop process (an attempt to stopped when not started)
* (iii)  LIVE:[string] the string contains the data recorded by the device in the format
*             "A:[angle]R:[rate]C:[acceleration]".  Angle, rate and acceleration are 
*             ordinary floating point numbers.  Angle is in degrees, rate is in degrees/sec
*             and acceleration is in degrees/sec/sec.
* (iv)   NDAT: indicates that there is no current data to send to the browser (deprecated)
* (v)    SAMP: returns the sample period
* (vi)   EIMU: IMU not detected or some IMU related failure
* (vii)  ESTD: bell not at stand (aborts started session)
* (viii) EMOV: bell moving when session started (the bell has to be stationary at start)
* (ix)   ESTR: internal error related to data / filenames (shouldn't happen)!
* (x)    DATA:[string]  chunk of data from a previously stored file.  In same format as LIVE:
* (xi)   EOVF: fifo overflow flagged.
* (xii)  FILE:[filename] in response to FILE: a list of the previous recordings in /data/samples
* (xiii) STRT: in response to STRT: indicates a sucessful start.
* (xiv)  LFIN:[number] indicates the end of a file download and the number of samples sent
* (xv)   TEST:[number] current orientation
* (xvi)  BATT:[number] battery percentage
* (xvii) EYEC:[roll],[pitch].[yaw],[qw],[qx],[qy],[qz] returns angles and quaternion for eye candy demo 
* (xviii)EWAI: Wait for initial calibration (zeroing)
* (xix)  EPUL: Wait for bell to be rung up
* (xx)   ECAL: Bell being pulled up - recording for gravity calibration
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
#include <sys/time.h>
#include <errno.h>
#include <linux/types.h>
#include <stdint.h>
#include <dirent.h>
#include "ICM20689.h"
#include <bcm2835.h>

#ifndef NULL
#define NULL 0
#endif

// Manchester estimation.  Use gcalculator.xlsx to work out for your latitude.
#define g0 9.8137   

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 

FILE *fd_write_out;

// calculated by calculate function
float q0 = 1;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

float direction = 0;

int ODR = 125;
int RUNNING = 0;
int EYECANDY = 0;
int OUT_COUNT = 0;

unsigned char SPI_BUFFER[256];
unsigned char I2C_BUFFER[16];

char READ_OUTBUF[1500];
char READ_OUTBUF_LINE[150];
int READ_OUTBUF_COUNT;
char local_outbuf[4000];
int local_count = 0;

char FILENAME[50];

const unsigned int LOOPSLEEP = 50000;  // main loop processes every 0.05 sec.  25 samples should be in FIFO.  Max number of samples in FIFO is 341.  Should be OK.
unsigned int LOOPCOUNT = 0;

struct {
    float samplePeriod;
    float accBiasX;
    float accBiasY;
    float accBiasZ;
    float accTransformMatrix[9];
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;
    float gyroTransformMatrix[9];
    float tareValue;
    unsigned int torn;
    float *gravityData;
    unsigned int gravityDataCount;
    float gravityValue;
    unsigned int gravityCalibrationState;
    unsigned int stable;
    unsigned int lastStrikeCount;
//    float dingTimeFromBDC;
//    float dongTimeFromBDC;
    unsigned int lastBDC;
//    float dingAngleFromBDC;
//    float dongAngleFromBDC;
//    unsigned int dingNumber;
//    unsigned int dongNumber;
    unsigned int faked;
    unsigned int fakeTimefromBDC;
    unsigned int bellNumber;
    float CPM;
    float openHandstroke;
    int wayup;
} calibrationData = {.samplePeriod = 0.002, .accBiasX=0, .accBiasY=0, .accBiasZ = 0,
                    .accTransformMatrix[0]=1, .accTransformMatrix[1]=0, .accTransformMatrix[2]=0, .accTransformMatrix[3]=0,
                    .accTransformMatrix[4]=1, .accTransformMatrix[5]=0, .accTransformMatrix[6]=0, .accTransformMatrix[7]=0,
                    .accTransformMatrix[8]=1, .gyroBiasX=0, .gyroBiasY=0, .gyroBiasZ=0, .gyroTransformMatrix[0]=1, 
                    .gyroTransformMatrix[1]=0, .gyroTransformMatrix[2]=0, .gyroTransformMatrix[3]=0, .gyroTransformMatrix[4]=1,
                    .gyroTransformMatrix[5]=0, .gyroTransformMatrix[6]=0, .gyroTransformMatrix[7]=0, .gyroTransformMatrix[8]=1,
                    .tareValue = 0.0 , .torn = 0, .gravityValue = 0.0, .gravityCalibrationState = 0, .gravityDataCount = 0, 
                    .stable = 0, .bellNumber = 6, .CPM = 31.0, .openHandstroke = 1.0, .faked = 0, .fakeTimefromBDC = 300, .wayup=1};

// only really important when run from command line
volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

// used in relation to circular buffer
#define PUSHBATCH 20 // number of samples taken before pushing out
#define BUFFERSIZE 256 // must be bigger than PUSHBATCH + SAVGOLLENGTH
unsigned int head;
unsigned int tail;
unsigned int available;
float angleBuffer[BUFFERSIZE];
float rateBuffer[BUFFERSIZE];

#define SAVGOLLENGTH  15
#define SAVGOLHALF 7 // = math.floor(SAVGOLLENGTH/2.0)

// savgol_coeffs(15,2,deriv=1,use="dot")
const float savGolCoefficients[] = {
    -2.50000000e-02, -2.14285714e-02, -1.78571429e-02, -1.42857143e-02,
    -1.07142857e-02, -7.14285714e-03, -3.57142857e-03,  2.95770353e-16,
    3.57142857e-03,  7.14285714e-03,  1.07142857e-02,  1.42857143e-02,
    1.78571429e-02,  2.14285714e-02,  2.50000000e-02};

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
 
    getSavedData();
    setup();

    while(!sig_exit){
        usleep(LOOPSLEEP);
		LOOPCOUNT += 1;
		if(LOOPCOUNT == 1 || !(LOOPCOUNT % 200)){ // check every 10 secs but also the first time through
			I2C_BUFFER[0]=2;
			bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 2 (power)
			bcm2835_i2c_read(I2C_BUFFER,2); usleep(100);
			unsigned int result = (I2C_BUFFER[0]<<8)+I2C_BUFFER[1];
			if(result & 0x8000) system("/usr/bin/sync && /usr/bin/shutdown -P now");
			if((result & 0x7FFF) > 519){
				printf("BATT:0\n");
			} else if((result & 0x7FFF) > 509){
				printf("BATT:10\n");
			} else if((result & 0x7FFF) > 503){
				printf("BATT:20\n");
			} else if((result & 0x7FFF) > 500){
				printf("BATT:30\n");
			} else if((result & 0x7FFF) > 495){
				printf("BATT:40\n");
			} else if((result & 0x7FFF) > 490){
				printf("BATT:50\n");
			} else if((result & 0x7FFF) > 482){
				printf("BATT:60\n");
			} else if((result & 0x7FFF) > 475){
				printf("BATT:70\n");
			} else if((result & 0x7FFF) > 466){
				printf("BATT:80\n");
			} else if((result & 0x7FFF) > 456){
				printf("BATT:90\n");
			} else {
				printf("BATT:100\n");
			}
		}
      
        if(calibrationData.gravityCalibrationState == 4){
            pushData();
        } else {
            doCalibration();
        }

        if(EYECANDY) printf("EYEC:%+06.1f, %+06.1f, %+06.1f, %+07.4f, %+07.4f, %+07.4f, %+07.4f\n", roll, pitch, yaw, q0, q1, q2, q3);

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
                printf("SAMP:%f\n",calibrationData.samplePeriod*4.0);
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
                printf("EFIL:\n");
                }
                continue;
            }
            if(strcmp("TEST:", command) == 0) {
				printf("TEST: R:%f P:%f Y:%f\n",roll,pitch,yaw);
                continue;
            }
            if(strcmp("EYEC:", command) == 0) {
                if(RUNNING) continue;
				EYECANDY = 1;
                continue;
            }
            if(strcmp("STEC:", command) == 0) {
				EYECANDY = 0;
                continue;
            }
            /////////////////
            if(strcmp("CLTA:", command) == 0) {
                FILE *fdDump;
                fdDump = fopen("/data/samples/dump","w");
                if(fdDump != NULL) {
                    fprintf(fdDump,"GV:%+09.1f, COUNT:%+08.1f WAYUP:%+07.1f, ROLL:%+07.1f\n", calibrationData.gravityValue, (float)calibrationData.gravityDataCount, (float)calibrationData.wayup, roll-calibrationData.tareValue);
                    for(int i = 0; i<calibrationData.gravityDataCount; ++i){
                        fprintf(fdDump,"A:%+09.1f,R:%+09.1f,C:%+07.1f\n", calibrationData.gravityData[(i*3)], calibrationData.gravityData[(i*3)+1], calibrationData.gravityData[(i*3)+2]);
                    }
                    fclose(fdDump);
                }
                continue;
            }

            if(strcmp("STND:", command) == 0) {  // standalone operation
                calibrationData.gravityValue = 0;
                calibrationData.gravityCalibrationState = 4;
                calibrationData.torn = 3;
                continue;
            }

            if(strcmp("PLRD:", command) == 0) {  // play and record operation
                calibrationData.gravityValue = 0;
                calibrationData.gravityCalibrationState = 0;
                calibrationData.torn = 0;
                getSavedData();
                continue;
            }
            if(strcmp("BELS:", command) == 0) {  // set number of bells.  Not used here but recorded in file
                if(entries == 2  && strlen(details) > 0){
                    calibrationData.bellNumber = 0;
                    calibrationData.CPM = 0;
                    calibrationData.openHandstroke = 1;
                    char *ent = strtok(details,","); 
                    if(ent) calibrationData.bellNumber = atoi(ent);
                    ent = strtok(NULL,",");
                    calibrationData.CPM = atof(ent);
                    ent = strtok(NULL,",");
                    calibrationData.openHandstroke = atof(ent);
                    ent = strtok(NULL,",");
                    calibrationData.fakeTimefromBDC = atoi(ent); // if "Auto" is sent then that results in zero here
                    if(calibrationData.fakeTimefromBDC == 0){
                        calibrationData.faked = 1;
                    } else {
                        calibrationData.faked = 0;
                    }
                }
                continue;
            }

            if(strcmp("STRT:", command) == 0) {
                if((calibrationData.torn < 3) || (calibrationData.gravityCalibrationState != 4)){
                    fprintf(stderr, "Still calibrating...\n");
                    printf("ESTR:\n");
                    printf("STPD:\n");                    
                }
                if(entries == 2  && strlen(details) < 34){  // use chosen filename
                    sprintf(FILENAME, "/data/samples/%s", details);
                } else {                                    // otherwise make a filename up
                    struct tm *timenow;
                    time_t now = time(NULL);
                    timenow = gmtime(&now);
                    strftime(FILENAME, sizeof(FILENAME), "/data/samples/Unnamed_%d-%m-%y_%H%M", timenow);
                }
                fd_write_out = fopen(FILENAME,"w");
                if(fd_write_out == NULL) {
                    fprintf(stderr, "Could not open file for writing\n");
                    printf("ESTR:\n");
                    printf("STPD:\n");
                    continue;
                }
                OUT_COUNT = 0;
                calibrationData.lastStrikeCount = 0;
                calibrationData.lastBDC = 0;
                startRun();
                continue;
            }
            if(strcmp("STOP:", command) == 0) {
                if(fd_write_out != NULL){
                    if(local_count != 0) {fputs(local_outbuf, fd_write_out); local_count = 0;}
                    fflush(fd_write_out);
                    fclose(fd_write_out);
                    fd_write_out = NULL;
                }
                RUNNING = 0;
                printf("STPD:%d\n", OUT_COUNT);
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
                        if(ln > 0 && READ_OUTBUF_LINE[ln -1] == '\n') READ_OUTBUF_LINE[ln - 1] = '\0';   // get rid of \n
//                        READ_OUTBUF_LINE[strcspn(READ_OUTBUF_LINE, "\n")] = 0;
                        if (READ_OUTBUF_COUNT + strlen(READ_OUTBUF_LINE) + 7 > (sizeof(READ_OUTBUF) -2)) {
                            printf("%s\n", READ_OUTBUF);
                            READ_OUTBUF_COUNT = 0;
                        }
                        READ_OUTBUF_COUNT += sprintf(&READ_OUTBUF[READ_OUTBUF_COUNT],"DATA:%s",READ_OUTBUF_LINE);
                        counter += 1;
                        if((counter % 50) == 0) pushData(); // collect data from FIFO whilst doing this
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
    system("/usr/bin/touch /var/lib/systemd/clock");
    return 0;
}

void doCalibration(void){
    float angularVelocity;
    
    int FIFOcount = readFIFOcount();
    if (FIFOcount > 4000){
        printf("\nEOVF:\n");
        writeRegister(ICM20689_USER_CTRL,0x15); // reset FIFO
        usleep(1000);
        writeRegister(ICM20689_USER_CTRL,0x50);
        return;
    }

    if(FIFOcount < 48) return;
   
    while(FIFOcount >=48){
        FIFOcount -= 48;
        angularVelocity = pullAndTransform();
        switch (calibrationData.gravityCalibrationState) {
            case 0:
                if(calibrationData.stable == 1 && calibrationData.torn == 3){ // bell is stable and zeroed
                    calibrationData.gravityCalibrationState = 1;
                    for(int i = 0; i< BUFFERSIZE; ++i){ // initialise with dummy data for savgol alignment
                        rateBuffer[i] = 0;
                        angleBuffer[i] = 180.0+(roll-calibrationData.tareValue);
                    }
                    head = SAVGOLHALF;
                    tail = 0;
                    available = SAVGOLHALF;
                }
                break;
            case 1:
                if((roll-calibrationData.tareValue) < -140.0 || (roll-calibrationData.tareValue) > +140.0){ // bell now high enough to start accumulating gravity calibration data
                    calibrationData.gravityCalibrationState = 2;
                }
                rateBuffer[head] = angularVelocity;
                angleBuffer[head] = 180.0+(roll-calibrationData.tareValue);
                head = (head + 1) % BUFFERSIZE;
                tail = (tail + 1) % BUFFERSIZE; // just ditch old data for the time being
                break;
            case 2:
                if(calibrationData.stable == 1 && abs(roll-calibrationData.tareValue) > 160 && abs(roll-calibrationData.tareValue) < 178) {
                    calibrationData.gravityCalibrationState = 3; // bell now rung up proceed to calibration calculations
                }
                rateBuffer[head] = angularVelocity;
                angleBuffer[head] = 180.0+(roll-calibrationData.tareValue);
                head = (head + 1) % BUFFERSIZE;
                available += 1;
                if(available < PUSHBATCH + SAVGOLLENGTH) break;
                for(int pushcounter = available; pushcounter > SAVGOLLENGTH; --pushcounter){
                    // Record only a range of angles as the bell is being pulled up.  See calculateError() for justification  
                    // At this stage we don't know which is handstroke and which is backstroke so record both sides and both directions.  
                    if((angleBuffer[tail] < 275 && angleBuffer[tail] > 260) ||
                        (angleBuffer[tail] < 100 && angleBuffer[tail] > 85)){
                        if(calibrationData.gravityDataCount < 8000){
                            calibrationData.gravityData[calibrationData.gravityDataCount*3] = angleBuffer[tail];
                            calibrationData.gravityData[(calibrationData.gravityDataCount*3)+1] = rateBuffer[tail];
                            calibrationData.gravityData[(calibrationData.gravityDataCount*3)+2] = savGol(tail);
                            calibrationData.gravityDataCount += 1;
                        }
                    }
                    tail = (tail + 1) % BUFFERSIZE;
                    available -= 1;
                }
                break;
            case 3:
                if((roll-calibrationData.tareValue) < 0.0) {  // Work out which way the bell rose and adjust accordingly (we now know which is handstroke and backstroke)
                    for (unsigned int j = 0; j < calibrationData.gravityDataCount; ++j){ 
                        calibrationData.gravityData[j*3] = 360.0-calibrationData.gravityData[j*3]; // swap angle
                        calibrationData.gravityData[(j*3)+1] *= -1; // swap rate
                        calibrationData.gravityData[(j*3)+2] *= -1; // flip accn
                    }
                    ///////// temporary
                    calibrationData.wayup = -1;
                }
                calibrationData.gravityValue = 0;
                for(int guessLog = 14; guessLog >= 0; --guessLog){
                    if(calculateError(calibrationData.gravityValue + pow(2,guessLog)) < 0) calibrationData.gravityValue += pow(2,guessLog);
                }
                calibrationData.gravityCalibrationState = 4;
 ///////               if(calibrationData.gravityData) {free(calibrationData.gravityData); calibrationData.gravityData = NULL;}
                FILE *fdGravity;
                fdGravity = fopen("/tmp/BBgravity","w");
                if(fdGravity != NULL) {
                    fprintf(fdGravity,"%+08.1f\n", calibrationData.gravityValue);
                    fclose(fdGravity);
                }
        }
    }
}

void startRun(void){
    if(abs(roll-calibrationData.tareValue) < 160 || abs(roll-calibrationData.tareValue) > 178){
        printf("ESTD:%+07.1f\n",roll-calibrationData.tareValue); // bell not at stand
        return;
    }

    if(roll < 0) {  // this bit works out which way the bell rose and adjusts accordingly
        direction = -1;
    } else {
        direction = +1;
    }

    for(int i = 0; i< BUFFERSIZE; ++i){ // initialise with dummy data for savgol alignment
        rateBuffer[i] = 0;
        angleBuffer[i] = direction*(roll-calibrationData.tareValue)-180.0;
    }
    head = SAVGOLHALF;
    tail = 0;
    available = SAVGOLHALF;

    RUNNING = 1;
	EYECANDY = 0;
    return;
}

void pushData(void){
    char local_outbuf_line[150];
    char remote_outbuf_line[150];
    static char remote_outbuf[4000]; // not necessary to be static as this buffer is cleared before function exits
    int remote_count = 0;
    static float accn = 0;
    static float nudgeAngle= 0;  // delete this and code below if not needed
    static int nudgeCount = 0;
    static int switchCount = 0;
    float taccn;

    pullData();
    pullData(); // call a second time to catch any fresh samples that arrived during processing of the first round

    if(!RUNNING) return;

    // I calculate angular accelerations from the rates reported by gyro using a Savitsky Golay
    // filter set to differentiate.  Optimal filter length determined by experiment.
    // Need to have at least SAVGOLLENGTH samples ready in buffer but don't go through this unless there are also
    // an additional PUSHBATCH samples available (PUSHBATCH is the number of samples pushed to the browser in each batch).
    // Samples are pushed into the rateBuffer and angleBuffer circular buffers by pullData function.
    // available and tail variables are global and are used to keep track of circular buffer (as is the head
    // variable but that is updated by the pullData function)
    if(available < PUSHBATCH + SAVGOLLENGTH) return;

    for(int counter = available; counter > SAVGOLLENGTH; --counter){
        // This bit allowed for slight angle drift.  It does this knowing that BDC = 180 degrees.
        // and that is the point when the bell's angular acceleration is zero.  Drift is so low
        // that this is more trouble than it's worth.  Left in and reported but not used.
        // Also testing shows BDC angle is slightly off that recorded when the bell is at rest.  Frictional effects?

        // TODO: use this to signal when bell is at BDC(ish) so that strikes can be faked - someone might want to use this
        // with tied clappers.  It would work by faking a strike a defined period of time after BDC.

        float value = 0;
        unsigned int data = 0;

        taccn = savGol(tail);
        unsigned int lastTail = (tail == 0) ? BUFFERSIZE-1 : tail -1;
        if(taccn * accn <= 0.0 && angleBuffer[tail] > 170.0 && angleBuffer[tail] < 190.0  && nudgeCount == 0) {
            float nudgeFactor = accn /(accn - taccn);
            float angleDiffFrom180 = (angleBuffer[lastTail] + nudgeFactor*(angleBuffer[tail]-angleBuffer[lastTail])) - 180.0;
            nudgeAngle = angleDiffFrom180;
            nudgeCount = 10;
            calibrationData.lastBDC = OUT_COUNT;
            value = nudgeAngle; // report 180 degree point
            data = 8;
        } 
        if(nudgeCount != 0) nudgeCount -= 1; // don't do this twice in one half stroke - allows for a bit of noise
        accn = taccn;
        float raccn = accn - calibrationData.gravityValue*sin(angleBuffer[tail]*DEGREES_TO_RADIANS_MULTIPLIER);
        
        unsigned int strike = dingDong(tail); // check to see if there has been a bell strike 1=ding(HS), 2=dong(BS) 0=nowt

        if(rateBuffer[tail] * rateBuffer[lastTail] <= 0.0 && switchCount == 0 && OUT_COUNT > 10) { // detect when bell changes direction
            data = 7;
            value = OUT_COUNT;
            switchCount = 5;
        } 
        if (switchCount != 0 && abs(rateBuffer[tail]) > 100.0) switchCount -= 1;

        if(strike > 0 && strike <= 5){
            data = strike;
            value = (float)(OUT_COUNT - calibrationData.lastStrikeCount)/ODR; // number of seconds since last strike.
            calibrationData.lastStrikeCount = OUT_COUNT;
        }

        if(OUT_COUNT == 0){ // first line of output will show the gravity calibration value
            value = calibrationData.gravityValue;
            data = 9;
        }
        
        if(OUT_COUNT == 1){ // second line of output shows the requested number of bells
            value = calibrationData.bellNumber;
            data = 9;
        }

        if(OUT_COUNT == 2){ // third line of output will show the requested CPM
            value = calibrationData.CPM;
            data = 9;
        }
        
        if(OUT_COUNT == 3){ // fourth line of output shows the requested open handstoke adjustment
            value = calibrationData.openHandstroke;
            data = 9;
        }

        if(OUT_COUNT == 4){ // fifth line of output shows the requested faked chime status (0=not faked)
            value = calibrationData.fakeTimefromBDC;
            data = 9;
        }
      
        sprintf(remote_outbuf_line, "LIVE:A:%+6.1f,R:%+6.1f,C:%+7.1f,D:%d,V:%+10.2f\n", angleBuffer[tail], rateBuffer[tail], raccn, data, value);
        if (remote_count + strlen(remote_outbuf_line) > (sizeof remote_outbuf -2)) {
            printf("%s",remote_outbuf);
            remote_count = 0;
        }
        
        remote_count += sprintf(&remote_outbuf[remote_count],remote_outbuf_line);     
        sprintf(local_outbuf_line,"A:%+6.1f,R:%+6.1f,C:%+7.1f,D:%d,V:%+10.2f\n", angleBuffer[tail], rateBuffer[tail], raccn, data, value);
        if (local_count + strlen(local_outbuf_line) > (sizeof local_outbuf -2)) {
            fputs(local_outbuf, fd_write_out);
            fflush(fd_write_out);
            local_count = 0;
        }
        local_count += sprintf(&local_outbuf[local_count],local_outbuf_line);
        tail = (tail + 1) % BUFFERSIZE;
        available -= 1;
        OUT_COUNT += 1;

//        fprintf(fd_write_out,"A:%+07.1f,R:%+07.1f,C:%+07.1f,P:%+07.1f,Y:%+07.1f,AX:%+06.3f,AY:%+06.3f,AZ:%+06.3f,GX:%+07.1f,GY:%+07.1f,GZ:%+07.1f,X:%+06.3f,Y:%+06.3f,Z:%+06.3f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang, pitch, yaw, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2],a[0],a[1],a[2]);
    }
    if(remote_count != 0) {printf("%s",remote_outbuf); remote_count = 0;}
}

void pullData(void){
    float angularVelocity;
    float dummy[6];
    static unsigned int angleCorrection = 0;

    int count = readFIFOcount();

    if (count > 4000){ // overflow (or nearly so)
        printf("\nEOVF:\n"); //shouldn't happen
        //clear data paths and reset FIFO (keep i2c disabled)
        writeRegister(ICM20689_USER_CTRL,0x15);
        usleep(1000);
 
        //restart FIFO - keep i2c disabled
        writeRegister(ICM20689_USER_CTRL,0x50);
        return;
    }

    if(count < 48) return;

    while(count >=48){
        count -= 48;
        angularVelocity = pullAndTransform();

        if(!RUNNING) continue;

        // need to correct reported angles to match the system we are 
        // using 0 degrees = bell at balance at handstroke, 
        // 360 degrees = bell at balance at backstroke. 180 degrees = BDC. 

        unsigned int lastHead = (head == 0) ? BUFFERSIZE-1 : head -1;
        float angle = direction*(roll-calibrationData.tareValue)-180.0;
        angle += angleCorrection;
        if((angleBuffer[lastHead] - angle) > 270){
            if(angleCorrection < 720 ) { // should always be true
                angleCorrection += 360; 
                angle += 360;
            }
        } else if ((angleBuffer[lastHead] - angle) < -270){
            if(angleCorrection > 0){ // should always be true
            angleCorrection -= 360; 
            angle -= 360;
            }
        }

        rateBuffer[head] = angularVelocity * direction;
        angleBuffer[head] = angle;
        head = (head + 1) % BUFFERSIZE;
        available += 1;
        if(available >= BUFFERSIZE) {
            printf("EOVF:\n");  // circular buffer full - should never happen
            available -= 1;
        }
    }
}

float pullAndTransform(){
    static float peaks[3] = {0};
    static float averages[3] = {0};
    static unsigned int count = 0;
    float fifoData[6];

    // Although the browser get samples at 125Hz, the ICM20689s are set up 
    // to push data samples out at 500Hz.  The angles and quaternion 
    // are computed at the higher rate as some filters operate better at
    // higher sample rates (although I have not tested whether the Mahony filter
    // (see calculate function) is actually any better).  Four sets
    // of samples are processed by this function at a time.  The browser will only see data
    // derived from last of these samples.
    for(int i = 0; i < 4; ++i){ 
        readFIFO(fifoData);
        
        // get average gyro rates - used for dynamic bias compensation calculation
        for(int i = 0; i < 3; ++i){
            averages[i] += fifoData[3+i];
        }

        fifoData[0] -= calibrationData.accBiasX;
        fifoData[1] -= calibrationData.accBiasY;
        fifoData[2] -= calibrationData.accBiasZ;

        fifoData[3] -= calibrationData.gyroBiasX;
        fifoData[4] -= calibrationData.gyroBiasY;
        fifoData[5] -= calibrationData.gyroBiasZ;

        // used for bias compensation calculation, checks to see if bell is at rest
        // by recording a peak gyro movement over the measurement period (4000 samples = 8 secs)
        for(int i = 0; i < 3; ++i){
            if(abs(fifoData[3+i]) > peaks[i]) peaks[i] = abs(fifoData[3+i]);
            if(peaks[i] > 2.1) calibrationData.stable = 0;
        }

        // transform data using transform matrix (from imu_tk and collected from Arduino via setup function)
        fifoData[0] = fifoData[0]*calibrationData.accTransformMatrix[0]
                    + fifoData[1]*calibrationData.accTransformMatrix[1]
                    + fifoData[2]*calibrationData.accTransformMatrix[2];

        fifoData[1] = fifoData[0]*calibrationData.accTransformMatrix[3]
                    + fifoData[1]*calibrationData.accTransformMatrix[4]
                    + fifoData[2]*calibrationData.accTransformMatrix[5];

        fifoData[2] = fifoData[0]*calibrationData.accTransformMatrix[6]
                    + fifoData[1]*calibrationData.accTransformMatrix[7]
                    + fifoData[2]*calibrationData.accTransformMatrix[8];

        fifoData[3] = fifoData[3]*calibrationData.gyroTransformMatrix[0]
                    + fifoData[4]*calibrationData.gyroTransformMatrix[1]
                    + fifoData[5]*calibrationData.gyroTransformMatrix[2];

        fifoData[4] = fifoData[3]*calibrationData.gyroTransformMatrix[3]
                    + fifoData[4]*calibrationData.gyroTransformMatrix[4]
                    + fifoData[5]*calibrationData.gyroTransformMatrix[5];

        fifoData[5] = fifoData[3]*calibrationData.gyroTransformMatrix[6]
                    + fifoData[4]*calibrationData.gyroTransformMatrix[7]
                    + fifoData[5]*calibrationData.gyroTransformMatrix[8];

        count +=1;
        if((count % 2500) == 0 && calibrationData.torn < 3) printf("EWAI:\n");  // sends signal to browser to indicate that the sensor is still calibrating zero
        if((count % 2500) == 0 && calibrationData.gravityCalibrationState == 1) printf("EPUL:\n");  // sends signal to browser to indicate that bell needs to be pulled up
        if((count % 2500) == 0 && calibrationData.gravityCalibrationState == 2) printf("ECAL:\n");  // sends signal to browser to indicate that the bell is high enough to start gravity calibration

        if((count % 4000) == 0){ // check bias every 8 seconds
            if(peaks[0] < 2.1 && peaks[1] < 2.1 && peaks[2] < 2.1){  // no peaks in gyro rate so can take the last 8 seconds as stationary
                calibrationData.gyroBiasX = averages[0]/4000.0;  // adjust biasses
                calibrationData.gyroBiasY = averages[1]/4000.0;
                calibrationData.gyroBiasZ = averages[2]/4000.0;
                calibrationData.stable = 1; // flag that bell is stable
                // Initial (bell down) gyro calibration is done in four stages (each lasting 8 seconds). 
                // First, measure biasses. 
                // Second, apply measured biasses.
                // Third, a further period to allow the readings to settle
                // Fourth, take "zero" tare value and save.  
                // The measured angles take a few seconds to settle down once the biasses are applied, hence the third stage.                  
                
                if (calibrationData.torn < 3 && abs(roll) < 20.0) { // if bell is down (or within 20 degrees of vertical) take that as the tare value
                    calibrationData.torn +=1;
                    if (calibrationData.torn == 3) {
                        calibrationData.tareValue = roll;
                        FILE *fdTare;
                        fdTare = fopen("/tmp/BBtare","w");
                        if(fdTare != NULL) {
                            fprintf(fdTare,"%+07.4f\n", calibrationData.tareValue);
                            fclose(fdTare);
                        }
                    }
                }
            } else {
                calibrationData.stable = 0;
            }
            for(int i = 0; i < 3; ++i) { averages[i] = 0.0; peaks[i] = 0.0; }
        }
        
        // This function calculates angles and a quaternion from the gyro and accelerometer measurements. The q0, q1,q2,q3, roll, pitch and yaw globals
        // are updated by this function (hence no return value)
        calculate(fifoData[3],fifoData[4],fifoData[5], fifoData[0],fifoData[1],fifoData[2],calibrationData.samplePeriod);
    }
    return(fifoData[3]);  // Just "x" gyro rate needed by the calling function (pullData).
}

float savGol(unsigned int currentPosition){
    unsigned int windowStartPosition = (currentPosition < SAVGOLHALF) ? (currentPosition + BUFFERSIZE) - SAVGOLHALF: currentPosition - SAVGOLHALF;
    float savGolResult=0;
    for(int i = 0; i < SAVGOLLENGTH; ++i){
        savGolResult += savGolCoefficients[i] * rateBuffer[(windowStartPosition +i) % BUFFERSIZE];
    }
    savGolResult *= ODR;
    return(savGolResult);
}

// This function calculates the difference (error) between a sine wave of amplitude "guess" and the
// acceleration profile of the bell.
float calculateError(float guess){
    float count = 0.0;
    float error = 0.0;
    for(unsigned int i = 0; i < calibrationData.gravityDataCount; ++i){
        // Only calculate gravity effect going up at backstroke because of fewer other effects such as vibration from strike and pulley/garter hole interaction.
        // Because this is intended to be used as the bell is being pulled up - best to do measurement when bell is rising - the ringer could be pulling quite
        // hard as the bell is falling (in order to get the bell up) and that skews the measurement.
        // Measure between angles 260 and 275 as we need to avoid the strike (could be as early as 280 degrees on light bells) 
        if(calibrationData.gravityData[(i*3)] < 275 && calibrationData.gravityData[(i*3)] > 260 && calibrationData.gravityData[(i*3)+1] > 0){ 
            count += 1;
            error += -guess*sin(calibrationData.gravityData[(i*3)]*DEGREES_TO_RADIANS_MULTIPLIER) + calibrationData.gravityData[(i*3)+2];
        }
        if(count > 4000) break; // should be enough
    }
    if(count != 0){
        error /= count;
    } else {
        error = 0;
    }
    return error;
}

// This function works out when the bell has been struck by the clapper
// The data is passed to the browser which works out inter-strike timings.
// Quite a a bit of experimentation here.  The function works out a base level
// volatility (rate of change of acceleration) of the bell system and when 
// this jumps suddenly (by 1.7x), a strike is recorded.
// Tested on heavy and light bells and produces sane and consistent results +- 20ms or so
// averging these could produce a half-decent odd-struckness meter.
// Need to test on plain bearing bells

unsigned int dingDong(unsigned int currentPosition){
    static int lastStrike = 2; // 2 = last chime was at backstroke, 1= last chime at handstroke 
    static float rollingValues[5] = {8,8,8,8,8};
    static unsigned int rollingPosition = 0;
    static float baseSD = 0.0; 
/*    
    if(OUT_COUNT == 0){
        calibrationData.dingTimeFromBDC = 0.0;
        calibrationData.dongTimeFromBDC = 0.0;
        calibrationData.dingAngleFromBDC = 0.0;
        calibrationData.dongAngleFromBDC = 0.0;
        calibrationData.dingNumber = 0;
        calibrationData.dongNumber = 0;
    }
*/ 
    if(calibrationData.faked){ // doing faked chimes (eg clapper tied)
        if(((currentPosition - calibrationData.lastBDC)/ODR) > (calibrationData.fakeTimefromBDC/1000.0)){
            if((lastStrike == 1) && (angleBuffer[currentPosition] < 160) && (rateBuffer[currentPosition] < -20)){
                lastStrike = 2;
                return(2);
            }
            if((lastStrike == 2) && (angleBuffer[currentPosition] > 200) && (rateBuffer[currentPosition] > 20)){
                lastStrike = 1;
                return(1);
            }
        }
        return(0);
    }
 
// savgol_coeffs(5,2,deriv=2,use="dot")
    static float dingDongCoefficients[] = { 0.28571429, -0.14285714, -0.28571429, -0.14285714,  0.28571429 };

    unsigned int windowStartPosition = (currentPosition < 2) ? (currentPosition + BUFFERSIZE) - 2 : currentPosition - 2;
    
    float savGolResult = 0;
    
    for(int i = 0; i < 5; ++i){
        savGolResult += dingDongCoefficients[i] * rateBuffer[(windowStartPosition +i) % BUFFERSIZE];
    }

    savGolResult *= 50;
    
    rollingValues[rollingPosition] = savGolResult;
    rollingPosition = (rollingPosition + 1) % 5;
    
    float SD = sqrt(rollingValues[0]*rollingValues[0] + rollingValues[1]*rollingValues[1] + rollingValues[2]*rollingValues[2] + rollingValues[3]*rollingValues[3] + rollingValues[4]*rollingValues[4]);
    
    // calculate base level of volatility and apply a bit of smoothing
    if(angleBuffer[currentPosition] > 100 && angleBuffer[currentPosition] < 260) baseSD = 0.8*baseSD + 0.2*SD;
    
    if((lastStrike == 1) && (SD > 1.7*baseSD) && (angleBuffer[currentPosition] < 70) && (rateBuffer[currentPosition] < 0)){
        lastStrike = 2;
//        calibrationData.dongNumber += 1;
//        calibrationData.dongTimeFromBDC += (OUT_COUNT - calibrationData.lastBDC)*125/1000.0;
//        calibrationData.dongTimeFromBDC /= calibrationData.dongNumber;
//        calibrationData.dongAngleFromBDC += abs(roll);
//        calibrationData.dongAngleFromBDC /= calibrationData.dongNumber;
        calibrationData.lastStrikeCount = OUT_COUNT;
//        printf("DONG\n");
        return(2);
    } else if((lastStrike == 2) && (SD > 1.7*baseSD) && (angleBuffer[currentPosition] > 290) && (rateBuffer[currentPosition] > 0)) {
        lastStrike = 1;
//        calibrationData.dingNumber += 1;
//        calibrationData.dingTimeFromBDC += (OUT_COUNT - calibrationData.lastBDC)*125/1000.0;
//        calibrationData.dingTimeFromBDC /= calibrationData.dingNumber;
//        calibrationData.dingAngleFromBDC += abs(roll);
//        calibrationData.dingAngleFromBDC /= calibrationData.dingNumber;
        calibrationData.lastStrikeCount = OUT_COUNT;
//        printf("DING\n");
        return(1);
    } else if ((lastStrike == 1) && (angleBuffer[currentPosition] < 40) && (rateBuffer[currentPosition] < 0)){
        lastStrike = 2;
        return(4); // force a strike because not detected
    } else if ((lastStrike == 2) && (angleBuffer[currentPosition] > 320) && (rateBuffer[currentPosition] > 0)){
        lastStrike = 1;
        return(3); // force a strike because not detected
    } 
    return(0);
}

// Madgwick's implementation of the Mahony filter.
// http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Neat as it does not have an internal state that needs to be maintained
// like the Kalman filters (other than the quaternion of course).
// Stripped out all the Ki stuff as we don't need to worry about gyro biases :)
// Units here are degrees per second for the gyro.  It does not matter
// what units the accelerometers are in as the measurements are normalised.
// h is sample interval (time in seconds since the last sample)
// The q0, q1, q2, q3, roll, pitch and yaw outputs are globals 
void calculate(float gx, float gy, float gz, float ax, float ay, float az, float h) {
    const float twoKp = 0.7;  // Magic number determined by experimentation
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx *= DEGREES_TO_RADIANS_MULTIPLIER;
    gy *= DEGREES_TO_RADIANS_MULTIPLIER;
    gz *= DEGREES_TO_RADIANS_MULTIPLIER;

    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
        recipNorm = sqrt(ax * ax + ay * ay + az * az);
        ax /= recipNorm;
        ay /= recipNorm;
        az /= recipNorm;
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5 + q3 * q3;
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
	}
    gx *= (0.5 * h);
    gy *= (0.5 * h);
    gz *= (0.5 * h);
    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= recipNorm;
    q1 /= recipNorm;
    q2 /= recipNorm;
    q3 /= recipNorm;

    roll = RADIANS_TO_DEGREES_MULTIPLIER * atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
    pitch = RADIANS_TO_DEGREES_MULTIPLIER * asin(-2.0 * (q1*q3 - q0*q2));
    yaw = RADIANS_TO_DEGREES_MULTIPLIER * atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
}

void getSavedData(){
    FILE *fdTare;
    char linein[50];
    fdTare = fopen("/tmp/BBtare","r");
    if(fdTare != NULL) {
        if (fgets(linein, sizeof(linein), fdTare) != NULL) {
            calibrationData.tareValue = atof(linein);
            calibrationData.torn = 3;
        }
        fclose(fdTare);
    }

    FILE *fdGravity;
    fdGravity = fopen("/tmp/BBgravity","r");
    if(fdGravity != NULL) {
        if (fgets(linein, sizeof(linein), fdGravity) != NULL) {
            calibrationData.gravityValue = atof(linein);
            calibrationData.gravityCalibrationState = 4;
        }
        fclose(fdGravity);
    } else {
        if(!calibrationData.gravityData) calibrationData.gravityData = (float *)malloc(sizeof(float) * 8000 * 3);
    }

}

void setup(void){	
    if (!bcm2835_init()){
        printf("Unable to inititalise bcm2835\n");
        exit(1);
    }
	
	//setup I2C
    if (!bcm2835_i2c_begin()){
        printf("Unable to inititalise i2c\n");
        exit(1);
    }
	
    bcm2835_i2c_set_baudrate(100000);
    bcm2835_i2c_setSlaveAddress(0x10);
	
	//  setup SPI
    if (!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. \n");
      exit(1);
    }
	
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    bcm2835_spi_set_speed_hz(8000000);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);

   //reset device
    writeRegisterBits(ICM20689_PWR_MGMT_1,0xFF,0x80);
    usleep(100000);
 
   //disable i2c
    writeRegisterBits(ICM20689_USER_CTRL,0xFF,0x10);

    if(readRegister(ICM20689_WHO_AM_I) != 0x98){
      printf("Unable to read from IMU device. \n");
      exit(1);        
    }
    // bring out of sleep, set clksel (to PLL) and disable temperature sensor
    writeRegister(ICM20689_PWR_MGMT_1,0x09);
    usleep(30000); 

    //configure DLPF
    writeRegister(ICM20689_CONFIG,0x01);

    //full scale accelerometer range to 4g
    writeRegister(ICM20689_ACCEL_CONFIG,0x08);

    //full scale gyro range to 500deg/s
    writeRegister(ICM20689_GYRO_CONFIG,0x08);

    //set sample rate divider we want 500Hz assuming base clock is 1KHz
    writeRegister(ICM20689_SMPLRT_DIV,0x01);

    //set FIFO size to 4K (this setting does not appear in the V1 datasheet!)
    writeRegister(ICM20689_ACCEL_CONFIG_2,0xC0);
    
    //select registers for FIFO
    writeRegister(ICM20689_FIFO_EN,0x78);

    //clear data paths and reset FIFO (keep i2c disabled)
    writeRegister(ICM20689_USER_CTRL,0x15);

	// load up calibration data from Arduino.  Calibration data uses
    // the method of D. Tedaldi, A. Pretto and E. Menegatti, 
    // "A Robust and Easy to Implement Method for IMU Calibration 
    // without External Equipments". In: Proceedings of the IEEE International 
    // Conference on Robotics and Automation (ICRA 2014), May 31 - June 7, 2014
    // Hong Kong, China, Page(s): 3042 - 3049
    // See https://bitbucket.org/alberto_pretto/imu_tk
    // notes on installing imu_tk and its dependencies and code to implement
    // the method are in this repository's utilities directory
/*
    (Arduino "registers" 10=samplePeriod, 11=accBiasX,
    12=accBiasY, 13=accBiasZ, 14=accScale00, 15=accScale01, 16=accScale02,
    17=accScale10, 18=accScale11, 19=accScale12, 20=accScale20, 21=accScale21,
    22=accScale22, 23=gyroBiasX, 24=gyroBiasY, 25=gyroBiasZ, 26=gyroScale00,
    27=gyroScale01, 28=gyroScale02, 29=gyroScale10, 30=gyroScale11, 31=gyroScale12, 
    32=gyroScale20, 33=gyroScale21, 34=gyroScale22, 
    200=accBiasXYZ(all in one go 12 bytes, three floats), 
    201=accScale0(all in one go 12 bytes, three floats),
    202=accScale1 (all in one go 12 bytes, three floats),
    203=accScale2 (all in one go 12 bytes, three floats),
    204=gyroBiasXYZ(all in one go 12 bytes, three floats), 
    205=gyroScale0(all in one go 12 bytes, three floats),
    206=gyroScale1 (all in one go 12 bytes, three floats),
    207=gyroScale2 (all in one go 12 bytes, three floats).
*/

    I2C_BUFFER[0]=10;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 10 (samplePeriod)
    bcm2835_i2c_read(I2C_BUFFER,4); usleep(100);
    float result;
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.samplePeriod = result;
	
    I2C_BUFFER[0]=200;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 200 (accBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.accBiasX = result/g0;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.accBiasY = result/g0;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.accBiasZ = result/g0;
	
    I2C_BUFFER[0]=201;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 201 (accTransform first row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.accTransformMatrix[0] = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.accTransformMatrix[1] = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.accTransformMatrix[2] = result;

    I2C_BUFFER[0]=202;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 201 (accTransform second row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.accTransformMatrix[3] = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.accTransformMatrix[4] = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.accTransformMatrix[5] = result;

    I2C_BUFFER[0]=203;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 203 (accTransform bottom row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.accTransformMatrix[6] = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.accTransformMatrix[7] = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.accTransformMatrix[8] = result;

    I2C_BUFFER[0]=204;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 204 (gyroBiasXYZ)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.gyroBiasX = result*RADIANS_TO_DEGREES_MULTIPLIER;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.gyroBiasY = result*RADIANS_TO_DEGREES_MULTIPLIER;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.gyroBiasZ = result*RADIANS_TO_DEGREES_MULTIPLIER;

    I2C_BUFFER[0]=205;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 205 (gyroTransform first row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[0] = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[1] = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[2] = result;

    I2C_BUFFER[0]=206;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 206 (gyroTransform second row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[3] = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[4] = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[5] = result;

    I2C_BUFFER[0]=207;
    bcm2835_i2c_write(I2C_BUFFER,1); usleep(100); // set register 207 (gyroTransform bottom row)
    bcm2835_i2c_read(I2C_BUFFER,12); usleep(100);
    result = extractFloat(0);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[6] = result;
    result = extractFloat(4);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[7] = result;
    result = extractFloat(8);
    if(!isnan(result)) calibrationData.gyroTransformMatrix[8] = result;

    //start FIFO - keep i2c disabled
    writeRegister(ICM20689_USER_CTRL,0x50);
}

float extractFloat(uint8_t index){
    union {
        float    _float;
        uint8_t  _bytes[sizeof(float)];
    } floatConv;
    floatConv._bytes[0] = I2C_BUFFER[index];
    floatConv._bytes[1] = I2C_BUFFER[index+1];
    floatConv._bytes[2] = I2C_BUFFER[index+2];
    floatConv._bytes[3] = I2C_BUFFER[index+3];
    return floatConv._float;
}

uint16_t readFIFOcount(){
    return ((uint16_t)readRegister(ICM20689_FIFO_COUNTH) << 8) + readRegister(ICM20689_FIFO_COUNTL); 
}

void readFIFO(float* values){
    float temp0;
    SPI_BUFFER[0] = ICM20689_FIFO_R_W | 0x80;
    for(uint8_t i = 1; i<13; ++i){
        SPI_BUFFER[i] = 0;  
    }
    bcm2835_spi_transfern(SPI_BUFFER,13);
    values[0]=((uint16_t)SPI_BUFFER[1] << 8) + SPI_BUFFER[2]; 
    values[1]=((uint16_t)SPI_BUFFER[3] << 8) + SPI_BUFFER[4];
    values[2]=((uint16_t)SPI_BUFFER[5] << 8) + SPI_BUFFER[6];
    values[3]=((uint16_t)SPI_BUFFER[7] << 8) + SPI_BUFFER[8]; 
    values[4]=((uint16_t)SPI_BUFFER[9] << 8) + SPI_BUFFER[10];
    values[5]=((uint16_t)SPI_BUFFER[11] << 8) + SPI_BUFFER[12];

    // remapping of axes X->Z Y->X  X->Y
    temp0=values[0];
    values[0]=values[1];
    values[1]=values[2];
    values[2]=temp0;

    temp0=values[3];
    values[3]=values[4];
    values[4]=values[5];
    values[5]=temp0;

    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] *= (4.0/32768.0); // 4G full scale
    }
  
    for(int i=3; i<6; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] *= (500.0/32768.0); // 500deg/sec full scale
    }
}

void writeRegister(uint8_t reg, uint8_t value){
    SPI_BUFFER[0] = reg;
    SPI_BUFFER[1] = value;
    bcm2835_spi_transfern(SPI_BUFFER,2);
}

void writeRegisterBits(uint8_t reg, uint8_t mask, uint8_t value){
    uint8_t readValue;
    readValue = readRegister(reg);
    SPI_BUFFER[0] = reg; // & 0x7F;  // clear high bit for a write
    SPI_BUFFER[1] = (readValue & mask) | value;
    bcm2835_spi_transfern(SPI_BUFFER,2);
}

uint8_t readRegister(uint8_t reg){
    SPI_BUFFER[0] = reg | 0x80;  //set high bit for a read
    SPI_BUFFER[1] = 0;
    bcm2835_spi_transfern(SPI_BUFFER,2);
    return SPI_BUFFER[1];
}
