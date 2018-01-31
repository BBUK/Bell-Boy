//gcc bb_dcmimu.c -o bb_dcmimu -lm

/*
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
#include <linux/i2c-dev.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <dirent.h>
#include "bb_dcmimu.h"

#ifndef NULL
#define NULL 0
#endif

#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif

#define g0 9.8189
#define g0_2 (g0*g0)
#define q_dcm2 (0.1*0.1)
#define q_gyro_bias2 (0.0001*0.0001)
#define r_acc2 (0.5*0.5)
#define r_a2 (10*10)
#define q_dcm2_init (1*1)
#define q_gyro_bias2_init (0.1*0.1)

#define DEGREES_TO_RADIANS_MULTIPLIER 0.017453 
#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 
#define I2CDEV "/dev/i2c-1"

int NXP_fd_gyro = -1;
int NXP_fd_accel = -1;
FILE *fd_write_out;
//int MPU6050_fd = -1;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float a[3];
float sample_period = 0.0;
float GYRO_BIAS[3] = { 0.0, 0.0, 0.0 };
int DEBUG = 0;


float NXP_gyro_scale_factor = 0;
float NXP_accel_scale_factor = 0;

float ROTATIONS[3] = { -1.0, -1.0, -1.0 }; //set rotation values for mounting IMU device x y z

int SWAPXY = 1; // swap XY for different mounting.  Swap is applied before rotation.

int ODR = 0;
int FS_GYRO = 0;
int FS_ACCEL = 0;

int RUNNING = 0;
int OUT_COUNT = 0;

char READ_OUTBUF[1500];
char READ_OUTBUF_LINE[150];
int READ_OUTBUF_COUNT;

char FILENAME[50];

int LOOPSLEEP = 100000; // sleep for 0.1 of a second unless actively sampling then sleep 4000000/ODR

volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}
/*
# There are eight possible command sent by the user's browser:
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
# (x)  ESTR: internal error related to data / filenames (shouldn't happen)!
# (xi) DATA:[string]  chunk of data from a previously stored file.  In same format as LIVE:
# (xii)EOVF: (sent by dcmimu) fifo overflow flagged.
# (xiii)FILE:[filename] in response to FILE: a list of the previous recordings in /data/samples
# (xiv) STRT: in response to STRT: indicates a sucessful start.
# (xv)  LFIN:[number] indicates the end of a file download and the number of samples sent
*/

/*
* Command line arguments bb_dcmimu {1} {2} {3}
* {1} = Operational data rate (ODR). One of 800, 400, 200, 100 and 50 samples per second.
* {2} = gyro full scale range. One of 500 and 1000 (in degrees/second).
* {3} = accelerometer full scale range.  One of 2 and 4 (g).
* Example: bb_dcmimu 200 500 2
* Commands (see above) are received on stdin and output is on stdout.
* Will work standalone but intended to be interfaced with websocketd https://github.com/joewalnes/websocketd
*/
int main(int argc, char const *argv[]){
    char linein[50];
    char command[6];
    char details[42];
    char oscommand[90];
    float start_angle = 0.0;
    int entries;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
    
    setbuf (stdout, NULL);
    setbuf (stdin, NULL);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    if(argc != 4){
        printf("Incorrect number of arguments for BellBoy app.  Expect ODR, FS gyro and FS accelerometer.\n");
        return -1;
    }
    ODR = atoi(argv[1]);
    if (ODR != 50 && ODR != 100 && ODR != 200 && ODR != 400 && ODR != 800) {
        printf("Incorrect ODR.  Expects 50, 100, 200, 400 or 800 as first argument.\n");
        return -1;
    }
    sample_period = 1.0/ODR; // this is an initial estimate.  SAMP: command measures this directly and updates sample_period
	
    FS_GYRO = atoi(argv[2]);
    if (FS_GYRO != 500 && FS_GYRO != 1000) {
        printf("Incorrect FS gyro.  Expects 500 or 1000 as second argument.\n");
        return -1;
    }
    
    FS_ACCEL = atoi(argv[3]);
    if (FS_ACCEL != 2 && FS_ACCEL != 4) {
        printf("Incorrect FS accel.  Expects 2 or 1000 as second argument.\n");
        return -1;
    }
    
    if(NXP_test() != 0){
        printf("EIMU:\n");
        return -1;
    }
    
    while(!sig_exit){
        usleep(LOOPSLEEP);
        if(RUNNING) NXP_pull_data();
        while(fgets(linein, sizeof(linein), stdin ) != NULL) {
            entries = sscanf(linein, "%5s%[^\n]", command, details);
            if(entries < 1) {
                fprintf( stderr, "\nError reading: %s \n", linein );
                continue;
            }
//            if(entries == 2) printf("Command: %s Details: %s Count: %d \n", command, details, count);
//            if(entries == 1) printf("Command: %s Count: %d \n", command, count);
  
            if(strcmp("DATE:", command) == 0 && entries == 2  && strlen(details) < 12){
                sprintf(oscommand,"/usr/bin/date --set=\"$(date --date='@%s')\" && /usr/bin/touch /var/lib/systemd/clock", details);
                system(oscommand);
                continue;
            }
            if(strcmp("SAMP:", command) == 0) {
                if(NXP_fifo_timer() == 0) {
                    printf("SAMP:%f\n",sample_period);
                } else {
                    printf("EIMU:\n");
                }
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
                    strftime(FILENAME, sizeof(FILENAME), "/data/samples/Unnamed_%d-%m-%y_%H:%M", timenow);
                }
                ROTATIONS[0] = -1;   // reset rotations
                ROTATIONS[1] = -1;
                ROTATIONS[2] = -1;
                start_angle = NXP_get_orientation(); // also sets gyro biasses
                if(start_angle == -999) {
                    printf("ESTR:\n");
                    fprintf(stderr, "Can't calculate start angle\n");
                }
                if(start_angle < 2.0 && start_angle > -2.0) {
                    printf("ESTD:\n"); // bell out of range for stand
                    continue;
                }
                if(start_angle >= 2.0){             // this check enables the user not to think
                    ROTATIONS[0] = -ROTATIONS[0];   // about which way to mount the sensor
                    ROTATIONS[1] = -ROTATIONS[1];   // it just flips things around as if the sensor
                    start_angle = -start_angle;     // was mounted the other way round
		    GYRO_BIAS[0] = -GYRO_BIAS[0];
                    GYRO_BIAS[1] = -GYRO_BIAS[1];
                }
                if(start_angle < -20){
                    printf("ESTD:\n"); // bell out of range for stand
                    continue;
                }
                if(abs(GYRO_BIAS[0]) > 5.0){
                    printf("EMOV:\n"); // bell is moving
                    continue;
                }
                fd_write_out = fopen(FILENAME,"w");
                if(fd_write_out == NULL) {
                    fprintf(stderr, "Could not open file for writing\n");
                    printf("ESTR:\n");
                    continue;
                } else {
                    if(NXP_start_fifos(ODR,FS_GYRO,FS_ACCEL) == 0){
                        LOOPSLEEP = 4000000/ODR;
                        RUNNING = 1;
			OUT_COUNT = 0;
                        printf("STRT:\n");
                    } else {
                        printf("EFIF:\n");
                        printf("STPD:\n");
                    }
                }
                continue;
            }
            if(strcmp("STOP:", command) == 0) {
                if(fd_write_out != NULL){
                    fclose(fd_write_out);
                    fd_write_out = NULL;
                }
                NXP_stop_fifos();
                RUNNING = 0;
                LOOPSLEEP = 100000;
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
    if(fd_write_out != NULL) fclose(fd_write_out);
    NXP_stop_fifos();
    system("/usr/bin/touch /var/lib/systemd/clock");
    return 0;
}

int NXP_test(void){
    if (NXP_fd_gyro != -1) return -10;
	if ((NXP_fd_gyro = open(I2CDEV, O_RDWR)) < 0) {
        NXP_fd_gyro = -1;
        return -1;
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
	    close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return -2;
	}
    if (i2c_smbus_read_byte_data(NXP_fd_gyro, NXP_GYRO_REGISTER_WHO_AM_I) != NXP_GYRO_ID) {
        close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return -3;
    }
    close(NXP_fd_gyro);
    NXP_fd_gyro = -1;

    if (NXP_fd_accel != -1) return -11;
	if ((NXP_fd_accel = open(I2CDEV, O_RDWR)) < 0) {
        NXP_fd_accel = -1;
        return -4;
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
	    close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return -5;
	}
    if (i2c_smbus_read_byte_data(NXP_fd_accel, NXP_ACCEL_REGISTER_WHO_AM_I) != NXP_ACCEL_ID){
        close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return -6;
    }
    close(NXP_fd_accel);
    NXP_fd_accel = -1;

    return 0;
}

int NXP_fifo_timer(void){
    struct timeval start, stop;
    int cco, loops;
    float dummy[3];
    float gyro_result = 0.0; //, accel_result = 0;
    if (NXP_start_fifos(ODR,500,2) < 0){
        NXP_stop_fifos();
        return -1;
    }
    loops = (int)(ODR/25);
    for(int i = 0; i < loops; ++i){
        while (NXP_read_gyro_fifo_count() != 0) NXP_read_gyro_data(dummy);
        while (NXP_read_gyro_fifo_count() == 0);
        gettimeofday(&start, NULL);
        NXP_read_gyro_data(dummy);
        while (NXP_read_gyro_fifo_count() < 26) usleep((int)((2/ODR)*1000000));  // sleep for about 2 periods
        while(1){
            cco = NXP_read_gyro_fifo_count();
            if(cco == 30) {
                gettimeofday(&stop, NULL);
                gyro_result += (float)((stop.tv_sec-start.tv_sec)+(float)(stop.tv_usec-start.tv_usec)/1000000.0);
                break;
            }
            if(cco > 30) {
                i -= 1;
                break;
            }
        }
    }
/*    for(int i = 0; i < loops; ++i){
        while (NXP_read_accel_fifo_count() != 0) NXP_read_accel_data(dummy);
        while (NXP_read_accel_fifo_count() == 0);
        gettimeofday(&start, NULL);
        NXP_read_accel_data(dummy);
        Py_BEGIN_ALLOW_THREADS
        // Py_BLOCK_THREADS // Py_UNBLOCK_THREADS
        while (NXP_read_accel_fifo_count() < 26) usleep((int)(2/ODR)*1000000);
        Py_END_ALLOW_THREADS
        while(1){
            cco = NXP_read_accel_fifo_count();
            if(cco == 30) {
                gettimeofday(&stop, NULL);
                accel_result += (float)((stop.tv_sec-start.tv_sec)+(float)(stop.tv_usec-start.tv_usec)/1000000.0);
                break;
            }
            if(cco > 30) {
                i -= 1;
                break;
            }
        }
    }
*/
    NXP_stop_fifos();
    sample_period = gyro_result/(30*loops);
//    if(DEBUG) accel_period = accel_result;
    return 0; 
}

float NXP_get_orientation(void){
    int count;
    float u0=0.0, u1=0.0, u2=0.0, z0=0.0, z1=0.0, z2=0.0;
    float gyro_data[3];
    float accel_data[3];
    
    if (NXP_fd_gyro != -1) return -999;
	if ((NXP_fd_gyro = open(I2CDEV, O_RDWR)) < 0) {
        NXP_fd_gyro = -1;
        return -999;
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
	    close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return -999;
	}
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x00);  // device to standby
    switch(FS_GYRO){
        case 1000:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x01);  
            NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_1000DPS;
            break;
        case 500:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x02); 
            NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_500DPS;
            break;
    }    
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x06); // 400 samples/sec

    if (NXP_fd_accel != -1) return -999;
	if ((NXP_fd_accel = open(I2CDEV, O_RDWR)) < 0) {
        NXP_fd_accel = -1;
        return -999;
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
	    close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return -999;
	}
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG2, 0x02);  // High resolution
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_MCTRL_REG1, 0x00); // disable magnetometer
    switch(FS_ACCEL){
        case 4:
            i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x01); 
            NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_4G;
            break;
        case 2:
            i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x00);
            NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_2G;
    }
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x0D); // 400 samples/sec

    usleep(5000);
    NXP_read_accel_data(accel_data); // ditch a sample or two
    NXP_read_gyro_data(gyro_data);
    usleep(5000);
    NXP_read_accel_data(accel_data);
    NXP_read_gyro_data(gyro_data);
	
    GYRO_BIAS[0] = 0.0;
    GYRO_BIAS[1] = 0.0;
    GYRO_BIAS[2] = 0.0;
 
    for(count = 0; count < 50; ++count){
        usleep(5000);
        NXP_read_accel_data(accel_data);
        NXP_read_gyro_data(gyro_data);

        u0 += gyro_data[0];
        u1 += gyro_data[1];
        u2 += gyro_data[2];
        z0 += accel_data[0];
        z1 += accel_data[1];
        z2 += accel_data[2];
    }
    u0 = u0/50.0;
    u1 = u1/50.0;
    u2 = u2/50.0;
    z0 = z0/50.0;
    z1 = z1/50.0;
    z2 = z2/50.0;
    
    GYRO_BIAS[0] = u0;
    GYRO_BIAS[1] = u1;
    GYRO_BIAS[2] = u2;
    
    printf("GXB:%+07.1f GYB:%+07.1f GZB:%+07.1f\n", u0,u1,u2); 
    
    close(NXP_fd_accel);
    NXP_fd_accel = -1;
    close(NXP_fd_gyro);
    NXP_fd_gyro = -1;
    
    return atan2(z1, z2) * RADIANS_TO_DEGREES_MULTIPLIER;
}

int NXP_start_fifos(int ODR, int gyro_fs, int accel_fs){
    if (NXP_fd_gyro != -1) return -1;
	if ((NXP_fd_gyro = open(I2CDEV, O_RDWR)) < 0) {
        NXP_fd_gyro = -1;
        return -1;
    }
  	if (ioctl(NXP_fd_gyro, I2C_SLAVE, NXP_GYRO_I2C_ADDRESS) < 0) {
	    close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
        return -1;
	}

    if (NXP_fd_accel != -1) return -1;
	if ((NXP_fd_accel = open(I2CDEV, O_RDWR)) < 0) {
        NXP_fd_accel = -1;
        return -1;
    }
  	if (ioctl(NXP_fd_accel, I2C_SLAVE, NXP_ACCEL_I2C_ADDRESS) < 0) {
	    close(NXP_fd_accel);
        NXP_fd_accel = -1;
        return -1;
	}

    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x00);  // device to standby
    switch(gyro_fs){
        case 1000:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x01);  
            NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_1000DPS;
            break;
        case 500:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG0, 0x02); 
            NXP_gyro_scale_factor = NXP_GYRO_SENSITIVITY_500DPS;
            break;
    }
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x80);  // enable fifo 
    switch(ODR){
        case 800:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x02);
            break;
        case 400:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x06);
            break;
        case 200:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x0A);
            break;
        case 100:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x0E);
            break;
        case 50:
            i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_CTRL_REG1, 0x12);
            break;
    }

    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x00);  // device to standby
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG2, 0x02);  // High resolution
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_MCTRL_REG1, 0x00); // disable magnetometer
    switch(accel_fs){
        case 4:
            i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x01); 
            NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_4G;
            break;
        case 2:
            i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_XYZ_DATA_CFG, 0x00);
            NXP_accel_scale_factor = NXP_ACCEL_SENSITIVITY_2G;
    }

    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x00);  // disable fifo
    i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_F_SETUP, 0x80);  // enable fifo

    switch(ODR){
        case 800:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x05);
            break;
        case 400:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x0D);
            break;
        case 200:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x15);
            break;
        case 100:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x1D);
            break;
        case 50:
            i2cWriteByteData(NXP_fd_accel, NXP_ACCEL_REGISTER_CTRL_REG1, 0x25);
            break;
    }
    return 0;
}

void NXP_stop_fifos(){
    if (NXP_fd_gyro != -1){
        i2cWriteByteData(NXP_fd_gyro,NXP_GYRO_REGISTER_F_SETUP, 0x00); 
        close(NXP_fd_gyro);
        NXP_fd_gyro = -1;
    }

    if (NXP_fd_accel != -1){
        i2cWriteByteData(NXP_fd_accel,NXP_ACCEL_REGISTER_F_SETUP, 0x00); 
        close(NXP_fd_accel);
        NXP_fd_accel = -1;
    }
}

void NXP_pull_data(){
    float gyro_data[3];
    float accel_data[3]; 
    float accTang;
    static float last_x_gyro = 0.0, last_angle = 0.0, angle_correction = 0.0;    
    int accel_count, number_to_pull, i, duplicate;

    static char local_outbuf[4000];
    static char remote_outbuf[1500];
    static char local_outbuf_line[150];
    static char remote_outbuf_line[150];
    int local_count = 0;
    int remote_count = 0;

    if (NXP_fd_gyro == -1 || NXP_fd_accel == -1 || fd_write_out == NULL) {
        fprintf(stderr,"Fifos not started or can't write to file\n");
        printf("EFIF:\n");
        printf("STPD:\n");
        NXP_stop_fifos();
        RUNNING = 0;
        LOOPSLEEP = 100000;
        if(fd_write_out != NULL){
            fclose(fd_write_out);
            fd_write_out = NULL;
        }
    }
    accel_count = NXP_read_accel_fifo_count();
    number_to_pull = NXP_read_gyro_fifo_count();
    if(number_to_pull == 0 || accel_count == 0) return;
    if((accel_count - number_to_pull) >= 2) NXP_read_accel_data(accel_data);   //acceleration ahead, ditch a sample from the acceleration fifo
    if (accel_count >= 31 || number_to_pull >= 31){ // overflow (or nearly so)
        printf("\nEOVF:\n");
    }

    for(i=0; i<number_to_pull; ++i){
        if (accel_count != 0) { // if we run out of accelerometer samples (because the fifo is running slower), use the last sample
            NXP_read_accel_data(accel_data);
            accel_count -= 1;
        }
        NXP_read_gyro_data(gyro_data);

        calculate(gyro_data[0],gyro_data[1],gyro_data[2], accel_data[0],accel_data[1],accel_data[2],sample_period);
        accTang = (gyro_data[0] - last_x_gyro)/sample_period;         // assumes rotation of bell is around x axis
        roll += angle_correction;                                     // need to correct reported angles to match
        if ((last_angle - roll) > 250 && angle_correction != 360){    // the system we are using 0 degrees = bell at
            angle_correction = 360;                                   // balance at handstroke, 360 degrees = bell at
            roll += 360.0;                                            // balance at backstroke. 180 degrees = BDC. 
        } else if ((last_angle - roll) < -250 && angle_correction != 0) {                                                 
            angle_correction = 0;
            roll -= 360.0;
        }
        sprintf(remote_outbuf_line, "LIVE:A:%+07.1f,R:%+07.1f,C:%+07.1f", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang);
        if (remote_count + strlen(remote_outbuf_line) > (sizeof remote_outbuf -2)) {
            printf("%s\n",remote_outbuf);
            remote_count = 0;
        }
        remote_count += sprintf(&remote_outbuf[remote_count],remote_outbuf_line);
 
        sprintf(local_outbuf_line,"A:%+07.1f,R:%+07.1f,C:%+07.1f,P:%+07.1f,Y:%+07.1f,AX:%+06.3f,AY:%+06.3f,AZ:%+06.3f,GX:%+07.1f,GY:%+07.1f,GZ:%+07.1f,X:%+06.3f,Y:%+06.3f,Z:%+06.3f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang, pitch, yaw, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2],a[0],a[1],a[2]);
        if (local_count + strlen(local_outbuf_line) > (sizeof local_outbuf -2)) {
            fputs(local_outbuf, fd_write_out);
            local_count = 0;
        }
        local_count += sprintf(&local_outbuf[local_count],local_outbuf_line);
       
//        fprintf(fd_write_out,"A:%+07.1f,R:%+07.1f,C:%+07.1f,P:%+07.1f,Y:%+07.1f,AX:%+06.3f,AY:%+06.3f,AZ:%+06.3f,GX:%+07.1f,GY:%+07.1f,GZ:%+07.1f,X:%+06.3f,Y:%+06.3f,Z:%+06.3f\n", roll, (gyro_data[0] + last_x_gyro)/2.0, accTang, pitch, yaw, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2],a[0],a[1],a[2]);
        last_x_gyro = gyro_data[0];
        last_angle = roll;
    }
    if(remote_count != 0) printf("%s\n",remote_outbuf);
    if(local_count != 0) fputs(local_outbuf, fd_write_out);
    OUT_COUNT += number_to_pull;
    
}

void NXP_read_gyro_data(float *values){
    __u8 returns[6];
    i2cReadBlockData(NXP_fd_gyro, NXP_GYRO_REGISTER_OUT_X_MSB, 6, returns);
    values[0]=(float)((returns[0] << 8) + returns[1]); 
    values[1]=(float)((returns[2] << 8) + returns[3]);
    values[2]=(float)((returns[4] << 8) + returns[5]);
    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] *= NXP_gyro_scale_factor;
    }
    if(SWAPXY){
        float temp;
        temp = values[0];
        values[0] = values[1];
        values[1] = temp;
    }
    values[0] *= ROTATIONS[0]; // deal with package being mounted differently
    values[1] *= ROTATIONS[1];
    values[2] *= ROTATIONS[2];

    values[0] -= GYRO_BIAS[0]; // adjust for gyro bias
    values[1] -= GYRO_BIAS[1];
    values[2] -= GYRO_BIAS[2];
}

void NXP_read_accel_data(float *values){
    __u8 returns[6];
    i2cReadBlockData(NXP_fd_accel, NXP_ACCEL_REGISTER_OUT_X_MSB, 6, returns);
    values[0]=(float)((returns[0] << 8) + returns[1]); 
    values[1]=(float)((returns[2] << 8) + returns[3]);
    values[2]=(float)((returns[4] << 8) + returns[5]);
    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] /= 4; // 14 bit data
        values[i] *= NXP_accel_scale_factor;
    }
    if(SWAPXY){
        float temp;
        temp = values[0];
        values[0] = values[1];
        values[1] = temp;
    }
    values[0] *= ROTATIONS[0]; // deal with package being mounted differently
    values[1] *= ROTATIONS[1];
    values[2] *= ROTATIONS[2];
}

int NXP_read_gyro_fifo_count(){
  return i2c_smbus_read_byte_data(NXP_fd_gyro, NXP_GYRO_REGISTER_F_STATUS) & 0x3F;
}

int NXP_read_accel_fifo_count(){
  return i2c_smbus_read_byte_data(NXP_fd_accel, NXP_GYRO_REGISTER_STATUS) & 0x3F;
}

/* 
This function does some magic with an Extended Kalman Filter to produce roll
pitch and yaw measurements from accelerometer/gyro data.  Of the filters tested,
it produced the best results.

Github is here: https://github.com/hhyyti/dcm-imu 

MIT licence but the readme includes this line:
    If you use the algorithm in any scientific context, please cite: Heikki Hyyti and Arto Visala, 
    "A DCM Based Attitude Estimation Algorithm for Low-Cost MEMS IMUs," International Journal 
    of Navigation and Observation, vol. 2015, Article ID 503814, 18 pages, 2015. http://dx.doi.org/10.1155/2015/503814

function inputs are 
Xgyro (u0 - in degrees/sec), Ygyro (u1 - in degrees/sec), Zgyro (u2 - in degrees/sec), 
Xaccel (z0 - in g), Yaccel (z1 - in g), Zaccel (z2 - in g),
interval (h - in seconds from the last time the function was called)
*/

void calculate(float u0, float u1, float u2, float z0, float z1,float z2, float h){

    static float x[] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    static float P[6][6] = {    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

// convert units
    u0 = u0 * DEGREES_TO_RADIANS_MULTIPLIER;
    u1 = u1 * DEGREES_TO_RADIANS_MULTIPLIER;
    u2 = u2 * DEGREES_TO_RADIANS_MULTIPLIER;
    
    z0 = z0 * g0;
    z1 = z1 * g0;
    z2 = z2 * g0;
    
    float x_last[] = {x[0], x[1], x[2]};
    float x_0 = x[0]-h*(u1*x[2]-u2*x[1]+x[1]*x[5]-x[2]*x[4]);
    float x_1 = x[1]+h*(u0*x[2]-u2*x[0]+x[0]*x[5]-x[2]*x[3]);
    float x_2 = x[2]-h*(u0*x[1]-u1*x[0]+x[0]*x[4]-x[1]*x[3]);
    float x_3 = x[3];
    float x_4 = x[4];
    float x_5 = x[5];

    float hh = h*h;
    float P_00 = P[0][0]-h*(P[0][5]*x[1]-P[0][4]*x[2]-P[4][0]*x[2]+P[5][0]*x[1]+P[0][2]*(u1-x[4])+P[2][0]*(u1-x[4])-P[0][1]*(u2-x[5])-P[1][0]*(u2-x[5]))+hh*(q_dcm2-x[1]*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]))+x[2]*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]))-(u1-x[4])*(P[4][2]*x[2]-P[5][2]*x[1]-P[2][2]*(u1-x[4])+P[1][2]*(u2-x[5]))+(u2-x[5])*(P[4][1]*x[2]-P[5][1]*x[1]-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5])));
    float P_01 = P[0][1]+h*(P[0][5]*x[0]-P[0][3]*x[2]+P[4][1]*x[2]-P[5][1]*x[1]+P[0][2]*(u0-x[3])-P[0][0]*(u2-x[5])-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5]))+hh*(x[0]*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]))-x[2]*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]))+(u0-x[3])*(P[4][2]*x[2]-P[5][2]*x[1]-P[2][2]*(u1-x[4])+P[1][2]*(u2-x[5]))-(u2-x[5])*(P[4][0]*x[2]-P[5][0]*x[1]-P[2][0]*(u1-x[4])+P[1][0]*(u2-x[5])));
    float P_02 = P[0][2]-h*(P[0][4]*x[0]-P[0][3]*x[1]-P[4][2]*x[2]+P[5][2]*x[1]+P[0][1]*(u0-x[3])-P[0][0]*(u1-x[4])+P[2][2]*(u1-x[4])-P[1][2]*(u2-x[5]))-hh*(x[0]*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]))-x[1]*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]))+(u0-x[3])*(P[4][1]*x[2]-P[5][1]*x[1]-P[2][1]*(u1-x[4])+P[1][1]*(u2-x[5]))-(u1-x[4])*(P[4][0]*x[2]-P[5][0]*x[1]-P[2][0]*(u1-x[4])+P[1][0]*(u2-x[5])));
    float P_03 = P[0][3]+h*(P[4][3]*x[2]-P[5][3]*x[1]-P[2][3]*(u1-x[4])+P[1][3]*(u2-x[5]));
    float P_04 = P[0][4]+h*(P[4][4]*x[2]-P[5][4]*x[1]-P[2][4]*(u1-x[4])+P[1][4]*(u2-x[5]));
    float P_05 = P[0][5]+h*(P[4][5]*x[2]-P[5][5]*x[1]-P[2][5]*(u1-x[4])+P[1][5]*(u2-x[5]));
    float P_10 = P[1][0]-h*(P[1][5]*x[1]-P[1][4]*x[2]+P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[1][2]*(u1-x[4])+P[0][0]*(u2-x[5])-P[1][1]*(u2-x[5]))+hh*(x[1]*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]))-x[2]*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]))+(u1-x[4])*(P[3][2]*x[2]-P[5][2]*x[0]-P[2][2]*(u0-x[3])+P[0][2]*(u2-x[5]))-(u2-x[5])*(P[3][1]*x[2]-P[5][1]*x[0]-P[2][1]*(u0-x[3])+P[0][1]*(u2-x[5])));
    float P_11 = P[1][1]+h*(P[1][5]*x[0]-P[1][3]*x[2]-P[3][1]*x[2]+P[5][1]*x[0]+P[1][2]*(u0-x[3])+P[2][1]*(u0-x[3])-P[0][1]*(u2-x[5])-P[1][0]*(u2-x[5]))+hh*(q_dcm2-x[0]*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]))+x[2]*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]))-(u0-x[3])*(P[3][2]*x[2]-P[5][2]*x[0]-P[2][2]*(u0-x[3])+P[0][2]*(u2-x[5]))+(u2-x[5])*(P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[0][0]*(u2-x[5])));
    float P_12 = P[1][2]-h*(P[1][4]*x[0]-P[1][3]*x[1]+P[3][2]*x[2]-P[5][2]*x[0]+P[1][1]*(u0-x[3])-P[2][2]*(u0-x[3])-P[1][0]*(u1-x[4])+P[0][2]*(u2-x[5]))+hh*(x[0]*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]))-x[1]*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]))+(u0-x[3])*(P[3][1]*x[2]-P[5][1]*x[0]-P[2][1]*(u0-x[3])+P[0][1]*(u2-x[5]))-(u1-x[4])*(P[3][0]*x[2]-P[5][0]*x[0]-P[2][0]*(u0-x[3])+P[0][0]*(u2-x[5])));
    float P_13 = P[1][3]-h*(P[3][3]*x[2]-P[5][3]*x[0]-P[2][3]*(u0-x[3])+P[0][3]*(u2-x[5]));
    float P_14 = P[1][4]-h*(P[3][4]*x[2]-P[5][4]*x[0]-P[2][4]*(u0-x[3])+P[0][4]*(u2-x[5]));
    float P_15 = P[1][5]-h*(P[3][5]*x[2]-P[5][5]*x[0]-P[2][5]*(u0-x[3])+P[0][5]*(u2-x[5]));
    float P_20 = P[2][0]-h*(P[2][5]*x[1]-P[3][0]*x[1]+P[4][0]*x[0]-P[2][4]*x[2]+P[1][0]*(u0-x[3])-P[0][0]*(u1-x[4])+P[2][2]*(u1-x[4])-P[2][1]*(u2-x[5]))-hh*(x[1]*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]))-x[2]*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]))+(u1-x[4])*(P[3][2]*x[1]-P[4][2]*x[0]-P[1][2]*(u0-x[3])+P[0][2]*(u1-x[4]))-(u2-x[5])*(P[3][1]*x[1]-P[4][1]*x[0]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4])));
    float P_21 = P[2][1]+h*(P[2][5]*x[0]+P[3][1]*x[1]-P[4][1]*x[0]-P[2][3]*x[2]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4])+P[2][2]*(u0-x[3])-P[2][0]*(u2-x[5]))+hh*(x[0]*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]))-x[2]*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]))+(u0-x[3])*(P[3][2]*x[1]-P[4][2]*x[0]-P[1][2]*(u0-x[3])+P[0][2]*(u1-x[4]))-(u2-x[5])*(P[3][0]*x[1]-P[4][0]*x[0]-P[1][0]*(u0-x[3])+P[0][0]*(u1-x[4])));
    float P_22 = P[2][2]-h*(P[2][4]*x[0]-P[2][3]*x[1]-P[3][2]*x[1]+P[4][2]*x[0]+P[1][2]*(u0-x[3])+P[2][1]*(u0-x[3])-P[0][2]*(u1-x[4])-P[2][0]*(u1-x[4]))+hh*(q_dcm2-x[0]*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]))+x[1]*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]))-(u0-x[3])*(P[3][1]*x[1]-P[4][1]*x[0]-P[1][1]*(u0-x[3])+P[0][1]*(u1-x[4]))+(u1-x[4])*(P[3][0]*x[1]-P[4][0]*x[0]-P[1][0]*(u0-x[3])+P[0][0]*(u1-x[4])));
    float P_23 = P[2][3]+h*(P[3][3]*x[1]-P[4][3]*x[0]-P[1][3]*(u0-x[3])+P[0][3]*(u1-x[4]));
    float P_24 = P[2][4]+h*(P[3][4]*x[1]-P[4][4]*x[0]-P[1][4]*(u0-x[3])+P[0][4]*(u1-x[4]));
    float P_25 = P[2][5]+h*(P[3][5]*x[1]-P[4][5]*x[0]-P[1][5]*(u0-x[3])+P[0][5]*(u1-x[4]));
    float P_30 = P[3][0]-h*(P[3][5]*x[1]-P[3][4]*x[2]+P[3][2]*(u1-x[4])-P[3][1]*(u2-x[5]));
    float P_31 = P[3][1]+h*(P[3][5]*x[0]-P[3][3]*x[2]+P[3][2]*(u0-x[3])-P[3][0]*(u2-x[5]));
    float P_32 = P[3][2]-h*(P[3][4]*x[0]-P[3][3]*x[1]+P[3][1]*(u0-x[3])-P[3][0]*(u1-x[4]));
    float P_33 = P[3][3]+hh*q_gyro_bias2;
    float P_34 = P[3][4];
    float P_35 = P[3][5];
    float P_40 = P[4][0]-h*(P[4][5]*x[1]-P[4][4]*x[2]+P[4][2]*(u1-x[4])-P[4][1]*(u2-x[5]));
    float P_41 = P[4][1]+h*(P[4][5]*x[0]-P[4][3]*x[2]+P[4][2]*(u0-x[3])-P[4][0]*(u2-x[5]));
    float P_42 = P[4][2]-h*(P[4][4]*x[0]-P[4][3]*x[1]+P[4][1]*(u0-x[3])-P[4][0]*(u1-x[4]));
    float P_43 = P[4][3];
    float P_44 = P[4][4]+hh*q_gyro_bias2;
    float P_45 = P[4][5];
    float P_50 = P[5][0]-h*(P[5][5]*x[1]-P[5][4]*x[2]+P[5][2]*(u1-x[4])-P[5][1]*(u2-x[5]));
    float P_51 = P[5][1]+h*(P[5][5]*x[0]-P[5][3]*x[2]+P[5][2]*(u0-x[3])-P[5][0]*(u2-x[5]));
    float P_52 = P[5][2]-h*(P[5][4]*x[0]-P[5][3]*x[1]+P[5][1]*(u0-x[3])-P[5][0]*(u1-x[4]));
    float P_53 = P[5][3];
    float P_54 = P[5][4];
    float P_55 = P[5][5]+hh*q_gyro_bias2;

    // kalman innovation
    float y0 = z0-g0*x_0;
    float y1 = z1-g0*x_1;
    float y2 = z2-g0*x_2;

    float a_len = sqrt(y0*y0+y1*y1+y2*y2);

    float S00 = r_acc2+a_len*r_a2+P_00*g0_2;
    float S01 = P_01*g0_2;
    float S02 = P_02*g0_2;
    float S10 = P_10*g0_2;
    float S11 = r_acc2+a_len*r_a2+P_11*g0_2;
    float S12 = P_12*g0_2;
    float S20 = P_20*g0_2;
    float S21 = P_21*g0_2;
    float S22 = r_acc2+a_len*r_a2+P_22*g0_2;

    // Kalman gain
    float invPart = 1.0 / (S00*S11*S22-S00*S12*S21-S01*S10*S22+S01*S12*S20+S02*S10*S21-S02*S11*S20);
    float K00 = (g0*(P_02*S10*S21-P_02*S11*S20-P_01*S10*S22+P_01*S12*S20+P_00*S11*S22-P_00*S12*S21))*invPart;
    float K01 = -(g0*(P_02*S00*S21-P_02*S01*S20-P_01*S00*S22+P_01*S02*S20+P_00*S01*S22-P_00*S02*S21))*invPart;
    float K02 = (g0*(P_02*S00*S11-P_02*S01*S10-P_01*S00*S12+P_01*S02*S10+P_00*S01*S12-P_00*S02*S11))*invPart;
    float K10 = (g0*(P_12*S10*S21-P_12*S11*S20-P_11*S10*S22+P_11*S12*S20+P_10*S11*S22-P_10*S12*S21))*invPart;
    float K11 = -(g0*(P_12*S00*S21-P_12*S01*S20-P_11*S00*S22+P_11*S02*S20+P_10*S01*S22-P_10*S02*S21))*invPart;
    float K12 = (g0*(P_12*S00*S11-P_12*S01*S10-P_11*S00*S12+P_11*S02*S10+P_10*S01*S12-P_10*S02*S11))*invPart;
    float K20 = (g0*(P_22*S10*S21-P_22*S11*S20-P_21*S10*S22+P_21*S12*S20+P_20*S11*S22-P_20*S12*S21))*invPart;
    float K21 = -(g0*(P_22*S00*S21-P_22*S01*S20-P_21*S00*S22+P_21*S02*S20+P_20*S01*S22-P_20*S02*S21))*invPart;
    float K22 = (g0*(P_22*S00*S11-P_22*S01*S10-P_21*S00*S12+P_21*S02*S10+P_20*S01*S12-P_20*S02*S11))*invPart;
    float K30 = (g0*(P_32*S10*S21-P_32*S11*S20-P_31*S10*S22+P_31*S12*S20+P_30*S11*S22-P_30*S12*S21))*invPart;
    float K31 = -(g0*(P_32*S00*S21-P_32*S01*S20-P_31*S00*S22+P_31*S02*S20+P_30*S01*S22-P_30*S02*S21))*invPart;
    float K32 = (g0*(P_32*S00*S11-P_32*S01*S10-P_31*S00*S12+P_31*S02*S10+P_30*S01*S12-P_30*S02*S11))*invPart;
    float K40 = (g0*(P_42*S10*S21-P_42*S11*S20-P_41*S10*S22+P_41*S12*S20+P_40*S11*S22-P_40*S12*S21))*invPart;
    float K41 = -(g0*(P_42*S00*S21-P_42*S01*S20-P_41*S00*S22+P_41*S02*S20+P_40*S01*S22-P_40*S02*S21))*invPart;
    float K42 = (g0*(P_42*S00*S11-P_42*S01*S10-P_41*S00*S12+P_41*S02*S10+P_40*S01*S12-P_40*S02*S11))*invPart;
    float K50 = (g0*(P_52*S10*S21-P_52*S11*S20-P_51*S10*S22+P_51*S12*S20+P_50*S11*S22-P_50*S12*S21))*invPart;
    float K51 = -(g0*(P_52*S00*S21-P_52*S01*S20-P_51*S00*S22+P_51*S02*S20+P_50*S01*S22-P_50*S02*S21))*invPart;
    float K52 = (g0*(P_52*S00*S11-P_52*S01*S10-P_51*S00*S12+P_51*S02*S10+P_50*S01*S12-P_50*S02*S11))*invPart;

    // update a posteriori
    x[0] = x_0+K00*y0+K01*y1+K02*y2;
    x[1] = x_1+K10*y0+K11*y1+K12*y2;
    x[2] = x_2+K20*y0+K21*y1+K22*y2;
    x[3] = x_3+K30*y0+K31*y1+K32*y2;
    x[4] = x_4+K40*y0+K41*y1+K42*y2;
    x[5] = x_5+K50*y0+K51*y1+K52*y2;

    //  update a posteriori covariance
    float r_adab = (r_acc2+a_len*r_a2);
    float P__00 = P_00-g0*(K00*P_00*2.0+K01*P_01+K01*P_10+K02*P_02+K02*P_20)+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+g0_2*(K00*(K00*P_00+K01*P_10+K02*P_20)+K01*(K00*P_01+K01*P_11+K02*P_21)+K02*(K00*P_02+K01*P_12+K02*P_22));
    float P__01 = P_01-g0*(K00*P_01+K01*P_11+K02*P_21+K10*P_00+K11*P_01+K12*P_02)+g0_2*(K10*(K00*P_00+K01*P_10+K02*P_20)+K11*(K00*P_01+K01*P_11+K02*P_21)+K12*(K00*P_02+K01*P_12+K02*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
    float P__02 = P_02-g0*(K00*P_02+K01*P_12+K02*P_22+K20*P_00+K21*P_01+K22*P_02)+g0_2*(K20*(K00*P_00+K01*P_10+K02*P_20)+K21*(K00*P_01+K01*P_11+K02*P_21)+K22*(K00*P_02+K01*P_12+K02*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
    float P__03 = P_03-g0*(K00*P_03+K01*P_13+K02*P_23+K30*P_00+K31*P_01+K32*P_02)+g0_2*(K30*(K00*P_00+K01*P_10+K02*P_20)+K31*(K00*P_01+K01*P_11+K02*P_21)+K32*(K00*P_02+K01*P_12+K02*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
    float P__04 = P_04-g0*(K00*P_04+K01*P_14+K02*P_24+K40*P_00+K41*P_01+K42*P_02)+g0_2*(K40*(K00*P_00+K01*P_10+K02*P_20)+K41*(K00*P_01+K01*P_11+K02*P_21)+K42*(K00*P_02+K01*P_12+K02*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
    float P__05 = P_05-g0*(K00*P_05+K01*P_15+K02*P_25+K50*P_00+K51*P_01+K52*P_02)+g0_2*(K50*(K00*P_00+K01*P_10+K02*P_20)+K51*(K00*P_01+K01*P_11+K02*P_21)+K52*(K00*P_02+K01*P_12+K02*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
    float P__10 = P_10-g0*(K00*P_10+K01*P_11+K02*P_12+K10*P_00+K11*P_10+K12*P_20)+g0_2*(K00*(K10*P_00+K11*P_10+K12*P_20)+K01*(K10*P_01+K11*P_11+K12*P_21)+K02*(K10*P_02+K11*P_12+K12*P_22))+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
    float P__11 = P_11-g0*(K10*P_01+K10*P_10+K11*P_11*2.0+K12*P_12+K12*P_21)+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+g0_2*(K10*(K10*P_00+K11*P_10+K12*P_20)+K11*(K10*P_01+K11*P_11+K12*P_21)+K12*(K10*P_02+K11*P_12+K12*P_22));
    float P__12 = P_12-g0*(K10*P_02+K11*P_12+K12*P_22+K20*P_10+K21*P_11+K22*P_12)+g0_2*(K20*(K10*P_00+K11*P_10+K12*P_20)+K21*(K10*P_01+K11*P_11+K12*P_21)+K22*(K10*P_02+K11*P_12+K12*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
    float P__13 = P_13-g0*(K10*P_03+K11*P_13+K12*P_23+K30*P_10+K31*P_11+K32*P_12)+g0_2*(K30*(K10*P_00+K11*P_10+K12*P_20)+K31*(K10*P_01+K11*P_11+K12*P_21)+K32*(K10*P_02+K11*P_12+K12*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
    float P__14 = P_14-g0*(K10*P_04+K11*P_14+K12*P_24+K40*P_10+K41*P_11+K42*P_12)+g0_2*(K40*(K10*P_00+K11*P_10+K12*P_20)+K41*(K10*P_01+K11*P_11+K12*P_21)+K42*(K10*P_02+K11*P_12+K12*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
    float P__15 = P_15-g0*(K10*P_05+K11*P_15+K12*P_25+K50*P_10+K51*P_11+K52*P_12)+g0_2*(K50*(K10*P_00+K11*P_10+K12*P_20)+K51*(K10*P_01+K11*P_11+K12*P_21)+K52*(K10*P_02+K11*P_12+K12*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
    float P__20 = P_20-g0*(K00*P_20+K01*P_21+K02*P_22+K20*P_00+K21*P_10+K22*P_20)+g0_2*(K00*(K20*P_00+K21*P_10+K22*P_20)+K01*(K20*P_01+K21*P_11+K22*P_21)+K02*(K20*P_02+K21*P_12+K22*P_22))+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
    float P__21 = P_21-g0*(K10*P_20+K11*P_21+K12*P_22+K20*P_01+K21*P_11+K22*P_21)+g0_2*(K10*(K20*P_00+K21*P_10+K22*P_20)+K11*(K20*P_01+K21*P_11+K22*P_21)+K12*(K20*P_02+K21*P_12+K22*P_22))+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
    float P__22 = P_22-g0*(K20*P_02+K20*P_20+K21*P_12+K21*P_21+K22*P_22*2.0)+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+g0_2*(K20*(K20*P_00+K21*P_10+K22*P_20)+K21*(K20*P_01+K21*P_11+K22*P_21)+K22*(K20*P_02+K21*P_12+K22*P_22));
    float P__23 = P_23-g0*(K20*P_03+K21*P_13+K22*P_23+K30*P_20+K31*P_21+K32*P_22)+g0_2*(K30*(K20*P_00+K21*P_10+K22*P_20)+K31*(K20*P_01+K21*P_11+K22*P_21)+K32*(K20*P_02+K21*P_12+K22*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
    float P__24 = P_24-g0*(K20*P_04+K21*P_14+K22*P_24+K40*P_20+K41*P_21+K42*P_22)+g0_2*(K40*(K20*P_00+K21*P_10+K22*P_20)+K41*(K20*P_01+K21*P_11+K22*P_21)+K42*(K20*P_02+K21*P_12+K22*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
    float P__25 = P_25-g0*(K20*P_05+K21*P_15+K22*P_25+K50*P_20+K51*P_21+K52*P_22)+g0_2*(K50*(K20*P_00+K21*P_10+K22*P_20)+K51*(K20*P_01+K21*P_11+K22*P_21)+K52*(K20*P_02+K21*P_12+K22*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
    float P__30 = P_30-g0*(K00*P_30+K01*P_31+K02*P_32+K30*P_00+K31*P_10+K32*P_20)+g0_2*(K00*(K30*P_00+K31*P_10+K32*P_20)+K01*(K30*P_01+K31*P_11+K32*P_21)+K02*(K30*P_02+K31*P_12+K32*P_22))+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
    float P__31 = P_31-g0*(K10*P_30+K11*P_31+K12*P_32+K30*P_01+K31*P_11+K32*P_21)+g0_2*(K10*(K30*P_00+K31*P_10+K32*P_20)+K11*(K30*P_01+K31*P_11+K32*P_21)+K12*(K30*P_02+K31*P_12+K32*P_22))+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
    float P__32 = P_32-g0*(K20*P_30+K21*P_31+K22*P_32+K30*P_02+K31*P_12+K32*P_22)+g0_2*(K20*(K30*P_00+K31*P_10+K32*P_20)+K21*(K30*P_01+K31*P_11+K32*P_21)+K22*(K30*P_02+K31*P_12+K32*P_22))+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
    float P__33 = P_33-g0*(K30*P_03+K31*P_13+K30*P_30+K31*P_31+K32*P_23+K32*P_32)+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+g0_2*(K30*(K30*P_00+K31*P_10+K32*P_20)+K31*(K30*P_01+K31*P_11+K32*P_21)+K32*(K30*P_02+K31*P_12+K32*P_22));
    float P__34 = P_34-g0*(K30*P_04+K31*P_14+K32*P_24+K40*P_30+K41*P_31+K42*P_32)+g0_2*(K40*(K30*P_00+K31*P_10+K32*P_20)+K41*(K30*P_01+K31*P_11+K32*P_21)+K42*(K30*P_02+K31*P_12+K32*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
    float P__35 = P_35-g0*(K30*P_05+K31*P_15+K32*P_25+K50*P_30+K51*P_31+K52*P_32)+g0_2*(K50*(K30*P_00+K31*P_10+K32*P_20)+K51*(K30*P_01+K31*P_11+K32*P_21)+K52*(K30*P_02+K31*P_12+K32*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
    float P__40 = P_40-g0*(K00*P_40+K01*P_41+K02*P_42+K40*P_00+K41*P_10+K42*P_20)+g0_2*(K00*(K40*P_00+K41*P_10+K42*P_20)+K01*(K40*P_01+K41*P_11+K42*P_21)+K02*(K40*P_02+K41*P_12+K42*P_22))+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
    float P__41 = P_41-g0*(K10*P_40+K11*P_41+K12*P_42+K40*P_01+K41*P_11+K42*P_21)+g0_2*(K10*(K40*P_00+K41*P_10+K42*P_20)+K11*(K40*P_01+K41*P_11+K42*P_21)+K12*(K40*P_02+K41*P_12+K42*P_22))+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
    float P__42 = P_42-g0*(K20*P_40+K21*P_41+K22*P_42+K40*P_02+K41*P_12+K42*P_22)+g0_2*(K20*(K40*P_00+K41*P_10+K42*P_20)+K21*(K40*P_01+K41*P_11+K42*P_21)+K22*(K40*P_02+K41*P_12+K42*P_22))+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
    float P__43 = P_43-g0*(K30*P_40+K31*P_41+K32*P_42+K40*P_03+K41*P_13+K42*P_23)+g0_2*(K30*(K40*P_00+K41*P_10+K42*P_20)+K31*(K40*P_01+K41*P_11+K42*P_21)+K32*(K40*P_02+K41*P_12+K42*P_22))+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
    float P__44 = P_44-g0*(K40*P_04+K41*P_14+K40*P_40+K42*P_24+K41*P_41+K42*P_42)+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+g0_2*(K40*(K40*P_00+K41*P_10+K42*P_20)+K41*(K40*P_01+K41*P_11+K42*P_21)+K42*(K40*P_02+K41*P_12+K42*P_22));
    float P__45 = P_45-g0*(K40*P_05+K41*P_15+K42*P_25+K50*P_40+K51*P_41+K52*P_42)+g0_2*(K50*(K40*P_00+K41*P_10+K42*P_20)+K51*(K40*P_01+K41*P_11+K42*P_21)+K52*(K40*P_02+K41*P_12+K42*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
    float P__50 = P_50-g0*(K00*P_50+K01*P_51+K02*P_52+K50*P_00+K51*P_10+K52*P_20)+g0_2*(K00*(K50*P_00+K51*P_10+K52*P_20)+K01*(K50*P_01+K51*P_11+K52*P_21)+K02*(K50*P_02+K51*P_12+K52*P_22))+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
    float P__51 = P_51-g0*(K10*P_50+K11*P_51+K12*P_52+K50*P_01+K51*P_11+K52*P_21)+g0_2*(K10*(K50*P_00+K51*P_10+K52*P_20)+K11*(K50*P_01+K51*P_11+K52*P_21)+K12*(K50*P_02+K51*P_12+K52*P_22))+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
    float P__52 = P_52-g0*(K20*P_50+K21*P_51+K22*P_52+K50*P_02+K51*P_12+K52*P_22)+g0_2*(K20*(K50*P_00+K51*P_10+K52*P_20)+K21*(K50*P_01+K51*P_11+K52*P_21)+K22*(K50*P_02+K51*P_12+K52*P_22))+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
    float P__53 = P_53-g0*(K30*P_50+K31*P_51+K32*P_52+K50*P_03+K51*P_13+K52*P_23)+g0_2*(K30*(K50*P_00+K51*P_10+K52*P_20)+K31*(K50*P_01+K51*P_11+K52*P_21)+K32*(K50*P_02+K51*P_12+K52*P_22))+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
    float P__54 = P_54-g0*(K40*P_50+K41*P_51+K42*P_52+K50*P_04+K51*P_14+K52*P_24)+g0_2*(K40*(K50*P_00+K51*P_10+K52*P_20)+K41*(K50*P_01+K51*P_11+K52*P_21)+K42*(K50*P_02+K51*P_12+K52*P_22))+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
    float P__55 = P_55-g0*(K50*P_05+K51*P_15+K52*P_25+K50*P_50+K51*P_51+K52*P_52)+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+g0_2*(K50*(K50*P_00+K51*P_10+K52*P_20)+K51*(K50*P_01+K51*P_11+K52*P_21)+K52*(K50*P_02+K51*P_12+K52*P_22));

    float xlen = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    float invlen3 = 1.0/(xlen*xlen*xlen);
    float invlen32 = (invlen3*invlen3);

    float x1_x2 = (x[1]*x[1]+x[2]*x[2]);
    float x0_x2 = (x[0]*x[0]+x[2]*x[2]);
    float x0_x1 = (x[0]*x[0]+x[1]*x[1]);

    // normalized a posteriori covariance
    P[0][0] = invlen32*(-x1_x2*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[0]*x[1]*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2])+x[0]*x[2]*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2]));
    P[0][1] = invlen32*(-x0_x2*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2])+x[0]*x[1]*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[1]*x[2]*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2]));
    P[0][2] = invlen32*(-x0_x1*(-P__02*x1_x2+P__12*x[0]*x[1]+P__22*x[0]*x[2])+x[0]*x[2]*(-P__00*x1_x2+P__10*x[0]*x[1]+P__20*x[0]*x[2])+x[1]*x[2]*(-P__01*x1_x2+P__11*x[0]*x[1]+P__21*x[0]*x[2]));
    P[0][3] = -invlen3*(-P__03*x1_x2+P__13*x[0]*x[1]+P__23*x[0]*x[2]);
    P[0][4] = -invlen3*(-P__04*x1_x2+P__14*x[0]*x[1]+P__24*x[0]*x[2]);
    P[0][5] = -invlen3*(-P__05*x1_x2+P__15*x[0]*x[1]+P__25*x[0]*x[2]);
    P[1][0] = invlen32*(-x1_x2*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[0]*x[1]*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2])+x[0]*x[2]*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2]));
    P[1][1] = invlen32*(-x0_x2*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2])+x[0]*x[1]*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[1]*x[2]*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2]));
    P[1][2] = invlen32*(-x0_x1*(-P__12*x0_x2+P__02*x[0]*x[1]+P__22*x[1]*x[2])+x[0]*x[2]*(-P__10*x0_x2+P__00*x[0]*x[1]+P__20*x[1]*x[2])+x[1]*x[2]*(-P__11*x0_x2+P__01*x[0]*x[1]+P__21*x[1]*x[2]));
    P[1][3] = -invlen3*(-P__13*x0_x2+P__03*x[0]*x[1]+P__23*x[1]*x[2]);
    P[1][4] = -invlen3*(-P__14*x0_x2+P__04*x[0]*x[1]+P__24*x[1]*x[2]);
    P[1][5] = -invlen3*(-P__15*x0_x2+P__05*x[0]*x[1]+P__25*x[1]*x[2]);
    P[2][0] = invlen32*(-x1_x2*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[0]*x[1]*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2])+x[0]*x[2]*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2]));
    P[2][1] = invlen32*(-x0_x2*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2])+x[0]*x[1]*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[1]*x[2]*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2]));
    P[2][2] = invlen32*(-x0_x1*(-P__22*x0_x1+P__02*x[0]*x[2]+P__12*x[1]*x[2])+x[0]*x[2]*(-P__20*x0_x1+P__00*x[0]*x[2]+P__10*x[1]*x[2])+x[1]*x[2]*(-P__21*x0_x1+P__01*x[0]*x[2]+P__11*x[1]*x[2]));
    P[2][3] = -invlen3*(-P__23*x0_x1+P__03*x[0]*x[2]+P__13*x[1]*x[2]);
    P[2][4] = -invlen3*(-P__24*x0_x1+P__04*x[0]*x[2]+P__14*x[1]*x[2]);
    P[2][5] = -invlen3*(-P__25*x0_x1+P__05*x[0]*x[2]+P__15*x[1]*x[2]);
    P[3][0] = -invlen3*(-P__30*x1_x2+P__31*x[0]*x[1]+P__32*x[0]*x[2]);
    P[3][1] = -invlen3*(-P__31*x0_x2+P__30*x[0]*x[1]+P__32*x[1]*x[2]);
    P[3][2] = -invlen3*(-P__32*x0_x1+P__30*x[0]*x[2]+P__31*x[1]*x[2]);
    P[3][3] = P__33;
    P[3][4] = P__34;
    P[3][5] = P__35;
    P[4][0] = -invlen3*(-P__40*x1_x2+P__41*x[0]*x[1]+P__42*x[0]*x[2]);
    P[4][1] = -invlen3*(-P__41*x0_x2+P__40*x[0]*x[1]+P__42*x[1]*x[2]);
    P[4][2] = -invlen3*(-P__42*x0_x1+P__40*x[0]*x[2]+P__41*x[1]*x[2]);
    P[4][3] = P__43;
    P[4][4] = P__44;
    P[4][5] = P__45;
    P[5][0] = -invlen3*(-P__50*x1_x2+P__51*x[0]*x[1]+P__52*x[0]*x[2]);
    P[5][1] = -invlen3*(-P__51*x0_x2+P__50*x[0]*x[1]+P__52*x[1]*x[2]);
    P[5][2] = -invlen3*(-P__52*x0_x1+P__50*x[0]*x[2]+P__51*x[1]*x[2]);
    P[5][3] = P__53;
    P[5][4] = P__54;
    P[5][5] = P__55;

    // normalized a posteriori state
    x[0] = x[0]/xlen;
    x[1] = x[1]/xlen;
    x[2] = x[2]/xlen;

    // compute Euler angles (not exactly a part of the extended Kalman filter)
    // yaw integration through full rotation matrix
    float u_nb1 = u1 - x[4];
    float u_nb2 = u2 - x[5];

    float cy = cos(yaw); //old angles (last state before integration)
    float sy = sin(yaw);
    float d = sqrt(x_last[1]*x_last[1] + x_last[2]*x_last[2]);
    float d_inv = 1.0 / d;

    // compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
    float R11 = cy * d;
    float R12 = -(x_last[2]*sy + x_last[0]*x_last[1]*cy) * d_inv;
    float R13 = (x_last[1]*sy - x_last[0]*x_last[2]*cy) * d_inv;
    float R21 = sy * d;
    float R22 = (x_last[2]*cy - x_last[0]*x_last[1]*sy) * d_inv;
    float R23 = -(x_last[1]*cy + x_last[0]*x_last[2]*sy) * d_inv;

    // update needed parts of R for yaw computation
    float R11_new = R11 + h*(u_nb2*R12 - u_nb1*R13);
    float R21_new = R21 + h*(u_nb2*R22 - u_nb1*R23);
    yaw = atan2(R21_new,R11_new) * RADIANS_TO_DEGREES_MULTIPLIER;

    // compute new pitch and roll angles from a posteriori states
    pitch = asin(-x[0]) * RADIANS_TO_DEGREES_MULTIPLIER;
    roll = atan2(x[1],x[2]) * RADIANS_TO_DEGREES_MULTIPLIER;

    // save the estimated non-gravitational acceleration
    a[0] = z0-x[0]*g0;
    a[1] = z1-x[1]*g0;
    a[2] = z2-x[2]*g0;

}

/*
  i2c functions
*/


__s32 i2cReadInt(int fd, __u8 address) {
	__s32 res = i2c_smbus_read_word_data(fd, address);
	if (0 > res) {
		close(fd);
		exit(1);
	}
	res = ((res<<8) & 0xFF00) | ((res>>8) & 0xFF);
	return res;
}

//Write a byte
void i2cWriteByteData(int fd, __u8 address, __u8 value) {
	if (0 > i2c_smbus_write_byte_data(fd, address, value)) {
		close(fd);
		exit(1);
	}
}

// Read a block of data
void i2cReadBlockData(int fd, __u8 address, __u8 length, __u8 *values) {
	if (0 > i2c_smbus_read_i2c_block_data(fd, address,length,values)) {
		close(fd);
		exit(1);
	}
}
    
__s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data) {
	struct i2c_smbus_ioctl_data args;
	__s32 err;
	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	err = ioctl(file, I2C_SMBUS, &args);
	if (err == -1)
		err = -errno;
	return err;
}

__s32 i2c_smbus_write_quick(int file, __u8 value) {
	return i2c_smbus_access(file, value, 0, I2C_SMBUS_QUICK, NULL);
}

__s32 i2c_smbus_read_byte(int file) {
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	if (err < 0)
		return err;
	return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte(int file, __u8 value) {
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

__s32 i2c_smbus_read_byte_data(int file, __u8 command) {
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0)
		return err;
	return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value) {
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

__s32 i2c_smbus_read_word_data(int file, __u8 command) {
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data);
	if (err < 0)
		return err;
	return 0x0FFFF & data.word;
}

__s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value){
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}

__s32 i2c_smbus_process_call(int file, __u8 command, __u16 value) {
	union i2c_smbus_data data;
	data.word = value;
	if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_PROC_CALL, &data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

/* Returns the number of read bytes */
__s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values){
	union i2c_smbus_data data;
	int i, err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BLOCK_DATA, &data);
	if (err < 0)
		return err;

	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

__s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length, const __u8 *values) {
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BLOCK_DATA, &data);
}

/* Returns the number of read bytes */
__s32 i2c_smbus_read_i2c_block_data(int file, __u8 command, __u8 length, __u8 *values) {
	union i2c_smbus_data data;
	int i, err;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       length == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN :
				I2C_SMBUS_I2C_BLOCK_DATA, &data);
	if (err < 0)
		return err;
	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

__s32 i2c_smbus_write_i2c_block_data(int file, __u8 command, __u8 length, const __u8 *values) {
	union i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_I2C_BLOCK_BROKEN, &data);
}

/* Returns the number of read bytes */
__s32 i2c_smbus_block_process_call(int file, __u8 command, __u8 length, __u8 *values) {
	union i2c_smbus_data data;
	int i, err;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	err = i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
			       I2C_SMBUS_BLOCK_PROC_CALL, &data);
	if (err < 0)
		return err;
	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

