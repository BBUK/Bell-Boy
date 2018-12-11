//gcc MPU6050_calibrate.c -o MPU6050_calibrate
/*
WARN=-Wfatal-errors -Wall -Wextra -Wconversion -Wunused -Wundef -Wcast-qual -Wredundant-decls -Wunreachable-code -Wwrite-strings -Warray-bounds -Wstrict-aliasing=3 -Wstrict-overflow=1 -Wstrict-prototypes -Winline -Wshadow -Wswitch -Wmissing-include-dirs -Woverlength-strings -Wpacked -Wdisabled-optimization -Wmissing-prototypes -Wformat=2 -Winit-self -Wmissing-declarations -Wunused-parameter -Wlogical-op -Wuninitialized -Wnested-externs -Wpointer-arith -Wdouble-promotion -Wunused-macros -Wunused-function -Wunsafe-loop-optimizations -Wnull-dereference -Wduplicated-cond -Wshift-overflow=2 -Wnonnull -Wcast-align -Warray-bounds=2
*/

// https://github.com/rpicopter/MotionSensorExample/blob/master/libs/I2Cdev/I2Cdev.c
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

/*
* This program is a calibration routine for IMU accelerometer sensors.  
* The program only needs to be run once after the Bell Boy device has been
* constructed.  The calibration "should" remain valid for the life of the device.
* There is, however, no harm in rerunning the calibration.
*
* This program works by using gravity measurements in the positive and negative
* directions of the Y and Z axes.  The X axis is not used as its accelerometer
* (and its calibration) is irrelevant for the Bell Boy application.
* 
* The result is an offset (which bb_dcmimu adds to the raw readings) 
* and a scale (which bb_dcmimu multiplies the raw readings by).
* 
* To use this get a completely flat surface ready.  Test the "flatness" of the 
* surface using a bulls eye spirit level.  Place one large flat side of the Bell Boy
* device on the surface, switch it on, connect to its wifi and run the bb_calibrate
* program over ssh. Note that you will be turning the device so that it is on
* the other flat side and on each of the "long" edges.  I have called these four
* orientations North, South, East and West.
* The program starts by taking some gyroscope measurements (overall biasses).  These are 
* not used in the calibration itself but are used to work out whether the devioe
* is still enough to make an accurate accelerometer reading.
* The program then takes the readings for the orientation it is in.  You will 
* probably see something like "insufficiently still" a few times before a reading
* is taken and reported.  Once this happens turn the device to one of the other
* orientations and wait for a reading to be taken (and so on).
* If you get a report "Not in the right position" this simply says that the device is still
* but probably not oriented correctly.  Wait to see if the error clears and also check
* the flatness of the surface you have placed the device on.
* The program exits once it has taken all its measurements and saves the Y and Z offset
* and scale values (four values) to /boot/bb_calibrations.
* The saved file is read and used by bb_dcmimu (the main grabber program).  If, however,
* bb_dcmimu does not see the file, it just uses uncalibrated values - you may
* not notice any difference!
*
* Finally note that the offset and scale values are temperature dependent.  If you
* wanted to do some scientific analysis then the values should be measured at different
* temperatures and some form of linear regression analysis should take place so that
* you would know what scale and offset values to apply.  That's why the IMUs have
* a user accessible temperature reading on them.  Testing showed that the effect of
* temperature was really small (to small to worry about really).  
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
#include "MPU6050_calibrate.h"

#ifndef NULL
#define NULL 0
#endif

#define MPU6050_1_I2C_ADDRESS 0x68
#define MPU6050_2_I2C_ADDRESS 0x69

#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif

#define I2CDEV "/dev/i2c-1"

int MPU6050_1_fd = -1;
int MPU6050_2_fd = -1;

float MPU6050_gyro_scale_factor = 0;
float MPU6050_accel_scale_factor = 0;
float threshold = 4.0;

float GYRO_BIAS[3] = { 0.0, 0.0, 0.0 };

FILE *fd_write_out;

volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

int main(int argc, char const *argv[]){
    float gyro_values[3];
    float return_values[6];
    float north_1,south_1,east_1,west_1;
    float north_2,south_2,east_2,west_2;

    int count = 0, nsew = 0;
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);
        
    if(MPU6050_setup() < 0) return -1;
    printf("Getting gyro biasses\n");

    while(sig_exit == 0){
        for(count = 0; count < 4; ++count){ // must be still for about 2 seconds
            if(MPU6050_get_calibration_values(gyro_values, return_values) < 0) break;
        }
        if(count != 4) {
            printf("Insufficiently still ...%f,%f,%f\n",gyro_values[0],gyro_values[1],gyro_values[2]);
            usleep(500000);
            continue;
        }

        GYRO_BIAS[0] = gyro_values[0];
        GYRO_BIAS[1] = gyro_values[1];
        GYRO_BIAS[2] = gyro_values[2];
        printf("Biasses read OK\n");
        threshold = 1.0;
        break;
    }

    while(sig_exit == 0){
        for(count = 0; count < 4; ++count){ // must be still for about 2 seconds
            if(MPU6050_get_calibration_values(gyro_values, return_values) < 0) break;
        }
        if(count != 4) {
            printf("Insufficiently still ...%f,%f,%f\n",gyro_values[0],gyro_values[1],gyro_values[2]);
            usleep(500000);
            continue;
        }
        if(return_values[4] > 0.90 && abs(return_values[5]) < 0.10  && abs(return_values[3]) < 0.10 && return_values[1] > 0.90){
            if(nsew & 1){
                printf("North already read, please flip to another face.\n");
            } else {
                nsew |= 1;
                north_2 = return_values[4];
                north_1 = return_values[1];
                printf("North read\n");
            }
        } else if(return_values[4] < -0.90 && abs(return_values[5]) < 0.10  && abs(return_values[3]) < 0.10 && return_values[1] < -0.90) {
            if(nsew & 2){
                printf("South already read, please flip to another face.\n");
            } else {
                nsew |= 2;
                south_2 = return_values[4];
                south_1 = return_values[1];
                printf("South read\n");
            }
        } else if(return_values[5] > 0.90 && abs(return_values[4]) < 0.10  && abs(return_values[3]) < 0.10 && return_values[2] < -0.90) {
            if(nsew & 4){
                printf("East already read, please flip to another face.\n");
            } else {
                nsew |= 4;
                east_2 = return_values[5];
                west_1 = return_values[2];
                printf("East read\n");
            }
        } else if(return_values[5] < -0.90 && abs(return_values[4]) < 0.10  && abs(return_values[3]) < 0.10 && return_values[2] > 0.90) {
            if(nsew & 8){
                printf("West already read, please flip to another face.\n");
            } else {
                nsew |= 8;
                west_2 = return_values[5];
                east_1 = return_values[2];
                printf("West read\n");
            }
        } else {
            printf("Not in the right position... %f,%f,%f,%f\n",return_values[1],return_values[2],return_values[4],return_values[5]);
        }
        if(nsew == 15){
            printf("Success!\n");
            printf("MPU:1 North= %f, South= %f, East= %f and West= %f\n", north_1,south_1,east_1,west_1);
            printf("MPU:2 North= %f, South= %f, East= %f and West= %f\n", north_2,south_2,east_2,west_2);

            float nsoffset_1 = -((north_1+south_1)/2.0);
            float nsoffset_2 = -((north_2+south_2)/2.0);

//               float nsoffset = -((north+south)/(north-south));
            float nsscale_1 = 2.0/(north_1-south_1);
            float nsscale_2 = 2.0/(north_2-south_2);

            north_1 += nsoffset_1;
            south_1 += nsoffset_1;
            north_1 *= nsscale_1;
            south_1 *= nsscale_1;

            north_2 += nsoffset_2;
            south_2 += nsoffset_2;
            north_2 *= nsscale_2;
            south_2 *= nsscale_2;
            
            printf("MPU:1 Y offset = %f, Y scale = %f\n",nsoffset_1, nsscale_1);
            printf("MPU:2 Y offset = %f, Y scale = %f\n",nsoffset_2, nsscale_2);

            float ewoffset_1 = -((east_1+west_1)/2.0);
            float ewoffset_2 = -((east_2+west_2)/2.0);
            //                float ewoffset = -((east+west)/(east-west));
            float ewscale_1 = 2.0/(east_1-west_1);
            float ewscale_2 = 2.0/(east_2-west_2);

            east_1 += ewoffset_1;
            west_1 += ewoffset_1;
            east_1 *= ewscale_1;
            west_1 *= ewscale_1;

            east_2 += ewoffset_2;
            west_2 += ewoffset_2;
            east_2 *= ewscale_2;
            west_2 *= ewscale_2;
           
            printf("MPU:1 Z offset = %f, Z scale = %f\n",ewoffset_1, ewscale_1);
            printf("MPU:2 Z offset = %f, Z scale = %f\n",ewoffset_2, ewscale_2);

            printf("MPU:1 North= %f, South= %f, East= %f and West= %f\n", north_1,south_1,east_1,west_1);
            printf("MPU:2 North= %f, South= %f, East= %f and West= %f\n", north_2,south_2,east_2,west_2);

            close(MPU6050_1_fd);
            MPU6050_1_fd = -1;
            close(MPU6050_2_fd);
            MPU6050_1_fd = -1;
            fd_write_out = fopen("/boot/bb_calibrations","w");  // "w"
            if(fd_write_out == NULL) {
                fprintf(stderr, "Could not open file for writing\n");
                return 1;
            } else {
                fprintf(fd_write_out,"YO1:%+07.4f,YS1:%+07.4f,ZO1:%+07.4f,ZS1:%+07.4f,YO2:%+07.4f,YS2:%+07.4f,ZO2:%+07.4f,ZS2:%+07.4f,\n", nsoffset_1,nsscale_1,ewoffset_1,ewscale_1,nsoffset_2,nsscale_2,ewoffset_2,ewscale_2);
                fclose(fd_write_out);
                fd_write_out = NULL;
                return 0;
            }
        }
    }
    if (MPU6050_1_fd != -1) close(MPU6050_1_fd);
    if (MPU6050_2_fd != -1) close(MPU6050_2_fd);
    if (fd_write_out != NULL) fclose(fd_write_out);
    return 0;
}
  
int MPU6050_setup(void){
    if (MPU6050_1_fd != -1) return -1;
	if ((MPU6050_1_fd = open(I2CDEV, O_RDWR)) < 0) {
        MPU6050_1_fd = -1;
        return -1;
    }
  	if (ioctl(MPU6050_1_fd, I2C_SLAVE, MPU6050_1_I2C_ADDRESS) < 0) {
	    close(MPU6050_1_fd);
        MPU6050_1_fd = -1;
        return -1;
	}

    if (MPU6050_2_fd != -1) return -1;
	if ((MPU6050_2_fd = open(I2CDEV, O_RDWR)) < 0) {
        MPU6050_2_fd = -1;
        return -1;
    }
  	if (ioctl(MPU6050_2_fd, I2C_SLAVE, MPU6050_2_I2C_ADDRESS) < 0) {
	    close(MPU6050_2_fd);
        MPU6050_2_fd = -1;
        return -1;
	}

	// reset device
	__u8 rtemp = i2c_smbus_read_byte_data(MPU6050_1_fd, MPU6050_RA_PWR_MGMT_1);
	i2cWriteByteData(MPU6050_1_fd, MPU6050_RA_PWR_MGMT_1, rtemp | 0x80);
	rtemp = i2c_smbus_read_byte_data(MPU6050_2_fd, MPU6050_RA_PWR_MGMT_1);
	i2cWriteByteData(MPU6050_2_fd, MPU6050_RA_PWR_MGMT_1, rtemp | 0x80);
    usleep(500000);

	// take out of sleep
	rtemp = i2c_smbus_read_byte_data(MPU6050_1_fd, MPU6050_RA_PWR_MGMT_1);
	i2cWriteByteData(MPU6050_1_fd, MPU6050_RA_PWR_MGMT_1, rtemp & 0xBF);
	rtemp = i2c_smbus_read_byte_data(MPU6050_2_fd, MPU6050_RA_PWR_MGMT_1);
	i2cWriteByteData(MPU6050_2_fd, MPU6050_RA_PWR_MGMT_1, rtemp & 0xBF);

    // set clock source to xgyro
	rtemp = i2c_smbus_read_byte_data(MPU6050_1_fd, MPU6050_RA_PWR_MGMT_1) & 0xF8;
	i2cWriteByteData(MPU6050_1_fd, MPU6050_RA_PWR_MGMT_1, rtemp | 0x01);
	rtemp = i2c_smbus_read_byte_data(MPU6050_2_fd, MPU6050_RA_PWR_MGMT_1) & 0xF8;
	i2cWriteByteData(MPU6050_2_fd, MPU6050_RA_PWR_MGMT_1, rtemp | 0x01);

    i2cWriteByteData(MPU6050_1_fd,MPU6050_RA_ACCEL_CONFIG, 0x00);  
	i2cWriteByteData(MPU6050_2_fd,MPU6050_RA_ACCEL_CONFIG, 0x00);
    MPU6050_accel_scale_factor = MPU6050_ACCEL_SCALE_MODIFIER_2G;
    
    i2cWriteByteData(MPU6050_1_fd,MPU6050_RA_GYRO_CONFIG, 0x08);  
    i2cWriteByteData(MPU6050_2_fd,MPU6050_RA_GYRO_CONFIG, 0x08);
    MPU6050_gyro_scale_factor = MPU6050_GYRO_SCALE_MODIFIER_500DEG;
    
    // LPF to 188Hz
	i2cWriteByteData(MPU6050_1_fd, MPU6050_RA_CONFIG, 0x01);
	i2cWriteByteData(MPU6050_2_fd, MPU6050_RA_CONFIG, 0x01);
    
    // sample rate to 200 hz
    i2cWriteByteData(MPU6050_1_fd,MPU6050_RA_SMPLRT_DIV, 4);
    i2cWriteByteData(MPU6050_2_fd,MPU6050_RA_SMPLRT_DIV, 4);
    
    i2cWriteByteData(MPU6050_1_fd,MPU6050_RA_FIFO_EN, 0x78);
    i2cWriteByteData(MPU6050_2_fd,MPU6050_RA_FIFO_EN, 0x78);

    // start FIFO
	i2cWriteByteData(MPU6050_1_fd, MPU6050_RA_USER_CTRL, 0x40);
	i2cWriteByteData(MPU6050_2_fd, MPU6050_RA_USER_CTRL, 0x40);
        
    return 0;
}

int MPU6050_get_calibration_values(float* gyro, float *results){
    int i;
    float fifo_returns[6];

    gyro[0] = 0.0;
    gyro[1] = 0.0;
    gyro[2] = 0.0;
    results[0] = 0.0;
    results[1] = 0.0;
    results[2] = 0.0;
    results[3] = 0.0;
    results[4] = 0.0;
    results[5] = 0.0;
    
    usleep(12000);
    while (MPU6050_read_fifo_count(MPU6050_2_fd) != 0) MPU6050_read_fifo_data(MPU6050_2_fd, fifo_returns);
    while (MPU6050_read_fifo_count(MPU6050_1_fd) != 0) MPU6050_read_fifo_data(MPU6050_1_fd, fifo_returns);

    for(i = 0; i < 100; ++i){
        while (MPU6050_read_fifo_count(MPU6050_2_fd) == 0) usleep(1000);
        MPU6050_read_fifo_data(MPU6050_2_fd, fifo_returns);

        gyro[0] += fifo_returns[3];
        gyro[1] += fifo_returns[4];
        gyro[2] += fifo_returns[5];

        results[3] += fifo_returns[0];
        results[4] += fifo_returns[1];
        results[5] += fifo_returns[2];

        if(abs(fifo_returns[3]) > threshold || abs(fifo_returns[4]) > threshold || abs(fifo_returns[5]) > threshold){
            gyro[0] /= (i+1);
            gyro[1] /= (i+1);
            gyro[2] /= (i+1);
            return -1;
        }
        while (MPU6050_read_fifo_count(MPU6050_1_fd) == 0) usleep(1000);
        MPU6050_read_fifo_data(MPU6050_1_fd, fifo_returns);

        results[0] += fifo_returns[0];
        results[1] += fifo_returns[1];
        results[2] += fifo_returns[2];
    }
    gyro[0] /= 100.0;
    gyro[1] /= 100.0;
    gyro[2] /= 100.0;
    results[0] /= 100.0;
    results[1] /= 100.0;
    results[2] /= 100.0;
    results[3] /= 100.0;
    results[4] /= 100.0;
    results[5] /= 100.0;

    return 0;
}


void MPU6050_read_fifo_data(int dfd, float *values){
    __u8 returns[12];
    i2cReadBlockData(dfd, MPU6050_RA_FIFO_R_W, 12, returns);
    values[0]=(float)((returns[0] << 8) + returns[1]); 
    values[1]=(float)((returns[2] << 8) + returns[3]);
    values[2]=(float)((returns[4] << 8) + returns[5]);
    values[3]=(float)((returns[6] << 8) + returns[7]); 
    values[4]=(float)((returns[8] << 8) + returns[9]);
    values[5]=(float)((returns[10] << 8) + returns[11]);
    for(int i=0; i<3; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] /= MPU6050_accel_scale_factor;
    }
   
    for(int i=3; i<6; ++i){
        if(values[i] >= 0x8000) values[i] -= 0x10000;
        values[i] /= MPU6050_gyro_scale_factor;
        if(dfd == MPU6050_2_fd) values[i] -= GYRO_BIAS[i-3];
     }
}

int MPU6050_read_fifo_count(int dfd){
    __s32 high = i2c_smbus_read_byte_data(dfd, MPU6050_RA_FIFO_COUNTH);
    return ((high << 8) + i2c_smbus_read_byte_data(dfd, MPU6050_RA_FIFO_COUNTL)) & 0x07FF;
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

