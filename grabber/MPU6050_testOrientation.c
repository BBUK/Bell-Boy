//gcc MPU6050_testOrientation.c -o MPU6050_testOrientation -lm
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

#define RADIANS_TO_DEGREES_MULTIPLIER 57.29578 
#define I2CDEV "/dev/i2c-1"

int MPU6050_1_fd = -1;
int MPU6050_2_fd = -1;

float MPU6050_gyro_scale_factor = 0;
float MPU6050_accel_scale_factor = 0;

float GYRO_BIAS[3] = { 0.0, 0.0, 0.0 };

float yoffset_1 = 0.0;
float yscale_1 = 1.0;
float zoffset_1 = 0.0;
float zscale_1 = 1.0;

float yoffset_2 = 0.0;
float yscale_2 = 1.0;
float zoffset_2 = 0.0;
float zscale_2 = 1.0;

volatile sig_atomic_t sig_exit = 0;
void sig_handler(int signum) {
    if (signum == SIGINT) fprintf(stderr, "received SIGINT\n");
    if (signum == SIGTERM) fprintf(stderr, "received SIGTERM\n");
    sig_exit = 1;
}

int main(int argc, char const *argv[]){
    float gyro_values[3];
    float return_values[6];
    
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(struct sigaction));
    sig_action.sa_handler = sig_handler;
    sigaction(SIGTERM, &sig_action, NULL);
    sigaction(SIGINT, &sig_action, NULL);

    FILE *fd_read_cal;
    fd_read_cal = fopen("/boot/bb_calibrations","r");
    if(fd_read_cal == NULL) {
        printf("Calibration file not found.\n");
    } else {
        if (fgets(READ_OUTBUF_LINE, sizeof(READ_OUTBUF_LINE), fd_read_cal) != NULL) {
            char *p = strtok(READ_OUTBUF_LINE,",");
            if(p!= NULL) yoffset_1 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) yscale_1 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) zoffset_1 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) zscale_1 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) yoffset_2 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) yscale_2 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) zoffset_2 = atof(&p[4]);
            p = strtok(NULL,",");
            if(p!= NULL) zscale_2 = atof(&p[4]);
        }
        fclose(fd_read_cal);
    }
        
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
        printf("X1:%+07.4f,Y1:%+07.4f,Z1:%+07.4f,X2:%+07.4f,Y2:%+07.4f,Z2:%+07.4f,R1:%+07.4f,R2:%+07.4f,\n", return_values[0],return_values[1],return_values[2],return_values[3],return_values[4],return_values[0],atan2(return_values[1], return_values[2]) * RADIANS_TO_DEGREES_MULTIPLIER,atan2(return_values[4], return_values[5]) * RADIANS_TO_DEGREES_MULTIPLIER);
        break;
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
    if(dfd == MPU6050_1_fd){
        values[1] += yoffset_1;
        values[1] *= yscale_1;
        values[2] += zoffset_1;
        values[2] *= zscale_1;
    } else {
        values[1] += yoffset_2;
        values[1] *= yscale_2;
        values[2] += zoffset_2;
        values[2] *= zscale_2;
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

