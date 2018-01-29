// https://www.raspberrypi.org/forums/viewtopic.php?f=33&t=10479
// g++ i2ctest.cpp -lwiringPi -o i2ctest
// http://wiringpi.com/reference/i2c-library/
// https://raspberrypi.stackexchange.com/questions/3627/is-there-an-i2c-library
// https://github.com/abrugsch/GBCartSlurp/blob/master/I2CBus.cpp

// this
// https://github.com/leon-anavi/rpi-examples/tree/master/BMP180/c

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include "fifotimer.h"
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#ifndef NULL
#define NULL 0
#endif

#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif

#define GYRO_I2C_ADDRESS 0x21
#define GYRO_REGISTER_OUT_X_MSB 0x01
#define GYRO_REGISTER_F_STATUS 0x08
#define GYRO_REGISTER_F_SETUP 0x09
#define GYRO_REGISTER_CTRL_REG0 0x0D
#define GYRO_REGISTER_CTRL_REG1 0x13

int fd;

int main(int argc, char **argv){
  struct timeval start, stop;
  int elapsed = 0, count = 0, cco = 0;
  float averaged = 0.0;
  int overflowFlag = 0;
  setup();
  i2cWriteByteData(fd,GYRO_REGISTER_CTRL_REG1, 0x00);  // device to standby
  i2cWriteByteData(fd,GYRO_REGISTER_CTRL_REG0, 0x02);  // HPF=disabled LPF=16Hz FSbandwidth=500deg/sec
  i2cWriteByteData(fd,GYRO_REGISTER_F_SETUP, 0x00);  // disable fifo
  i2cWriteByteData(fd,GYRO_REGISTER_F_SETUP, 0x80);  // enable fifo
  i2cWriteByteData(fd,GYRO_REGISTER_CTRL_REG1, 0x06);  // 400hz ODR and active
  usleep(100000);

  while(1){
    count = 0;
    elapsed = 0;
    while(count < 10){
      while (read_fifo_count() != 0) read_gyro_data();
      while (read_fifo_count() == 0);
      gettimeofday(&start, NULL);
      read_gyro_data();
      while (read_fifo_count() < 26) usleep(2000);
      while(1){
        cco = read_fifo_count();
        if(cco == 30) break;
        if(cco > 30) {
          printf("Overflow: %d ... ",cco);
          overflowFlag = 1;
          break;
        }
      }
      gettimeofday(&stop, NULL);
      elapsed +=  (int)((stop.tv_sec-start.tv_sec)*1000000ULL+(stop.tv_usec-start.tv_usec));
      count += 1;

    }
    if(!overflowFlag) {
      if(averaged == 0.0) averaged=elapsed/300000.0;
      averaged = 0.95*averaged + 0.05*elapsed/300000.0;
    } else {
      overflowFlag=0;
    }
    printf("Elapsed %.5f Average %5f\n", elapsed/300000.0,averaged);
  }
  return 0;
}

int read_fifo_count(){
  return i2c_smbus_read_byte_data(fd, GYRO_REGISTER_F_STATUS) & 0x3F;
}

void read_gyro_data(){
  __u8 values[6];
  i2cReadBlockData(fd, GYRO_REGISTER_OUT_X_MSB, 6, values);
}
  
void setup(){
	char *fileName = "/dev/i2c-1";
	if ((fd = open(fileName, O_RDWR)) < 0) {
      printf("Failed to open I2C file\n");
      exit(1);
    }
	if (ioctl(fd, I2C_SLAVE, GYRO_I2C_ADDRESS) < 0) {
	  close(fd);
	  printf("Failed to open ioctl\n");
      exit(1);
	}
}

// Read two words
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
