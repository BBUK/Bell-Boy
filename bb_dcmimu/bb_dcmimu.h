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

#include <linux/types.h>
#include <linux/i2c.h>

void i2cReadBlockData(int fd, __u8 address, __u8 length, __u8 *values);
void i2cWriteByteData(int fd, __u8 address, __u8 value);
void setup(void);
void read_gyro_data(void);
int read_fifo_count(void);
void calculate(float u0, float u1, float u2, float z0, float z1,float z2, float h);
int NXP_start_fifos(int ODR, int gyro_fs, int accel_fs);
void NXP_stop_fifos(void);
void NXP_pull_data(void);
void NXP_read_gyro_data(float *values);
void NXP_read_accel_data(float *values);
void NXP_clear_fifos(void);
int NXP_read_accel_fifo_count(void);
int NXP_read_gyro_fifo_count(void);
int NXP_fifo_timer(void);
int NXP_test(void);
float NXP_get_orientation(void);

#define NXP_GYRO_I2C_ADDRESS             0x21
#define NXP_GYRO_ID                  0xD7
#define NXP_GYRO_SENSITIVITY_250DPS  0.0078125
#define NXP_GYRO_SENSITIVITY_500DPS  0.015625  
#define NXP_GYRO_SENSITIVITY_1000DPS 0.03125   
#define NXP_GYRO_REGISTER_STATUS              0x00
#define NXP_GYRO_REGISTER_OUT_X_MSB           0x01
#define NXP_GYRO_REGISTER_OUT_X_LSB           0x02
#define NXP_GYRO_REGISTER_OUT_Y_MSB           0x03
#define NXP_GYRO_REGISTER_OUT_Y_LSB           0x04
#define NXP_GYRO_REGISTER_OUT_Z_MSB           0x05
#define NXP_GYRO_REGISTER_OUT_Z_LSB           0x06
#define NXP_GYRO_REGISTER_DR_STATUS           0x07
#define NXP_GYRO_REGISTER_F_STATUS           0x08
#define NXP_GYRO_REGISTER_F_SETUP           0x09
#define NXP_GYRO_REGISTER_F_EVENT           0x0A
#define NXP_GYRO_REGISTER_WHO_AM_I            0x0C   
#define NXP_GYRO_REGISTER_CTRL_REG0           0x0D   
#define NXP_GYRO_REGISTER_CTRL_REG1           0x13   
#define NXP_GYRO_REGISTER_CTRL_REG2           0x14   


#define NXP_ACCEL_I2C_ADDRESS 0x1F
#define NXP_ACCEL_ID 0xC7
#define NXP_ACCEL_SENSITIVITY_2G 0.000244
#define NXP_ACCEL_SENSITIVITY_4G 0.000488
#define NXP_ACCEL_SENSITIVITY_8G 0.000976
#define NXP_ACCEL_REGISTER_STATUS          0x00
#define NXP_ACCEL_REGISTER_OUT_X_MSB       0x01
#define NXP_ACCEL_REGISTER_OUT_X_LSB       0x02
#define NXP_ACCEL_REGISTER_OUT_Y_MSB       0x03
#define NXP_ACCEL_REGISTER_OUT_Y_LSB       0x04
#define NXP_ACCEL_REGISTER_OUT_Z_MSB       0x05
#define NXP_ACCEL_REGISTER_OUT_Z_LSB       0x06
#define NXP_ACCEL_REGISTER_F_SETUP         0x09
#define NXP_ACCEL_REGISTER_WHO_AM_I        0x0D
#define NXP_ACCEL_REGISTER_XYZ_DATA_CFG    0x0E
#define NXP_ACCEL_REGISTER_CTRL_REG1       0x2A
#define NXP_ACCEL_REGISTER_CTRL_REG2       0x2B
#define NXP_ACCEL_REGISTER_CTRL_REG3       0x2C
#define NXP_ACCEL_REGISTER_CTRL_REG4       0x2D
#define NXP_ACCEL_REGISTER_CTRL_REG5       0x2E
#define NXP_ACCEL_REGISTER_MSTATUS         0x32
#define NXP_ACCEL_REGISTER_MOUT_X_MSB      0x33
#define NXP_ACCEL_REGISTER_MOUT_X_LSB      0x34
#define NXP_ACCEL_REGISTER_MOUT_Y_MSB      0x35
#define NXP_ACCEL_REGISTER_MOUT_Y_LSB      0x36
#define NXP_ACCEL_REGISTER_MOUT_Z_MSB      0x37
#define NXP_ACCEL_REGISTER_MOUT_Z_LSB      0x38
#define NXP_ACCEL_REGISTER_MCTRL_REG1      0x5B
#define NXP_ACCEL_REGISTER_MCTRL_REG2      0x5C
#define NXP_ACCEL_REGISTER_MCTRL_REG3      0x5D

#define MPU6050_RA_XG_OFFS_TC 0x00
#define MPU6050_RA_YG_OFFS_TC 0x01
#define MPU6050_RA_ZG_OFFS_TC 0x02
#define MPU6050_RA_X_FINE_GAIN 0x03
#define MPU6050_RA_Y_FINE_GAIN 0x04
#define MPU6050_RA_Z_FINE_GAIN 0x05
#define MPU6050_RA_XA_OFFS_H 0x06
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B
#define MPU6050_RA_XG_OFFS_USRH 0x13
#define MPU6050_RA_XG_OFFS_USRL 0x14
#define MPU6050_RA_YG_OFFS_USRH 0x15
#define MPU6050_RA_YG_OFFS_USRL 0x16
#define MPU6050_RA_ZG_OFFS_USRH 0x17
#define MPU6050_RA_ZG_OFFS_USRL 0x18
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_FIFO_EN 0x23
#define MPU6050_RA_INT_STATUS 0x3A
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68
#define MPU6050_RA_USER_CTRL 0x6A
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_COUNTL 0x73
#define MPU6050_RA_FIFO_R_W 0x74
#define MPU6050_RA_WHO_AM_I 0x75

#define MPU6050_TC_PWR_MODE_BIT 7
#define MPU6050_TC_OFFSET_BIT 6
#define MPU6050_TC_OFFSET_LENGTH 6

#define MPU6050_VDDIO_LEVEL_VLOGIC 0
#define MPU6050_VDDIO_LEVEL_VDD 1

#define MPU6050_DLPF_BW_256 0x00
#define MPU6050_DLPF_BW_188 0x01
#define MPU6050_DLPF_BW_98 0x02
#define MPU6050_DLPF_BW_42 0x03
#define MPU6050_DLPF_BW_20 0x04
#define MPU6050_DLPF_BW_10 0x05
#define MPU6050_DLPF_BW_5 0x06

#define MPU6050_GCONFIG_FS_SEL_BIT 4
#define MPU6050_GCONFIG_FS_SEL_LENGTH 2

#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

#define MPU6050_ACONFIG_XA_ST_BIT 7
#define MPU6050_ACONFIG_YA_ST_BIT 6
#define MPU6050_ACONFIG_ZA_ST_BIT 5
#define MPU6050_ACONFIG_AFS_SEL_BIT 4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH 2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT 2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH 3

#define MPU6050_ACCEL_SCALE_MODIFIER_2G 0x4000
#define MPU6050_ACCEL_SCALE_MODIFIER_4G 0x2000
#define MPU6050_ACCEL_SCALE_MODIFIER_8G 0x1000
#define MPU6050_ACCEL_SCALE_MODIFIER_16G 0x0800

#define MPU6050_GYRO_SCALE_MODIFIER_250DEG 131.0 
#define MPU6050_GYRO_SCALE_MODIFIER_500DEG 65.5 
#define MPU6050_GYRO_SCALE_MODIFIER_1000DEG 32.8
#define MPU6050_GYRO_SCALE_MODIFIER_2000DEG 16.4

#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_ACCEL_FS_8 0x02
#define MPU6050_ACCEL_FS_16 0x03

#define MPU6050_DHPF_RESET 0x00
#define MPU6050_DHPF_5 0x01
#define MPU6050_DHPF_2P5 0x02
#define MPU6050_DHPF_1P25 0x03
#define MPU6050_DHPF_0P63 0x04
#define MPU6050_DHPF_HOLD 0x07

#define MPU6050_TEMP_FIFO_EN_BIT 7
#define MPU6050_XG_FIFO_EN_BIT 6
#define MPU6050_YG_FIFO_EN_BIT 5
#define MPU6050_ZG_FIFO_EN_BIT 4
#define MPU6050_ACCEL_FIFO_EN_BIT 3

#define MPU6050_CLOCK_DIV_348 0x0
#define MPU6050_CLOCK_DIV_333 0x1
#define MPU6050_CLOCK_DIV_320 0x2
#define MPU6050_CLOCK_DIV_308 0x3
#define MPU6050_CLOCK_DIV_296 0x4
#define MPU6050_CLOCK_DIV_286 0x5
#define MPU6050_CLOCK_DIV_276 0x6
#define MPU6050_CLOCK_DIV_267 0x7
#define MPU6050_CLOCK_DIV_258 0x8
#define MPU6050_CLOCK_DIV_500 0x9
#define MPU6050_CLOCK_DIV_471 0xA
#define MPU6050_CLOCK_DIV_444 0xB
#define MPU6050_CLOCK_DIV_421 0xC
#define MPU6050_CLOCK_DIV_400 0xD
#define MPU6050_CLOCK_DIV_381 0xE
#define MPU6050_CLOCK_DIV_364 0xF

#define MPU6050_INTCFG_INT_LEVEL_BIT 7
#define MPU6050_INTCFG_INT_OPEN_BIT 6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT 5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT 4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT 3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT 2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT 1
#define MPU6050_INTCFG_CLKOUT_EN_BIT 0

#define MPU6050_INTMODE_ACTIVEHIGH 0x00
#define MPU6050_INTMODE_ACTIVELOW 0x01

#define MPU6050_INTDRV_PUSHPULL 0x00
#define MPU6050_INTDRV_OPENDRAIN 0x01

#define MPU6050_INTLATCH_50USPULSE 0x00
#define MPU6050_INTLATCH_WAITCLEAR 0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD 0x01

#define MPU6050_INTERRUPT_FF_BIT 7
#define MPU6050_INTERRUPT_MOT_BIT 6
#define MPU6050_INTERRUPT_ZMOT_BIT 5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT 4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT 3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT 2
#define MPU6050_INTERRUPT_DMP_INT_BIT 1
#define MPU6050_INTERRUPT_DATA_RDY_BIT 0

#define MPU6050_USERCTRL_DMP_EN_BIT 7
#define MPU6050_USERCTRL_FIFO_EN_BIT 6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT 5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT 4
#define MPU6050_USERCTRL_DMP_RESET_BIT 3
#define MPU6050_USERCTRL_FIFO_RESET_BIT 2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT 1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT 0

#define MPU6050_PWR1_DEVICE_RESET_BIT 7
#define MPU6050_PWR1_SLEEP_BIT 6
#define MPU6050_PWR1_CYCLE_BIT 5
#define MPU6050_PWR1_TEMP_DIS_BIT 3

#define MPU6050_CLOCK_INTERNAL 0x00
#define MPU6050_CLOCK_PLL_XGYRO 0x01
#define MPU6050_CLOCK_PLL_YGYRO 0x02
#define MPU6050_CLOCK_PLL_ZGYRO 0x03
#define MPU6050_CLOCK_PLL_EXT32K 0x04
#define MPU6050_CLOCK_PLL_EXT19M 0x05
#define MPU6050_CLOCK_KEEP_RESET 0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT 7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH 2
#define MPU6050_PWR2_STBY_XA_BIT 5
#define MPU6050_PWR2_STBY_YA_BIT 4
#define MPU6050_PWR2_STBY_ZA_BIT 3
#define MPU6050_PWR2_STBY_XG_BIT 2
#define MPU6050_PWR2_STBY_YG_BIT 1
#define MPU6050_PWR2_STBY_ZG_BIT 0

#define MPU6050_WAKE_FREQ_1P25 0x0
#define MPU6050_WAKE_FREQ_2P5 0x1
#define MPU6050_WAKE_FREQ_5 0x2
#define MPU6050_WAKE_FREQ_10 0x3

#define MPU6050_WHO_AM_I_BIT 6
#define MPU6050_WHO_AM_I_LENGTH 6


#ifndef LIB_I2C_SMBUS_H
#define LIB_I2C_SMBUS_H

extern __s32 i2c_smbus_access(int file, char read_write, __u8 command,
			      int size, union i2c_smbus_data *data);

extern __s32 i2c_smbus_write_quick(int file, __u8 value);
extern __s32 i2c_smbus_read_byte(int file);
extern __s32 i2c_smbus_write_byte(int file, __u8 value);
extern __s32 i2c_smbus_read_byte_data(int file, __u8 command);
extern __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);
extern __s32 i2c_smbus_read_word_data(int file, __u8 command);
extern __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);
extern __s32 i2c_smbus_process_call(int file, __u8 command, __u16 value);

/* Returns the number of read bytes */
extern __s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values);
extern __s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length,
					const __u8 *values);

/* Returns the number of read bytes */
/* Until kernel 2.6.22, the length is hardcoded to 32 bytes. If you
   ask for less than 32 bytes, your code will only work with kernels
   2.6.23 and later. */
extern __s32 i2c_smbus_read_i2c_block_data(int file, __u8 command, __u8 length,
					   __u8 *values);
extern __s32 i2c_smbus_write_i2c_block_data(int file, __u8 command, __u8 length,
					    const __u8 *values);

/* Returns the number of read bytes */
extern __s32 i2c_smbus_block_process_call(int file, __u8 command, __u8 length,
					  __u8 *values);

#endif /* LIB_I2C_SMBUS_H */
