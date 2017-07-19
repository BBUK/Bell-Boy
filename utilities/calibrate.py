#!/usr/bin/python2
import smbus
import time
from math import atan2
import math
from MPUConstants import MPUConstants as C

DEV_ADDR=0x68

i2c=smbus.SMBus(1)
buffersize=1000     #Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
acel_deadzone=8.0     #Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
giro_deadzone=1.1     #Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

accelgyro = None
mean_ax=None
mean_ay=None
mean_az=None
mean_gx=None
mean_gy=None
mean_gz=None
ax_offset=None
ay_offset=None
az_offset=None
gx_offset=None
gy_offset=None
gz_offset=None

def main():
    calibrate(DEV_ADDR)
    # TODO for two sensors try to align fifo rates by using different
    # MPU6050 clocks

def calibrate(address):
    global accelgyro

    accelgyro = MPU6050(address)

    orig_ax = accelgyro.read_word_signed(C.MPU6050_RA_XA_OFFS_H)
    orig_ay = accelgyro.read_word_signed(C.MPU6050_RA_YA_OFFS_H)
    orig_az = accelgyro.read_word_signed(C.MPU6050_RA_ZA_OFFS_H)
    orig_gx = accelgyro.read_word_signed(C.MPU6050_RA_XG_OFFS_USRH)
    orig_gy = accelgyro.read_word_signed(C.MPU6050_RA_YG_OFFS_USRH)
    orig_gz = accelgyro.read_word_signed(C.MPU6050_RA_ZG_OFFS_USRH)

    accelgyro.set_x_accel_offset(0)
    accelgyro.set_y_accel_offset(0)
    accelgyro.set_z_accel_offset(0)
    accelgyro.set_x_gyro_offset(0)
    accelgyro.set_y_gyro_offset(0)
    accelgyro.set_z_gyro_offset(0)
    
    state = 0
    while True:	
        if (state==0):
            print "\nReading sensors for first time..."
            meansensors()
            state += 1
            time.sleep(1)
        if (state==1):
            print "\nCalculating offsets..."
            calibration();
            state += 1
            time.sleep(1)
        if (state==2):
            meansensors()
            print "\nFINISHED!"
            print "\nSensor readings with offsets:\t"
            print "Readings: AX:{0},AY:{1},AZ:{2},GX:{3},GY:{4},GZ:{5}".format(mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz)
            print "Offsets: AX:{0},AY:{1},AZ:{2},GX:{3},GY:{4},GZ:{5}".format(ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset)
            print "Factory:  AX:{0},AY:{1},AZ:{2},GX:{3},GY:{4},GZ:{5}".format(orig_ax,orig_ay,orig_az,orig_gx,orig_gy,orig_gz)
            break

def meansensors():
	global mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz
	i=0
	buff_ax=0
	buff_ay=0
	buff_az=0
	buff_gx=0
	buff_gy=0
	buff_gz=0

	while (i<(buffersize+101)):
    # read raw accel/gyro measurements from device
		accel_reading = accelgyro.get_acceleration()
		ax = accel_reading[0]
		ay = accel_reading[1]
		az = accel_reading[2]

		gyro_reading = accelgyro.get_rotation()
		gx = gyro_reading[0]
		gy = gyro_reading[1]
		gz = gyro_reading[2]
    
		if (i>100 and i<=(buffersize+100)): #First 100 measures are discarded
			buff_ax=buff_ax+ax
			buff_ay=buff_ay+ay
			buff_az=buff_az+az
			buff_gx=buff_gx+gx
			buff_gy=buff_gy+gy
			buff_gz=buff_gz+gz
 
		if (i==(buffersize+100)):
			mean_ax=int(buff_ax/buffersize)
			mean_ay=int(buff_ay/buffersize)
			mean_az=int(buff_az/buffersize)
			mean_gx=int(buff_gx/buffersize)
			mean_gy=int(buff_gy/buffersize)
			mean_gz=int(buff_gz/buffersize)

		i += 1
		time.sleep(0.002)

def calibration():
	global ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset
	global mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz
	ax_offset=-mean_ax/8.0
	ay_offset=-mean_ay/8.0
	az_offset=(16384.0-mean_az)/8.0
	gx_offset=-mean_gx/4.0
	gy_offset=-mean_gy/4.0
	gz_offset=-mean_gz/4.0

	while True:
		ready=0
		accelgyro.set_x_accel_offset(int(ax_offset + 0.5))
		accelgyro.set_y_accel_offset(int(ay_offset  + 0.5))
		accelgyro.set_z_accel_offset(int(az_offset + 0.5))
		accelgyro.set_x_gyro_offset(int(gx_offset + 0.5))
		accelgyro.set_y_gyro_offset(int(gy_offset + 0.5))
		accelgyro.set_z_gyro_offset(int(gz_offset + 0.5))
		meansensors()
		if (abs(mean_ax)<=acel_deadzone):
			ready += 1
		else:
			ax_offset=ax_offset-mean_ax/acel_deadzone

		if (abs(mean_ay)<=acel_deadzone):
			ready += 1
		else:
			ay_offset=ay_offset-mean_ay/acel_deadzone

		if (abs(16384.0-mean_az)<=acel_deadzone):
			ready += 1
		else: 
			az_offset=az_offset+(16384.0-mean_az)/acel_deadzone

		if (abs(mean_gx)<=giro_deadzone):
			ready += 1
		else:
			gx_offset=gx_offset-mean_gx/(giro_deadzone+1)

		if (abs(mean_gy)<=giro_deadzone):
			ready += 1
		else:
			gy_offset=gy_offset-mean_gy/(giro_deadzone+1)

		if (abs(mean_gz)<=giro_deadzone):
			ready += 1
		else:
			gz_offset=gz_offset-mean_gz/(giro_deadzone+1)

		print "... %d, AX=%d, AY=%d, AZ=%d, GX=%d, GY=%d, GZ=%d, " %(ready, ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset)

		if (ready == 6):
			break

class MPU6050:
    def __init__(self,address):
        self.dev_addr = address
        self.accn_scale = None
        self.gyro_scale = None
        self.FIFO_block_length = 12

    # reset device
        self.write_bit(C.MPU6050_RA_PWR_MGMT_1,C.MPU6050_PWR1_DEVICE_RESET_BIT, 1)
        sleep(0.5)
    #wakeup
    #possibly some undocumented behaviour here - gyro config register does
    #not accept write of FS factor unless device woken up first
        self.write_bit(C.MPU6050_RA_PWR_MGMT_1, C.MPU6050_PWR1_SLEEP_BIT, 0)
        
    #set clock source
    #because we have two accelerometers with slightly different clocks (and no way of syncing
    #them with the GY-521 breakouts we are using), we have to select which pll gives the closest
    #sync on a case by case basis.  Should only be a one time calibration.  The FIFO read code
    #above takes account of a lack of sync by ditching fifo packets from the faster running device
    #but wise to keep things as far in sync as we can.
        t1 = i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_PWR_MGMT_1) & 0xFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_PWR_MGMT_1, (t1 & 0xF8) | C.MPU6050_CLOCK_PLL_ZGYRO)

    # set full scale acceleration range
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ACCEL_CONFIG, C.MPU6050_ACCEL_FS_2 << 3)
        self.accn_scale = C.MPU6050_ACCEL_SCALE_MODIFIER_2G

    # set full scale gyro range
        i2c.write_byte_data(self.dev_addr,C.MPU6050_RA_GYRO_CONFIG, C.MPU6050_GYRO_FS_250 << 3)
        self.gyro_scale = C.MPU6050_GYRO_SCALE_MODIFIER_250DEG
                
    def who_am_i(self):
        return i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_WHO_AM_I) & 0xFF
 
    def write_bit(self,a_reg_add, a_bit_num, a_bit):
        byte = i2c.read_byte_data(self.dev_addr, a_reg_add)
        if a_bit:
            byte |= 1 << a_bit_num
        else:
            byte &= ~(1 << a_bit_num)
        i2c.write_byte_data(self.dev_addr, a_reg_add, byte)

    def set_x_accel_offset(self,a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XA_OFFS_H,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XA_OFFS_L_TC,(a_offset & 0xFF))

    def set_y_accel_offset(self,a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YA_OFFS_H,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YA_OFFS_L_TC,(a_offset & 0xFF))

    def set_z_accel_offset(self,a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZA_OFFS_H,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZA_OFFS_L_TC,(a_offset & 0xFF))

    def set_x_gyro_offset(self, a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XG_OFFS_USRH,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_XG_OFFS_USRL,(a_offset & 0xFF))

    def set_y_gyro_offset(self, a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YG_OFFS_USRH,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_YG_OFFS_USRL,(a_offset & 0xFF))

    def set_z_gyro_offset(self, a_offset):
        a_offset = a_offset & 0xFFFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZG_OFFS_USRH,(a_offset >> 8))
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_ZG_OFFS_USRL,(a_offset & 0xFF))

    def read_word_signed(self, register):
        high = i2c.read_byte_data(self.dev_addr, register)
        res = (high << 8) + i2c.read_byte_data(self.dev_addr, register + 1)
        if (res >= 0x8000):
            return res - 0x10000
        return res

    def write_word_signed(self,register,value):
        value = value & 0xFFFF
        i2c.write_byte_data(self.dev_addr, register,(value >> 8))
        i2c.write_byte_data(self.dev_addr, register + 1,(value & 0xFF))
        
    def get_acceleration(self):
        result = []
        readbytes = i2c.read_i2c_block_data(self.dev_addr, C.MPU6050_RA_ACCEL_XOUT_H, 6)
        result.append(readbytes[0] << 8 | readbytes[1])
        result.append(readbytes[2] << 8 | readbytes[3])
        result.append(readbytes[4] << 8 | readbytes[5])
        for index in range(3):
            if (result[index] >= 0x8000):
                result[index] -= 0x10000
        return result

    def get_rotation(self):
        result = []
        readbytes = i2c.read_i2c_block_data(self.dev_addr, C.MPU6050_RA_GYRO_XOUT_H, 6)
        result.append(readbytes[0] << 8 | readbytes[1])
        result.append(readbytes[2] << 8 | readbytes[3])
        result.append(readbytes[4] << 8 | readbytes[5])
        for index in range(3):
            if (result[index] >= 0x8000):
                result[index] -= 0x10000
        return result
        
    def reset_FIFO(self):
        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_EN_BIT, 0)
        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_RESET_BIT, 1)
        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_EN_BIT, 1)

    def get_int_status(self):
        return i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_INT_STATUS)
        
    def get_FIFO_count(self):
        high = i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_COUNTH)
        return (high << 8) + i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_COUNTL)

    def read_FIFO_block(self):
        result = [] # accn x,y,z then gyro x,y,z
        readbytes=[]
        for index in range(self.FIFO_block_length):
            readbytes.append(i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_R_W))
        for index in range(0, self.FIFO_block_length,2):
            result.append((readbytes[index] << 8) + readbytes[index+1])
            if (result[len(result)-1] >= 0x8000):
                result[len(result)-1] -= 0x10000
        result[0] /= float(self.accn_scale) # results in g
        result[1] /= float(self.accn_scale)
        result[2] /= float(self.accn_scale)
        result[3] /= float(self.gyro_scale) # result degrees/sec
        result[4] /= float(self.gyro_scale)
        result[5] /= float(self.gyro_scale)
        return result
        
    def enable_FIFO(self):
        # set LPF to 188Hz
        
        t1 = i2c.read_byte_data(self.dev_addr, C.MPU6050_RA_CONFIG) & 0xFF
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_CONFIG, (t1 & 0xF8) | C.MPU6050_DLPF_BW_188) # 188 Hz LPF

        #set gyroscope output rate divider
        # * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
        # * Motion detection, and Free Fall detection are all based on the Sample Rate.
        # * The Sample Rate is generated by dividing the gyroscope output rate by
        # * SMPLRT_DIV:
        # *
        # * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        # *
        # * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
        # * 7), and 1kHz when the DLPF is enabled (see Register 26).
        # *
        # * Note: The accelerometer output rate is 1kHz. This means that for a Sample
        # * Rate greater than 1kHz, the same accelerometer sample may be output to the
        # * FIFO, DMP, and sensor registers more than once.
        # *
        # * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
        # * of the MPU-6000/MPU-6050 Product Specification document

        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_SMPLRT_DIV, 19) # takes us to 50hz
        
        # enable fifo, select accel and gyro to go in
        i2c.write_byte_data(self.dev_addr, C.MPU6050_RA_FIFO_EN, 0x78) # enable temp = 0xF8

        self.write_bit(C.MPU6050_RA_USER_CTRL,C.MPU6050_USERCTRL_FIFO_EN_BIT, 1)   

if __name__ == "__main__":
    main()
