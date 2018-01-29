#!/usr/bin/python2
import time
import gc
import smbus
i2c=smbus.SMBus(1)
from MPUConstants import NXPGyroConstants as G
from MPUConstants import NXPAccelConstants as A


class NXP:
    def __init__(self): # this does *not* activate the device.  reset_FIFO does that job.

# reset gyro
#        try: # no ack sent following reset
#            i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_CTRL_REG1, 0x00)  # device to standby
#            i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_CTRL_REG1, 0x40)  # reset
#        except:
#            pass
#        sleep(0.1)
# accelerometer
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_CTRL_REG1, 0x00)  # device to standby
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_XYZ_DATA_CFG, 0x00)  # 2G FS ACCEL
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_CTRL_REG2, 0x02)  # High resolution
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_MCTRL_REG1, 0x00) # disable magnetometer
        
# gyro
        i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_CTRL_REG1, 0x00)  # device to standby
        i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_CTRL_REG0, 0x02)  # HPF=disabled LPF=16Hz FSbandwidth=500deg/sec 
                
    def who_am_i_accel(self):
        return i2c.read_byte_data(A.ACCEL_ADDRESS, A.ACCEL_REGISTER_WHO_AM_I) & 0xFF

    def who_am_i_gyro(self):
        return i2c.read_byte_data(G.GYRO_ADDRESS, G.GYRO_REGISTER_WHO_AM_I) & 0xFF
 
    def write_bit(self,a_reg_add, a_bit_num, a_bit):
        byte = i2c.read_byte_data(self.dev_addr, a_reg_add)
        if a_bit:
            byte |= 1 << a_bit_num
        else:
            byte &= ~(1 << a_bit_num)
        i2c.write_byte_data(self.dev_addr, a_reg_add, byte)

    def read_word_signed(self, dev_addr, register):
        high = i2c.read_byte_data(dev_addr, register)
        res = (high << 8) + i2c.read_byte_data(dev_addr, register + 1)
        if (res >= 0x8000):
            return res - 0x10000
        return res

    def write_word_signed(self,dev_addr, register,value):
        value = value & 0xFFFF
        i2c.write_byte_data(dev_addr, register,(value >> 8))
        i2c.write_byte_data(dev_addr, register + 1,(value & 0xFF))
        
    def get_acceleration(self):
        result = []
        readbytes = i2c.read_i2c_block_data(A.ACCEL_ADDRESS, A.ACCEL_REGISTER_OUT_X_MSB, 6)
        result.append(readbytes[0] << 8 | readbytes[1])
        result.append(readbytes[2] << 8 | readbytes[3])
        result.append(readbytes[4] << 8 | readbytes[5])
        for index in range(3):
            if (result[index] >= 0x8000):
                result[index] -= 0x10000
            result[index] /= 4 # left shift for 14 bit data
            result[index] /= 4096.0 # convert to g
        return result

    def get_rotation(self):
        result = []
        readbytes = i2c.read_i2c_block_data(G.GYRO_ADDRESS, G.GYRO_REGISTER_OUT_X_MSB, 6)
        result.append(readbytes[0] << 8 | readbytes[1])
        result.append(readbytes[2] << 8 | readbytes[3])
        result.append(readbytes[4] << 8 | readbytes[5])
        for index in range(3):
            if (result[index] >= 0x8000):
                result[index] -= 0x10000
            result[index] *= G.GYRO_SENSITIVITY_500DPS
        return result
        
    def reset_FIFO(self):
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_CTRL_REG1, 0x24)  # 50hz ODR and standby
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_F_SETUP, 0x00)  # disable fifo
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_F_SETUP, 0x80)  # reenable
        i2c.write_byte_data(A.ACCEL_ADDRESS,A.ACCEL_REGISTER_CTRL_REG1, 0x25)  # 50hz ODR and out of standby

        i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_CTRL_REG1, 0x10)  # 50hz ODR and standby
        i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_F_SETUP, 0x00)  # disable fifo
        i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_F_SETUP, 0x80)  # reenable
        i2c.write_byte_data(G.GYRO_ADDRESS,G.GYRO_REGISTER_CTRL_REG1, 0x12)  # 50hz ODR and active
        time.sleep(0.1)
        
    def get_FIFO_overflow(self):
		if (i2c.read_byte_data(A.ACCEL_ADDRESS, A.ACCEL_REGISTER_STATUS) & 0x80) or \
		    (i2c.read_byte_data(G.GYRO_ADDRESS, G.GYRO_REGISTER_F_STATUS) & 0x80):
		    return True
		else:
			return False

    def get_accel_FIFO_count(self):
        return i2c.read_byte_data(A.ACCEL_ADDRESS, A.ACCEL_REGISTER_STATUS) & 0x3F

    def get_gyro_FIFO_count(self):
        return i2c.read_byte_data(G.GYRO_ADDRESS, G.GYRO_REGISTER_F_STATUS) & 0x3F
      
imu=NXP()
imu.reset_FIFO()
count=0
aggregate = 0.0
current_time=time.time()
elapsed_time = 0.0
while count < 100:
    while imu.get_gyro_FIFO_count() == 0:
        pass
    a = time.time()
    elapsed_time=a - current_time
    current_time=a
    count += 1
    imu.get_rotation()
    print 1.0/elapsed_time
    aggregate += elapsed_time

print 1/(aggregate/100.0)
 

