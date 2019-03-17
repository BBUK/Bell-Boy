#define ICM20689_SELF_TEST_X_GYRO    (0x00)
#define ICM20689_SELF_TEST_Y_GYRO    (0x01)
#define ICM20689_SELF_TEST_Z_GYRO    (0x02)
#define ICM20689_SELF_TEST_X_ACCEL   (0x0D)
#define ICM20689_SELF_TEST_Y_ACCEL   (0x0E)
#define ICM20689_SELF_TEST_Z_ACCEL   (0x0F)

#define ICM20689_XG_OFFS_USRH        (0x13)
#define ICM20689_XG_OFFS_USRL        (0x14)
#define ICM20689_YG_OFFS_USRH        (0x15)
#define ICM20689_YG_OFFS_USRL        (0x16)
#define ICM20689_ZG_OFFS_USRH        (0x17)
#define ICM20689_ZG_OFFS_USRL        (0x18)

#define ICM20689_SMPLRT_DIV          (0x19)

#define ICM20689_CONFIG              (0x1A)
#define ICM20689_GYRO_CONFIG         (0x1B)
#define ICM20689_ACCEL_CONFIG        (0x1C)
#define ICM20689_ACCEL_CONFIG_2      (0x1D)
#define ICM20689_LP_MODE_CFG         (0x1E)

#define ICM20689_FIFO_EN             (0x23)

#define ICM20689_FSYNC_INT           (0x36)

#define ICM20689_INT_PIN_CFG         (0x37)

#define ICM20689_INT_ENABLE          (0x38)

#define ICM20689_INT_STATUS          (0x3A)

#define ICM20689_ACCEL_XOUT_H        (0x3B)
#define ICM20689_ACCEL_XOUT_L        (0x3C)
#define ICM20689_ACCEL_YOUT_H        (0x3D)
#define ICM20689_ACCEL_YOUT_L        (0x3E)
#define ICM20689_ACCEL_ZOUT_H        (0x3F)
#define ICM20689_ACCEL_ZOUT_L        (0x40)

#define ICM20689_TEMP_OUT_H          (0x41)
#define ICM20689_TEMP_OUT_L          (0x42)

#define ICM20689_GYRO_XOUT_H         (0x43)
#define ICM20689_GYRO_XOUT_L         (0x44)
#define ICM20689_GYRO_YOUT_H         (0x45)
#define ICM20689_GYRO_YOUT_L         (0x46)
#define ICM20689_GYRO_ZOUT_H         (0x47)
#define ICM20689_GYRO_ZOUT_L         (0x48)
 
#define ICM20689_SIGNAL_PATH_RESET   (0x68)
 
#define ICM20689_ACCEL_INTEL_CTRL    (0x69)

#define ICM20689_USER_CTRL           (0x6A)

#define ICM20689_PWR_MGMT_1          (0x6B)
#define ICM20689_PWR_MGMT_2          (0x6C)

#define ICM20689_FIFO_COUNTH         (0x72)
#define ICM20689_FIFO_COUNTL         (0x73)
#define ICM20689_FIFO_R_W            (0x74)

#define ICM20689_WHO_AM_I            (0x75)

#define ICM20689_XA_OFFSET_H         (0x77)
#define ICM20689_XA_OFFSET_L         (0x78)
#define ICM20689_YA_OFFSET_H         (0x79)
#define ICM20689_YA_OFFSET_L         (0x7A)
#define ICM20689_ZA_OFFSET_H         (0x7D)
#define ICM20689_ZA_OFFSET_L         (0x7E)
 
void setup(void);
void start(uint32_t dataRate);
void saveCalibration(void);
void setCalibration(char accel, char gyro, char mag);
void calculate(float u0, float u1, float u2, float z0, float z1,float z2, float h);
void tare(void);
void tareZ(void);
void alignFIFOs(void);
void pushData(void);
float savGol(unsigned int startPosition);
void clearPersistentTare(void);
void startRun(void);
void readFIFO(uint8_t device, float* values);
void writeRegister(uint8_t device, uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t device, uint8_t reg);
void writeRegisterBits(uint8_t device, uint8_t reg, uint8_t mask, uint8_t value);
uint16_t readFIFOcount(uint8_t device);
void fifoTimer(void);
float timer(uint8_t device);
void pullData(void);
float extractFloat(uint8_t index);
