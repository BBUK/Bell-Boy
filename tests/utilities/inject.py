#!/usr/bin/python2
import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import csv

# Note: timestamp is in microseconds

timestamp = 0
lastgyro = 0.0
lastAngle = 0.0
flipAngleCorrection = 0.0

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
imu.IMUInit()

print("IMU Name: " + imu.IMUName())

with open('input', 'rb') as f:
    reader = csv.reader(f)
    imu_data = list(reader)

# this is how to turn off the magnetometer

imu.setCompassEnable(False)
output=[]
smaccel = 0
for row in imu_data:
    oa = float(row[0][2:])
    ay = -float(row[3][3:])*9.81
    ax = float(row[4][3:])*9.81
    az = -float(row[5][3:])*9.81
    mx = float(0)
    my = float(0)
    mz = float(0)
    gy = math.radians(float(row[6][3:]))
    gx = -math.radians(float(row[7][3:]))
    gz = math.radians(float(row[8][3:]))

    imu.setExtIMUData(gx, gy, gz, ax, ay, az, mx, my, mz, timestamp)
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    gyro = data["gyro"]
    accel = imu.getAccel()
#    if abs(gyro[0]) > 1.5:
#        imu.setSlerpPower(0.00)
#    else:
#    imu.setSlerpPower(0.5)

    angle = math.degrees(fusionPose[0]) + flipAngleCorrection
#    angle = math.degrees(fusionPose[0])
    if (lastAngle - angle) > 120 : #positive to negative flip
        angle -= flipAngleCorrection
        flipAngleCorrection += 360
        angle += flipAngleCorrection
    elif (lastAngle - angle) < -120: # negative to positive flip
        angle -= flipAngleCorrection
        flipAngleCorrection -= 360
        angle += flipAngleCorrection
    lastAngle = angle
    
    smaccel = 0.95*smaccel + 0.05*((math.degrees(gyro[0])-lastgyro)/0.0025)

#    output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},OA :{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},GX1:{7:.3f},GY1:{8:.3f},GZ1:{9:.3f}".format(math.degrees(fusionPose[0]),math.degrees(gyro[0]),math.degrees(accel[1])*8192/50,oa,accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]))
#    output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},ATN:{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},GX1:{7:.3f},GY1:{8:.3f},GZ1:{9:.3f}".format(angle,math.degrees(gyro[0]),math.degrees(gyro[0]-lastgyro)*8192.0/100.0,math.atan2(ay,az)*180/3.142,accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]))
    output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},ATN:{3:1f},AX1:{4:.3f},AY1:{5:.3f},AZ1:{6:.3f},GX1:{7:.3f},GY1:{8:.3f},GZ1:{9:.3f}".format(angle,math.degrees(gyro[0]),smaccel,math.atan2(ay,az)*180/3.142,accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]))
    lastgyro = math.degrees(gyro[0])
    timestamp += 2500

with open('output', 'wb') as f:
    writer = csv.writer(f)
    for row in output:
        writer.writerow(row.split(','))
