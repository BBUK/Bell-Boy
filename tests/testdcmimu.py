#!/usr/bin/python2

from time import time,sleep,strftime
from os import listdir,system
from os.path import isfile, join
from math import atan, atan2, sqrt
from subprocess import call
import sys
import os.path
import math

import dcmimu

sample_period = None    
                        
chosen_sample_rate = 200

rotations = [ -1, -1, -1 ]
outputs = []

def main():
    if dcmimu.NXP_test() < 0:
        print("IMU not detected")
        return
    if dcmimu.set_rotations(rotations[0],rotations[1],rotations[2]) < 0:
        print("Could not set rotations")
        return
    dcmimu.set_swap_XY()
    dcmimu.set_debug()
    initials = dcmimu.NXP_get_orientation()
    print initials
    if abs(initials[0]) > 5.0 or abs(initials[1]) > 5.0 or abs(initials[2]) > 5.0:
        print("Bell not at stand")
        return
    start_angle = atan2(initials[4],initials[5])*180/3.142
    dcmimu.set_gyro_bias(initials[0],initials[1],initials[2]);
    if start_angle < 20 and start_angle >= 3: #angle is "wrong" way round
        print "wrong %f" % (start_angle)
        dcmimu.set_rotations(-rotations[0],-rotations[1],rotations[2])
    elif start_angle > -20 and start_angle <= -3: #angle is "right" way round
        print "right %f" % (start_angle)
    else:
        print "out of range %f" % (start_angle)
    if dcmimu.NXP_start_fifos(200,500,2) < 0 :# ODR, gyro range, accel range
        print("Start FIFO error")
    while(True):
        sleep(0.05)
        results = dcmimu.NXP_pull_data()
        print results[:29] + "\n"
        outputs.append(results)
            
if __name__ == '__main__':
    try:
        main()  
    except KeyboardInterrupt:
        dcmimu.NXP_stop_fifos()
        with open('dcoutputs', "w") as f:
            f.writelines( "%s\n" % item for item in outputs )
