# Copyright (c) 2017 Peter Budd. All rights reserved
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
# associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
# subject to the following conditions:
#     * The above copyright notice and this permission notice shall be included in all copies or substantial 
#       portions of the Software.
#     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#       BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
#       IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
#       WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
#       SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER, 
#       ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY 
#       FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW

# This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
# bell rope.  The Bell-Boy uses rotationsl acceleration of the bell as a proxy for force applied.  The
# hardware is currently a Pi Zero running Arch Linux and an Adafruit Precision NXP Breakout Board

# I pulled anippets of code from loads of places so if any of you recognise anything you wrote - thanks! 

# The script below is used by pywebsocket https://github.com/google/pywebsocket.  The websocket created by pywebsocket
# is used to deal with communication with the users browser and the Javascrip code running on it. 

from multiprocessing import Process, Queue, Event
from time import time,sleep,strftime
from os import listdir,system
from os.path import isfile, join
from math import atan, atan2, sqrt
from subprocess import call
import sys
import os.path
import math

import dcmimu

sample_period = None    # this is the sample period measured by dcmimu.NXP_fifo_timer().  It
                        # may differ from the chosen sample rate (datasheet says by up to 2.5%)
chosen_sample_rate = 200 # this is the chosen number of samples per second.  Must be one of 25 50 100 200 400 and 800

dataQueue=Queue()

worker = None
workerevent = None

#system("/usr/bin/sync && /usr/bin/shutdown -P now")

def web_socket_do_extra_handshake(request):
    pass
#    request.ws_stream.send_message("SAMP:%f", binary=False)

    
# There are eight possible command sent by the user's browser:
# (a)   STRT:[filename] Start recording a session with a bell.  The bell
#       must be at stand and (reasonably) stationary otherwise this script
#       will return ESTD: or EMOV: either of which signals the browser 
#       to stop expecting data.  If Filename is not provided then a
#       default filename of "CurrentRecording [date]" is used.
# (b)   STOP: stops a recording in progress.  This also saves off the
#       collected data into a file in /data/samples/
# (c)   LDAT: a request for data from the broswer, only works when there
#       is a recording in progress
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
# (iv)  NDAT: indicates that there is no current data to send to the browser
# (v)   SAMP: returns the sample period
# (vi)  EIMU: IMU not detected
# (vii) ESTD: bell not at stand (aborts started session)
# (viii)EMOV: bell moving when session started (the bell has to be stationary at start)
# (ix)  ESAM: (sent by dcmimu) sample period needs to be measured first - dcmimu.NXP_fifo_timer()
# (x)   EFIF: (sent by dcmimu) fifos need to be started before calling dcmimu.NXP_pull_data()
# (xi)  ESTR: (sent by dcmimu) internal error related to data from dcmimu - shouldn't happen!
# (xii) DATA:[string]  chunk of data from a previously stored file.  In same format as LIVE:
# (xiii)EOVF: (sent by dcmimu) fifo overflow flagged.

def web_socket_transfer_data(request):
    global worker,workerevent,sample_period
    while True:
        instruction = request.ws_stream.receive_message()
        if instruction is None:
            return
        if instruction[:5] == "STRT:":
##TODO: check whether there is already a thread running and end it
            while (not dataQueue.empty()):
                dataQueue.get(False)
            workerevent = Event()
            fileName = instruction[5:]
            if fileName == "":
                fileName = "CurrentRecording" + strftime("%Y%m%d-%H%M%S")
            worker = process_sample(fileName, sample_period, dataQueue,workerevent)
            worker.start()
        elif instruction[:5] == "STOP:":
            if worker is not None:
                workerevent.set()
                sleep(4) # allow existing grabber thread to stop but must dump any further LDATS: recieved (done in JS)
                request.ws_stream.send_message("STPD:quite a few", binary=False)
                while (not dataQueue.empty()):
                    dataQueue.get(False)
            else:
                request.ws_stream.send_message("ESTP:", binary=False)
                while (not dataQueue.empty()):
                    dataQueue.get(False)

        elif instruction[:5] == "LDAT:":
            if not dataQueue.empty():
                data_out=[]
                while (not dataQueue.empty()):
                    temp=dataQueue.get()
                    if temp[:1] == "E": # error flagged
                        workerevent.set() # shut down grabber thread.  Doesn't appear to need a sleep here
                        data_out=[]
                        data_out.append(temp)
                        while (not dataQueue.empty()):
                            temp = dataQueue.get(False)
                    else:
                        data_out.append("LIVE:" + temp)
                request.ws_stream.send_message(''.join(data_out), binary=False)
            else:
                request.ws_stream.send_message("NDAT:", binary=False) # send no-data as response

        elif instruction[:5] == "FILE:":
            for line in [f for f in listdir("/data/samples/") if isfile(join("/data/samples/", f))]:
                request.ws_stream.send_message("FILE:" + line, binary=False)
        elif instruction[:5] == "LOAD:":
            with open('/data/samples/' + instruction[5:]) as f:
                chunk = 0
                data_out = []
                reduceSampleFreq = 0 # only give one out of 2 lines to client.
                for line in f:
                    reduceSampleFreq += 1
                    if reduceSampleFreq & 0x01 != 0:
                        continue
                    chunk += 1
                    if (line == "" or len(line.split(",")) < 3): #ignore bad line
                        chunk -= 1
                        continue
                    data_out.append("DATA:" + line)
                    if (chunk % 60) == 0:
                        request.ws_stream.send_message(''.join(data_out), binary=False)
                        data_out = []
                if (chunk % 60) != 0:
                    request.ws_stream.send_message(''.join(data_out), binary=False)
                request.ws_stream.send_message("LFIN: " + str(chunk), binary=False)
        elif instruction[:5] == "DATE:":
            system("/usr/bin/date --set=\"$(date --date='@%s')\" && touch /var/lib/systemd/clock" % instruction[5:])
        elif instruction[:5] == "SAMP:":
            try:
                result = dcmimu.NXP_test()
                if result < 0:
                    request.ws_stream.send_message("EIMU:", binary=False)
                else:
                    sample_period=dcmimu.NXP_fifo_timer(chosen_sample_rate)
                    request.ws_stream.send_message("SAMP:" + str(sample_period), binary=False)
                    sample_period = result 
            except:
                request.ws_stream.send_message("EIMU:", binary=False)
        elif instruction[:5] == "SHDN:":
            system("/usr/bin/sync && /usr/bin/shutdown -P now")
            #pass


# The purpose of this class is to pull readings from the IMU sensor and use them to calculate the rotational
# acceleration, rotational velocity and position (angle) of the bell.
#
# The co-ordinate system we are using is such that zero degrees is TDC at start of handstroke/end of backstroke
# and 360 degrees is TDC at end of handstroke/beginning of backstroke.  180 degrees is with the open end of the
# bell facing straight down.  The overall range is between about -15 degrees (stand at handstroke) to 
# 375 degrees (stand at backstroke).
# 
# The special properties of the bell system introduces some complexities and makes some simplifications possible.
# 
# One complexity is that for a significant proportion of the stroke, the bell is under large angular
# acceleration - this means that accelerometer values can not be reliably used to determine orientation.
# The traditional methods of calulating orientation (eg using complementary or kalman filters) will not work well.
# What this class does, therefore, is to use a gyro-only position calculation when the bell is accelerating
# hard and flip to a gyro only method otherwise.

class process_sample(Process):
    def __init__ (self, filename, interval, q, workerevent):
        self.filename = filename
        self.interval = interval
        self.q = q
        self.datastore = []
        self.stopped = workerevent
        self.rotations = [ -1, -1, -1] # x,y and z direction corrections
        Process.__init__ (self)
            
    def run(self):
        try:
            if dcmimu.NXP_test() < 0:
                self.q.put("EIMU:")
                return
            if dcmimu.set_rotations(self.rotations[0],self.rotations[1],self.rotations[2]) < 0:
                self.q.put("EIMU:")
                return
            dcmimu.set_debug()
            dcmimu.set_swap_XY()
            initials = dcmimu.NXP_get_orientation()
            if abs(initials[0]) > 5.0 or abs(initials[1]) > 5.0 or abs(initials[2]) > 5.0:
                self.q.put("EMOV:") # bell moving
                return
            start_angle = atan2(initials[4],initials[5])*180/3.142
            if start_angle < 20 and start_angle >= 3: #angle is "wrong" way round
                print "wrong %f" % (start_angle)
                dcmimu.set_rotations(-self.rotations[0],-self.rotations[1],self.rotations[2])
            elif start_angle > -20 and start_angle <= -3: #angle is "right" way round
                print "right %f" % (start_angle)
            else:
                print "out of range %f" % (start_angle)
                self.q.put("ESTD:")
            dcmimu.NXP_start_fifos(200,500,2) # ODR, gyro range, accel range
            sleep(0.05)
            results = dcmimu.NXP_pull_data() # get Kalman to settle down
            print results
        except:
            print("Exception")
            self.q.put("EIMU:")
            dcmimu.NXP_stop_fifos();
            return

        while not self.stopped.wait(self.interval * 4): # wait for a few samples to build up before pulling them
            results = dcmimu.NXP_pull_data()
            if len(results) == 0:
                continue
            for entry in results.split("\n"):
                if entry[:5] == "EOVF:":
                    print("Overflow")
                else:
                    self.q.put(entry[:29] + "\n")
                self.datastore.append(entry)
#         output.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX:{3:.3f},AY:{4:.3f},AZ:{5:.3f},GX:{6:.3f},GY:{7:.3f},GZ:{8:.3f}, ATN:{9:.3f}".format(calcAngle,gx1,(accTang*8192.0/100.0)-0*sin(calcAngle*3.1416/180),ax1,ay1,az1,gx1,gy1,gz1, accAngle))

            if self.q.qsize() > 500: # if nowt being pulled for a bit then assume broken link and save off what we have
                self.filename += "(aborted)"
                break
        dcmimu.NXP_stop_fifos()
        with open('/data/samples/' + self.filename, "w") as f:
            f.writelines( "%s\n" % item for item in self.datastore )
          
