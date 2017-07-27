import threading
import Queue
from time import sleep,strftime
from os import listdir,system
from os.path import isfile, join
from math import atan, atan2, sqrt
from subprocess import call
import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import math

SETTINGS_FILE = "RTIMULib"
imu=None

dataQueue=Queue.Queue()

currentline = 0
worker = None
workerevent = None

def web_socket_do_extra_handshake(request):
	pass

def web_socket_transfer_data(request):
	global worker,imu,workerevent
	while True:
		instruction = request.ws_stream.receive_message()
		if instruction is None:
			return
		if instruction[:5] == "STRT:":
			if imu is None:
				s = RTIMU.Settings("/srv/http/RTIMULib")
				imu = RTIMU.RTIMU(s)
				if (not imu.IMUInit()):
					request.ws_stream.send_message("EIMU:", binary=False)
				else:
					imu.setSlerpPower(0.02)
					imu.setGyroEnable(True)
					imu.setAccelEnable(True)
					imu.setCompassEnable(False)
##TODO: check whether there is already a thread running and end it
			while (not dataQueue.empty()):
				dataQueue.get(False)
			workerevent = threading.Event()
			worker = process_sample("CurrentRecording" + strftime("%Y%m%d-%H%M%S"), imu, imu.IMUGetPollInterval(), dataQueue,workerevent)
			worker.start()
		elif instruction[:5] == "STOP:":
			if worker is not None:
				workerevent.set()
				sleep(1) # allow existing server threads to stop but must dump any further LDATS: recieved (done in JS)
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
						workerevent.set() # doesn't appear to need a sleep here
						data_out=[]
						data_out.append(temp)
						while (not dataQueue.empty()):
							temp = dataQueue.get(False)
					else:
						data_out.append("LIVE:" + temp)
				request.ws_stream.send_message(''.join(data_out), binary=False)
			else:
				request.ws_stream.send_message("NDAT:", binary=False)

		elif instruction[:5] == "FILE:":
			for line in [f for f in listdir("/data/samples/") if isfile(join("/data/samples/", f))]:
				request.ws_stream.send_message("FILE:" + line, binary=False)
		elif instruction[:5] == "LOAD:":
			with open('/data/samples/' + instruction[5:]) as f:
				chunk = 0
				data_out = []
				for line in f:
					chunk += 1
# when using new live data recordings then data_entry = line
#					data_entry = extract_data_from_line(line)
					if (line == "" or len(line.splt(",")) < 3): #ignore bad line
						chunk -= 1
						continue
					data_out.append("DATA:" + line)
					if (chunk % 60) == 0:
						request.ws_stream.send_message(''.join(data_out), binary=False)
						data_out = []
				if (chunk % 60) != 0:
					request.ws_stream.send_message(''.join(data_out), binary=False)
				request.ws_stream.send_message("LFIN: " + str(chunk), binary=False)
		elif instruction[:5] == "SHDN:":
			system("/usr/bin/sync && /usr/bin/shutdown -P now")
			#pass

class process_sample(threading.Thread):
	def __init__ (self, filename, imu, interval, q, workerevent):
		self.filename = filename
		self.interval = interval
		self.imu = imu
		self.q = q
		self.initcount = 0
		self.datastore = []
		self.lastAngle=999
		self.angleCorrection=0
		self.flipAngleCorrection=0
		self.negateAngle = None
		self.stopped = workerevent
		threading.Thread.__init__ (self)
   
	def run(self):
		while not self.stopped.wait(self.interval/1000.0):
			if self.imu.IMURead():
				data = self.imu.getIMUData()
				fusionPose = data["fusionPose"]
				if self.initcount != 100:
					self.initcount +=1
					if self.initcount == 100:
						self.lastAngle = math.degrees(fusionPose[0]) #check angle of stand
						if self.lastAngle > 160 and self.lastAngle <= 178: #angle is "correct" way round
							self.angleCorrection =  +180.0
							self.negateAngle=False
							self.flipAngleCorrection=-360.0
							self.lastAngle += (self.angleCorrection + self.flipAngleCorrection)
#							print "correct %f %f" % (math.degrees(fusionPose[0]),self.lastAngle)
						elif (self.lastAngle < -160 and self.lastAngle >= -178): #angle is "wrong" way round
							self.angleCorrection =  +180.0
							self.negateAngle=True
							self.flipAngleCorrection=-360.0
							self.lastAngle= -1.0 * self.lastAngle
							self.lastAngle += (self.angleCorrection + self.flipAngleCorrection)
#							print "wrong %f %f" % (math.degrees(fusionPose[0]),self.lastAngle)
						else:
#							print "out of range %f" % (math.degrees(fusionPose[0]))
							self.q.put("ESTD:")
						if data['gyro'][0] > 0.004:
							self.q.put("EMOV:")
					continue
				gyro = data["gyro"]
				accel = self.imu.getAccelResiduals()
#				accel = self.imu.getAccel()

				angle = math.degrees(fusionPose[0])
				rate = gyro[0]
				accn = accel[1]*8192
				if self.negateAngle:
					angle = -1.0*angle
					rate = -1.0*rate
					accn = -1.0*accn
				angle += (self.angleCorrection + self.flipAngleCorrection)
				
				if (self.lastAngle - angle) > 120 : #positive to negative flip
					angle -= self.flipAngleCorrection
					self.flipAngleCorrection += 360
					angle += self.flipAngleCorrection
				elif (self.lastAngle - angle) < -120: # negative to positive flip
					angle -= self.flipAngleCorrection
					self.flipAngleCorrection -= 360
					angle += self.flipAngleCorrection
				self.lastAngle = angle
				entry = "A:{0:.3f},R:{1:.3f},C:{2:.3f}".format(angle,rate,accn)
				self.q.put(entry + "\n")
                self.datastore.append("A:{0:.3f},R:{1:.3f},C:{2:.3f},AX1:{3:.3f},AY1:{4:.3f},AZ1:{5:.3f},GX1:{6:.3f},GY1:{7:.3f},GZ1:{8:.3f}".format(angle,rate,accn,accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]))
#				self.datastore.append("A:{0:.3f},R:{1:.3f},C:{2:.3f}, X:{3:.3f}, Z:{4:.3f}".format(angle,rate,accn,accel[0],accel[2]))
#				self.datastore.append(entry)
				if self.q.qsize() > 500: # if nowt being pulled for 10 secs assume broken link and save off what we have
					break

		with open('/data/samples/' + self.filename, "w") as f:
			f.writelines( "%s\n" % item for item in self.datastore )


