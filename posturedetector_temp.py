

""" Posture data aquisition & labeling module.

<Original> Bluetooth motion tracker module.
Copyright 2017 Mark Mitterdorfer

Class to read from a Bluetooth MPU6050 device.
Obtain acceleration, angular velocity, angle and temperature
"""

import sys
import time, threading
import struct
import bluetooth, subprocess
import keyboard

import os, csv
from itertools import zip_longest
import numpy as np
import math
from gpiozero import LED
# import pandas as pd

global green
global blue
global red

green = LED(17)
blue = LED(27)
red = LED(22)

class MotionTracker(object):
	"""Class to track movement from MPU6050 Bluetooth device.
	"""

	def __init__(self, bd_addr, port, delta):
		"""Initialization for tracker object.

		Args:
			bd_addr (str) : Bluetooth address
			port (int, optional) : Port, defaults to 1
		Attributes:
			bd_addr (str): Bluetooth address
			port (int): Port
			sock (bluetooth.bluez.BluetoothSocket) : Bluetooth socket object
			t (float) : time lapsed since start of receiving data 
			acc_x (float) : acceleration in X
			acc_y (float) : acceleration in Y
			acc_z (float) : acceleration in Z
			angv_x (float) : angular velocity in X
			angv_y (float) : angular velocity in Y
			angv_z (float) : angular velocity in Z
			ang_x (float) : angle degrees in X
			ang_y (float) : angle degrees in Y
			ang_z (float) : angle degrees in Z
			temperature (float) : temperature in degrees celsius
			__thread_read_device_data (threading.Thread) : Read input thread
			
		"""
		
		self.bd_addr = bd_addr
		self.port = port
		self.delta = delta
		
		self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		self.sock.connect((self.bd_addr, self.port))
		print("<<IMU connected!>>")
		
		self.acc_x = 0.0
		self.acc_y = 0.0
		self.acc_z = 0.0

		self.angv_x = 0.0
		self.angv_y = 0.0
		self.angv_z = 0.0

		self.ang_x = 0.0
		self.ang_y = 0.0
		self.ang_z = 0.0

		self.t = 0.0
		self.pos = 4
		self.__thread_read_device_data = None
		self.timer_thread = None
		
		global t # time
		global l # posture label
		global acc_x
		global acc_y
		global acc_z
		global w_x
		global w_y
		global w_z
		global ang_x
		global ang_y
		global ang_z


		t = [0]
		l = [4]
		acc_x = [0]
		acc_y = [0]
		acc_z = [0]
		w_x = [0]
		w_y = [0]
		w_z = [0]
		ang_x = [0]
		ang_y = [0]
		ang_z = [0]
		


	def start_read_data(self):
		"""Start reading from device. Wait for a second or two before
		reading class attributes to allow values to 'settle' in.
		Non blocking I/O performed via a private read thread.
		"""

		self.__thread_read_device_data = threading.Thread(target=self.__read_device_data)
		self.__thread_read_device_data.is_running = True
		self.__thread_read_device_data.start()

	def stop_read_data(self):
		"""Stop reading from device. Join back to main thread and
		close the socket.
		"""

		self.__thread_read_device_data.is_running = False
		self.__thread_read_device_data.join()
		self.sock.close()

	def __read_device_data(self):
		"""Private method to read device data in 9 byte blocks.
		"""
		
		start = time.time()
		while self.__thread_read_device_data.is_running:
			data_block = self.sock.recv(1)
			if data_block == b'\x55':
				data_block_type = self.sock.recv(1)
				# Acceleration
				if data_block_type == b'\x51':
					# Read 9 byte block
					ax_l = self.sock.recv(1)
					ax_h = self.sock.recv(1)
					ay_l = self.sock.recv(1)
					ay_h = self.sock.recv(1)
					az_l = self.sock.recv(1)
					az_h = self.sock.recv(1)
					t_l = self.sock.recv(1)
					t_h = self.sock.recv(1)
					self.sock.recv(1) # Check sum, ignore

					self.acc_x = round((struct.unpack("<h", ax_l + ax_h)[0] / 32768.0 * 16.0), 3)
					self.acc_y = round((struct.unpack("<h", ay_l + ay_h)[0] / 32768.0 * 16.0), 3)
					self.acc_z = round((struct.unpack("<h", az_l + az_h)[0] / 32768.0 * 16.0), 3)
					
					self.t = round(time.time() - start, 3)
					t.append(self.t)
					l.append(self.pos)
					
					acc_x.append(self.acc_x)
					acc_y.append(self.acc_y)
					acc_z.append(self.acc_z)

				# Angular velocity
				elif data_block_type == b'\x52':
					# Read 9 byte block
					wx_l = self.sock.recv(1)
					wx_h = self.sock.recv(1)
					wy_l = self.sock.recv(1)
					wy_h = self.sock.recv(1)
					wz_l = self.sock.recv(1)
					wz_h = self.sock.recv(1)
					t_l = self.sock.recv(1)
					t_h = self.sock.recv(1)
					self.sock.recv(1)  # Check sum, ignore

					self.angv_x = round((struct.unpack("<h", wx_l + wx_h)[0] / 32768.0 * 2000.0), 2)
					self.angv_y = round((struct.unpack("<h", wy_l + wy_h)[0] / 32768.0 * 2000.0), 2)
					self.angv_z = round((struct.unpack("<h", wz_l + wz_h)[0] / 32768.0 * 2000.0), 2)
					
					w_x.append(self.angv_x)
					w_y.append(self.angv_y)
					w_z.append(self.angv_z)
				# Angle
				elif data_block_type == b'\x53':
					# Read 9 byte block
					roll_l = self.sock.recv(1)
					roll_h = self.sock.recv(1)
					pitch_l = self.sock.recv(1)
					pitch_h = self.sock.recv(1)
					yaw_l = self.sock.recv(1)
					yaw_h = self.sock.recv(1)
					t_l = self.sock.recv(1)
					t_h = self.sock.recv(1)
					self.sock.recv(1)  # Check sum, ignore

					self.ang_x = round((struct.unpack("<h", roll_l + roll_h)[0] / 32768.0 * 180.0), 2)
					self.ang_y = -round((struct.unpack("<h", pitch_l + pitch_h)[0] / 32768.0 * 180.0), 2)
					self.ang_z = round((struct.unpack("<h", yaw_l + yaw_h)[0] / 32768.0 * 180.0), 2)
					
					ang_x.append(self.ang_x)
					ang_y.append(self.ang_y)
					ang_z.append(self.ang_z)
					
	def calibration(self):
		off_ind = 3 # index of data point for calibration
		global ang_xos
		global ang_yos
		global ang_zos
		print("<<Start calibration>>")
		while True:
			print("Calibration...")
			time.sleep(self.delta)
			if len(t) <= 4:
				pass
			else:
				ang_xos = ang_x[off_ind]
				ang_yos = ang_y[off_ind]
				ang_zos = ang_z[off_ind]
				print("<<End calibration>>")
				break
				

	def detectpos(self):
		# this function will run every delta seconds
		# weights & parameters in linear model learned from SVM
		ss_wt = [-0.032, 0.054, 0.004]
		sw_wt = [-0.062, 0.195, -0.027]
		w_ths = 10
		ss_ths = 1.717 # stand-sit threshold
		sw_ths = 1.195 # sway-back threshold
		
		ang_xn = self.ang_x - ang_xos
		ang_yn = self.ang_y - ang_yos
		ang_zn = self.ang_z - ang_zos
		
		w_mag = math.sqrt(self.angv_x*self.angv_x + self.angv_y*self.angv_y \
		+ self.angv_z*self.angv_z)
			
		if w_mag < w_ths: # if still
			ss_y = ss_wt[0]*ang_xn + ss_wt[1]*ang_yn + ss_wt[2]*ang_zn
			if ss_y < ss_ths: # if stand/sway-back
				sw_y = sw_wt[0]*ang_xn + sw_wt[1]*ang_yn + sw_wt[2]*ang_zn
				if sw_y > sw_ths: # if sway-back
					self.pos = 1 
				else:
					self.pos = 0
			else:
				self.pos = 2
		else:
			self.pos = 3
				

	def update_state(self):	
		next_call = time.time()
		while True:
			try:
				self.detectpos()
				self.feedback()
				print("time:", self.t, " ang_x:", self.ang_x, " ang_y:", \
				self.ang_y, " ang_z:", self.ang_z, " label:", self.pos)
				next_call = next_call + self.delta
				time.sleep(next_call - time.time())
			except KeyboardInterrupt:
				self.stop_read_data()
				savedata()
				break				
		
	def feedback(self):	
		if self.pos	== 0: # good standing posture: blue
			blue.on()
			green.off()
			red.off()
		elif self.pos == 1: # sway-back: red
			red.on()
			blue.off()
			green.off()
		elif self.pos == 2: # sit: green
			blue.off()
			green.on()
			red.off()
		elif self.pos == 3: # moving: no led on
			blue.off()
			green.off()
			red.off()

		
	def label_data(self):
		# initialize labeling
		global g_prs
		global s_prs
		global t_prs
		global m_prs
		g_prs = 0
		s_prs = 0
		t_prs = 0
		m_prs = 0	
		
		while True:
			try:
				# Detect keyboard input
				g_prs = keyboard.is_pressed('0')
				s_prs = keyboard.is_pressed('1')
				t_prs = keyboard.is_pressed('2')
				m_prs = keyboard.is_pressed('3')
				if self.pos == 4: # when not in any labeled state
					if g_prs:
						self.pos = 0
						print("Good standing posture")	
					elif s_prs:
						self.pos = 1
						print("Sway-back posture!")
					elif t_prs:
						self.pos = 2
						print("Sitting")
					elif m_prs:
						self.pos = 3
						print("Not still")
				elif 1 in [g_prs, s_prs, t_prs, m_prs]: # when in labeled state
					self.pos = 4
					print("Posture ends")
				time.sleep(0.2)					
				
			except KeyboardInterrupt:
				self.stop_read_data()
				savedata()
				break		

def savedata():
	""" Save & export data to csv file.
	"""
	# Data to be saved
	data = [t, acc_x, acc_y, acc_z, w_x, w_y, w_z, ang_x, ang_y,\
				ang_z, l]
	export = zip_longest(*data, fillvalue = '')
	filename = 'data0.csv'
	# Increment index in file name if name taken
	while(os.path.isfile(filename)):
		dotInd = filename.index('.')
		filename = filename.replace(filename[4:dotInd], str(int(filename[4:dotInd])+1))
	try:
		with open(filename,'w+') as mycsv:
			wr = csv.writer(mycsv, quoting = csv.QUOTE_ALL)
			wr.writerow(("time","acc_x","acc_y","acc_z","w_x","w_y",\
			"w_z","ang_x","ang_y","ang_z","label"))
			wr.writerows(export)
			mycsv.flush() 
			print("Done saving data to " + filename)
	except:
		print("Error when writing file!")
		pass			   

def main():
	"""Test driver stub.
		"""
	name1 = 'IMU1'
	name2 = 'IMU2'
	addr1 = '20:18:08:08:09:01' # addr of IMU1
	addr2 = '20:18:08:08:11:42' # addr of IMU2
	passkey = "1234"


	# kill any "bluetooth-agent" process that is already running
	subprocess.call("kill -9 `pidof bluetoothctl`", shell=True)

	# Start a new "bluetooth-agent" process where XXXX is the passkey
	status = subprocess.call("bluetoothctl " + passkey + " &", shell=True)
	
	try:
		IMU1 = MotionTracker(bd_addr=addr1, port=1, delta=2)
		IMU1.start_read_data()
		#IMU1.label_data()
		IMU1.calibration()
		IMU1.update_state()		

	except KeyboardInterrupt:
		pass

			
if __name__ == "__main__":
	main()
