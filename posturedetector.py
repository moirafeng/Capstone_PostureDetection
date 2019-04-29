

""" Posture data aquisition & labeling module.

<Original> Bluetooth motion tracker module.
Copyright 2017 Mark Mitterdorfer

Class to read from a Bluetooth MPU6050 device.
Obtain acceleration, angular velocity, angle and temperature
"""

import time, threading
import struct
import bluetooth, subprocess

import os, csv
from itertools import zip_longest
import numpy as np
from gpiozero import LED
# import pandas as pd



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
		self.__thread_read_device_data = None
		self.timer_thread = None

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
		global t
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
		acc_x = [0]
		acc_y = [0]
		acc_z = [0]
		w_x = [0]
		w_y = [0]
		w_z = [0]
		ang_x = [0]
		ang_y = [0]
		ang_z = [0]
		
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
					self.ang_y = round((struct.unpack("<h", pitch_l + pitch_h)[0] / 32768.0 * 180.0), 2)
					self.ang_z = round((struct.unpack("<h", yaw_l + yaw_h)[0] / 32768.0 * 180.0), 2)
					
					ang_x.append(self.ang_x)
					ang_y.append(self.ang_y)
					ang_z.append(self.ang_z)
				#~ else :
					#~ pass

	def detectpos(self):
		# this function will run every delta seconds
		#~ self.pos = 0
		#~ return pos
		pass

	#~ def update_state(self):	
		#~ self.detectpos()
		#~ feedback()
		#~ print("time:", self.t, " ang_x:", self.ang_x, " ang_y:", \
		#~ self.ang_y, " ang_z:", self.ang_z)
		#~ next_call = next_call + self.delta
		#~ try:
			#~ self.timer_thread = threading.Timer(self.delta, self.update_state)
			#~ self.timer_thread.daemen = True
			#~ self.timer_thread.start()
		#~ except KeyboardInterrupt:
			#~ self.timer_thread.cancel()
			#~ print("cancel...")

	def update_state(self):	
		next_call = time.time()
		while True:
			try:
				self.detectpos()
				feedback()
				print("time:", self.t, " ang_x:", self.ang_x, " ang_y:", \
				self.ang_y, " ang_z:", self.ang_z)
				next_call = next_call + self.delta
				time.sleep(next_call - time.time())
			except KeyboardInterrupt:
				self.stop_read_data()
				savedata()
				break
				

	#~ def update_state(self):	
		#~ self.detectpos()
		#~ feedback()
		#~ print("time:", self.t, " ang_x:", self.ang_x, " ang_y:", \
		#~ self.ang_y, " ang_z:", self.ang_z)
		#~ next_call = next_call + self.delta
		#~ try:
			#~ self.timer_thread = threading.Timer(self.delta, self.update_state)
			#~ self.timer_thread.daemen = True
			#~ self.timer_thread.start()
		#~ except KeyboardInterrupt:
			#~ self.timer_thread.cancel()
			#~ print("cancel...")
						
		
def feedback():		
	pass	

def savedata():
	""" Save & export data to csv file.
	"""
	# Data to be saved
	data = [t, acc_x, acc_y, acc_z, w_x, w_y, w_z, ang_x, ang_y,\
				ang_z]
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
			"w_z","ang_x","ang_y","ang_z"))
			wr.writerows(export)
			mycsv.flush() 
			print("Done saving data to " + filename)
	except:
		print("Error when writing file...")
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
		IMU1.update_state()
		
		#~ IMU1.timer_thread = threading.Thread(target=IMU1.update_state)
		#~ IMU1.timer_thread.daemen = True
		#~ IMU1.timer_thread.start()

		#~ while True:
			#~ now = time.time()
			#~ print("time:", IMU1.t, " ang_x:", IMU1.ang_x, " ang_y:", \
			#~ IMU1.ang_y, " ang_z:", IMU1.ang_z)		

	except KeyboardInterrupt:
		pass

			
if __name__ == "__main__":
	main()
