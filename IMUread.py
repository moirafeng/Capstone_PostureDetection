"""Bluetooth motion tracker module.
Copyright 2017 Mark Mitterdorfer
Class to read from a Bluetooth MPU6050 device.
Obtain acceleration, angular velocity, angle and temperature
"""

import threading
import struct
import bluetooth
import subprocess



class MotionTracker(object):
    """Class to track movement from MPU6050 Bluetooth device.
    """

    def __init__(self, bd_addr, port):
        """Initialization for tracker object.
        Args:
            bd_addr (str) : Bluetooth address
            port (int, optional) : Port, defaults to 1
        Attributes:
            bd_addr (str): Bluetooth address
            port (int): Port
            sock (bluetooth.bluez.BluetoothSocket) : Bluetooth socket object
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
        print("try to connect....")
        		
	
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.connect((self.bd_addr, self.port))
        
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        self.angv_x = 0.0
        self.angv_y = 0.0
        self.angv_z = 0.0

        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0

        self.temperature = 0.0
        self.__thread_read_device_data = None

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

                    self.acc_x = struct.unpack("<h", ax_l + ax_h)[0] / 32768.0 * 16.0
                    self.acc_y = struct.unpack("<h", ay_l + ay_h)[0] / 32768.0 * 16.0
                    self.acc_z = struct.unpack("<h", az_l + az_h)[0] / 32768.0 * 16.0
                    self.temperature = struct.unpack("<h", t_l + t_h)[0] / 340.0 + 36.25
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

                    self.angv_x = struct.unpack("<h", wx_l + wx_h)[0] / 32768.0 * 2000.0
                    self.angv_y = struct.unpack("<h", wy_l + wy_h)[0] / 32768.0 * 2000.0
                    self.angv_z = struct.unpack("<h", wz_l + wz_h)[0] / 32768.0 * 2000.0
                    self.temperature = struct.unpack("<h", t_l + t_h)[0] / 340.0 + 36.25
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

                    self.ang_x = round((struct.unpack("<h", roll_l + roll_h)[0] / 32768.0 * 180.0), 3)
                    self.ang_y = round((struct.unpack("<h", pitch_l + pitch_h)[0] / 32768.0 * 180.0), 3)
                    self.ang_z = round((struct.unpack("<h", yaw_l + yaw_h)[0] / 32768.0 * 180.0), 3)
                    self.temperature = struct.unpack("<h", t_l + t_h)[0] / 340.0 + 36.25
                    
               

def main():
    """Test driver stub.
        """
    name1 = 'IMU1'
    name2 = 'IMU2'
    addr1 = '20:18:08:08:09:01' # addr of IMU1
    addr2 = '20:18:08:08:11:42' # addr of IMU2
    passkey = "1234"

    # kill any "bluetooth-agent" process that is already running
    subprocess.call("kill -9 `pidof bluetoothctl`",shell=True)

    # Start a new "bluetooth-agent" process where XXXX is the passkey
    status = subprocess.call("bluetoothctl " + passkey + " &",shell=True)
    
    try:
        IMU1 = MotionTracker(bd_addr=addr2,port=1)
        print("connected to IMU1")
        IMU1.start_read_data()
        #IMU1.stop_read_data()
        #print("ang_x:", IMU1.ang_x, "ang_y:", IMU1.ang_y, "ang_z:", IMU1.ang_z)
        #IMU2 = MotionTracker(bd_addr=addr1,port=2)
        #print("connected to IMU2")
        #IMU2.start_read_data()
        #print("IMU2 ang_x:", IMU2.ang_x, "ang_y:", IMU2.ang_y, "ang_z:", IMU2.ang_z)
        #IMU1.start_read_data()
        #print("ang_x:", IMU1.ang_x, "ang_y:", IMU1.ang_y, "ang_z:", IMU1.ang_z)

        while True:
            print("IMU1 ang_x:", IMU1.ang_x, "ang_y:", IMU1.ang_y, "ang_z:", IMU1.ang_z)
            #print("IMU2 ang_x:", IMU2.ang_x, "ang_y:", IMU2.ang_y, "ang_z:", IMU2.ang_z)

    except KeyboardInterrupt:
        session.stop_read_data()
        
        
if __name__ == "__main__":
    main()
