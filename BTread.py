
import bluetooth,subprocess

# scan for devices
devices = bluetooth.discover_devices(duration = 4, lookup_names = True,\
flush_cache = True,lookup_class = False)

name1 = "IMU1"
name2 = "IMU2"
addr1 = '20:18:08:08:09:01' # addr of IMU1
addr2 = '20:18:08:08:09:01' # addr of IMU2
port = 1
passkey = "1234"

# kill any "bluetooth-agent" process that is already running
subprocess.call("kill -9 `pidof bluetoothctl`",shell=True)

# Start a new "bluetooth-agent" process where XXXX is the passkey
status = subprocess.call("bluetoothctl " + passkey + " &",shell=True)
print("hi")
# Now, connect in the same way as always with PyBlueZ
try:
    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    b = 6
    print("hi2")
    s.connect((addr1,port))
    a = 5
except bluetooth.btcommon.BluetoothError as err:
    # Error handler
    print("error when pairing")
    pass
    



