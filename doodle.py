import numpy as np
import math
import time
from gpiozero import LED

global green
global blue
global red

green = LED(17)
blue = LED(27)
red = LED(22)

def main():
	#~ st = time.time()
	#~ m = np.linalg.norm([1,2,3])
	#~ m = math.sqrt(1*1 + 2*2 + 3*3)
	#~ delt = time.time() - st
	#~ print(delt)
	pos = 1

	while True:
		
		if pos	== 0: # good standing posture: blue
			blue.on()
			green.off()
			red.off()
			print("blue")
		elif pos == 1: # sway-back: red
			red.on()
			blue.off()
			green.off()
		elif pos == 2: # sit: green
			blue.off()
			green.on()
			red.off()
		elif pos == 3: # moving: no led on
			blue.off()
			green.off()
			red.off()


if __name__ == "__main__":
	main()
