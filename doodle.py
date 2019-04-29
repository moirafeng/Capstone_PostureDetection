import numpy as np
import math
import time

def main():
	st = time.time()
	m = np.linalg.norm([1,2,3])
	#~ m = math.sqrt(1*1 + 2*2 + 3*3)
	delt = time.time() - st
	print(delt)

if __name__ == "__main__":
	main()
