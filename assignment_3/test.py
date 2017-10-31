import time
import numpy as np
from collections import Counter
import itertools
import random

while True:
	raw_input("Measure")
	data = []
	while len(data)<1000:
	    data.append(random.randint(0,100))
		#time.sleep(0.05)
	unique = [(g[0], len(list(g[1]))) for g in itertools.groupby(data)]
	print(" Measurements: "+str(len(data)))
	print(" Data: ")
	print(unique)

interface.terminate()
