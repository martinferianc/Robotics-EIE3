import brickpi
import time
import numpy as np

interface=brickpi.Interface()
interface.initialize()

port = 3

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

while True:
	#raw_input("Measure")
	data = []
	while len(data)<1000:
		usReading = interface.getSensorValue(port)
		if usReading:
			data.append(usReading[0])
		time.sleep(0.05)
	data = np.array(data)
	unique, counts = np.unique(data, return_index = False, return_inverse = False, return_counts = True)
	print(" Measurements: " + str(len(data)))
	data = dict(zip(unique, counts))
	print(" Data: ")
	print(data)

interface.terminate()
