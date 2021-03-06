import brickpi
import time
import numpy as np

interface=brickpi.Interface()
interface.initialize()

port = 3

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

while True:
	distance = int(input("What is the distance being measured?"))
	data = []
	for i in range(10):
		usReading = interface.getSensorValue(port)
		if usReading and usReading[0]!=255:
			data.append(usReading[0])
		else:
			print "Failed US reading"
		time.sleep(0.05)
	data = np.array(data)
	print("Mean: "+str(np.mean(data))+
		  " Variance: "+str(np.var(data))+
		  " Standard deviation: "+str(np.std(data)))

interface.terminate()
