import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

port = 2 # port which ultrasoic sensor is plugged in to

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC);

while True:
	usReading = interface.getSensorValue(port)

	if usReading :
		print usReading
	else:
		print "Failed US reading"

	time.sleep(0.05)

interface.terminate()
