class pidParameters():
    def __init__(self):
        self.minOutput = 0
        self.minOutput = 0
        self.k_p = 0
        self.k_i = 0
        self.k_d = 0

class MotorParams():
    
    def __init__(self):
        self.maxRotationAcceleration = 0
        self.maxRotationSpeed = 0
        self.feedForwardGain = 0
        self.minPWM = 0
        self.pidParameters = pidParameters()

class Interface:
    
    def __init__(self):
        
        self.sensors = {}
        pass
    
    def initialize(self):
        return True
    
    def motorEnable(self,port):
        return True
    
    def MotorAngleControllerParameters(self):
        return MotorParams()
    
    def setMotorAngleControllerParameters(self,port, params):
        return True
    
    def setMotorRotationSpeedReference(self,motors, speeds):
        return True
    
    def sensorEnable(self,port, sensor_type):
        self.sensors[port] = sensor_type
        return True

    def getSensorValue(self,port):
        if self.sensors[port] == "SENSOR_ULTRASONIC":
            # Insert some random function here
            return (10,0)
        else:
            return (0,0)
    
    def getMotorAngles(self,ports):
        return [(1,0) for x in ports]
    
    def increaseMotorAngleReferences(self,ports, distances):
        return 1
    
    def motorAngleReferencesReached(self,ports):
        return 1
    
    def setMotorRotationSpeedReferences(self,ports,speeds):
        return True
    
    def setMotorPwm(self,port, value):
        return True
    
    def startLogging(self,logfile):
        print("Logging started.")
        return True
     
    def stopLogging(self):
        print("Logging stopped.")
        return True
     
    def terminate(self):
        print("Shutting down robot.")
        return True
