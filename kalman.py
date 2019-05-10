import serial
import numpy as np


# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT = 'COM5'
# be sure to set this to the same rate used on the Arduino
SERIAL_RATE = 115200


def main():
    global measurements 
    ser = serial.Serial("COM5", baudrate = 115200)
    while True:
         #using ser.readline() assumes each line contains a single reading
         #sent using Serial.println() on the Arduino
        reading = ser.readline().decode('utf-8')
        measurements = reading.split('_')
        # reading is a string...do whatever you want from here
        print(reading)
    
class kalman:
    def __init__(self):
       
        #Initilizing State 
        self.x = np.matrix([[0],
                            [0],
                            [0],
                            [0],
                            [0],
                            [0]])
        
        self.P = np.matrix([[1000, 0., 0., 0., 0., 0.],
                            [0., 1000., 0., 0., 0., 0.],
                            [0., 0., 1000., 0., 0., 0.],
                            [0., 0., 0., 1000., 0., 0.],
                            [0., 0., 0., 0., 1000., 0.],
                            [0., 0., 0., 0., 0., 1000.]])
        
        self.F = np.matrix([[1., 0., 0., 0.01, 0., 0.],
                            [0., 1, 0., 0., 0.01, 0.],
                            [0., 0., 1., 0., 0., 0.01],
                            [0., 0., 0., 1., 0., 0.],
                            [0., 0., 0., 0., 1., 0.],
                            [0., 0., 0., 0., 0., 1.]])
        
        self.H = np.matrix([[1., 0., 0., 0., 0., 0.],
                            [0., 1., 0., 0., 0., 0.],
                            [0., 0., 1., 0., 0., 0.]])
        
        self.R = np.matrix([[0.5, 0., 0.],
                            [0., 0.5, 0.],
                            [0., 0., 0.5]])
        
        self.I = np.matrix([[1, 0., 0., 0., 0., 0.],
                            [0., 1., 0., 0., 0., 0.],
                            [0., 0., 1., 0., 0., 0.],
                            [0., 0., 0., 1., 0., 0.],
                            [0., 0., 0., 0., 1., 0.],
                            [0., 0., 0., 0., 0., 1.]])
        
        
        
    def predict(self, dt):
        self.P[0,0] += 0.1
        self.P[1,1] += 0.1
        self.P[2,2] += 0.1
        self.P[3,3] += 0.1
        self.P[4,4] += 0.1
        self.P[5,5] += 0.1
        self.x = self.F * self.x + self.u
        self.P = self.F * self.P * np.transpose(self.F)
        return
        
    def update(self, measurements, dt):
        Z = np.matrix(measurements)
        y = np.transpose(Z) - (self.H * self.x)
        S = (self.H * self.P * np.transpose(self.H)+ self.R)
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)
        
        self.x = self.x + (K * y)
        self.P = (self.I - (K * self.H)) * self.P
        return [self.x[0], self.x[1]]
        
if __name__ == "__main__":
    main()
    kalmanfil = kalman()
    kalmanfil.predict(10)
    kalmanfil.update(measurements, 10)