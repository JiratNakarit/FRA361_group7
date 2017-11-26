import serial
import time


class SerialPIC(object):
    """docstring for RecieveParkinson"""

    def __init__(self, port='COM19', brate=115200):
        self.serial = serial.Serial(port, brate, timeout=5)
        self.serial.flush()
        # self.serial.dtr = False
        # self.serial.rst = False
        self.serial.setDTR(False)
        self.serial.setRTS(False)
        self.Package = []

    def SEND(self, package=[]):
        self.serial.write(package)

    def recieveRawPackage(self):
        DATA = []
        while (True):
            if (self.serial.inWaiting() > 0):
                for i in range(0, self.serial.inWaiting()):
                    DATA.append(ord(self.serial.read()))
                break
        return DATA

    def recieveRawPackage1(self):
        DATA = []
        flagNotRecieve = 0
        while self.serial.inWaiting():
            if flagNotRecieve == 0:
                flagNotRecieve = 1
            DATA.append(ord(self.serial.read()))
        # print self.serial.inWaiting()
        if flagNotRecieve:
            flagNotRecieve = 0
            return DATA

    def recieveRawPackage2(self):
        DATA = []
        flagNotRecieve = 0
        while self.serial.inWaiting():
            if flagNotRecieve == 0:
                flagNotRecieve = 1
            DATA.append(ord(self.serial.read()))
        # print self.serial.inWaiting()
        if flagNotRecieve:
            flagNotRecieve = 0
            return DATA

    def closePort(self):
        self.serial.close()


if __name__ == '__main__':
    ser = SerialPIC()
    Jimmy = "[uk]"
    time.sleep(0.3)
    for i in Jimmy:
        ser.Package.append(ord(i))
    ser.SEND(ser.Package)
    print(ser.Package)
    ser.Package = []
    time.sleep(0.3)
    j = ser.recieveRawPackage()
    for o in j:
        print(chr(o))
    print('Next!!')
