# import serial
# import time
#
#
# class SerialPIC(object):
#     """docstring for RecieveParkinson"""
#
#     def __init__(self, port='COM11', brate=115200):
#         self.serial = serial.Serial(port, brate, timeout=3)
#         self.serial.flush()
#         # self.serial.dtr = False
#         # self.serial.rst = False
#         self.serial.setDTR(False)
#         self.serial.setRTS(False)
#         self.Package = []
#
#     def SEND(self, package=[]):
#         self.serial.write(package)
#
#     def recieveRawPackage(self):
#         DATA = []
#         while True:
#             if self.serial.inWaiting() > 0:
#                 for i in range(0, self.serial.inWaiting()):
#                     DATA.append(ord(self.serial.read()))
#                 break
#         return DATA
#
#     def recieveRawPackage1(self):
#         DATA = []
#         flagNotRecieve = 0
#         while self.serial.inWaiting():
#             if flagNotRecieve == 0:
#                 flagNotRecieve = 1
#             DATA.append(ord(self.serial.read()))
#         # print self.serial.inWaiting()
#         if flagNotRecieve:
#             flagNotRecieve = 0
#             return DATA
#
#     def recieveRawPackage2(self):
#         DATA = []
#         flagNotRecieve = 0
#         while self.serial.inWaiting():
#             if flagNotRecieve == 0:
#                 flagNotRecieve = 1
#             DATA.append(ord(self.serial.read()))
#         # print self.serial.inWaiting()
#         if flagNotRecieve:
#             flagNotRecieve = 0
#             return DATA
#
#     def closePort(self):
#         self.serial.close()
#
#
# if __name__ == '__main__':
#
#     # p_ax = []
#     # p_ay = []
#     recivePIC = SerialPIC(port="COM19", brate=115200)
#     "[h]"
#     recivePIC.SEND(package=[91, 104, 93])
#     print("Send!")
#     while True:
#         # print(recivePIC.recieveRawPackage())
#         print(recivePIC.recieveRawPackage1())
#         # print(recivePIC.recieveRawPackage2())

my_list = ['apple', 'banana', 'grapes', 'pear']
counter_list = list(enumerate(my_list))
print(counter_list)