import serial
import time
import cv2

# initialization and open the port

# possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

ser = serial.Serial()
# ser.port = "/dev/ttyUSB0"
# ser.port = "/dev/ttyUSB7"
ser.port = "COM19"
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser.parity = serial.PARITY_NONE  # set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
# ser.timeout = None          #block read
ser.timeout = None  # non-block read
# ser.timeout = None  # timeout block read
ser.xonxoff = False  # disable software flow control
ser.rtscts = False  # disable hardware (RTS/CTS) flow control
ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control
# ser.writeTimeout = 2  # timeout for write
# ser.readTimeout = 2  # timeout for write
t = '[3]\r\n'
s = t.encode('utf8')
j = []
try:
    ser.open()
except Exception:
    print("error open serial port: ")
    exit()

if ser.isOpen():

    try:
        # and discard all that is in buffer

        ser.setRTS(True)
        ser.setDTR(True)
        time.sleep(1)
        ser.setRTS(False)
        ser.setDTR(False)

        ser.flushInput()  # flush input buffer, discarding all its contents
        ser.flushOutput()  # flush output buffer, aborting current output

        data = s
        ser.write(bytearray(b'\x91\x51\x93'))
        print(ser.out_waiting)
        print("write data: ", s)

        time.sleep(0.1)
        print("waiting")
        while ser.in_waiting > 0:
            response = ser.read()

            if 32 < ord(response) < 128:
                print("read data: ", response)
                j.append(response)
        print(j)

        ser.close()
    except Exception:
        print("error communicating...: ")
