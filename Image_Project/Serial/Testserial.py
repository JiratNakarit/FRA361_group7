import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='COM18',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()
ser._rts_state()
t = [(16, 13), (1, 1)]
s = str(t).encode('utf8')
j = []

for i in s:
    if i == 32:
        # j.append(i)
        pass
    else:
        j.append(i)

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

inp = 1
while 1:
    # get keyboard input
    # inp = raw_input(">> ")
    # Python 3 users
    inp = input(">> ")
    if input == 'exit':
        ser.close()
        exit()
    else:
        # send the character to the device
        # (note that I append a \r\n carriage return and line feed to the characters - this is requested by my device)
        ser.write(j)
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1)

        if out != '':
            print(">>" + out)
