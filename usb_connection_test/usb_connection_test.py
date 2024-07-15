# pip install pyserial

import serial_test

ser = serial_test.Serial(
    port='COM6',\
    baudrate=115200,\
    parity=serial_test.PARITY_NONE,\
    stopbits=serial_test.STOPBITS_ONE,\
    bytesize=serial_test.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8')
        print(line, end='')
        
ser.close()