# pip install pyserial

import serial

ser = serial.Serial(
    port='COM6',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8')
        print(line, end='')

        ser.write(b'BLUETOOTH EXEMPLO\n') # Exemplo de nome de dispositivo bluetooth

ser.close()