import serial
from time import sleep
#init serial port and bound
# bound rate on two ports must be the same
ser = serial.Serial('COM4', 115200)
print(ser.portstr)

#send data via serial port
ser.write(str.encode("START\r"))
while True:
    sleep(0.2)
    ser.write(str.encode("ABC010\r"))

ser.close()