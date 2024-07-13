import serial
import time

ser = serial.Serial('/dev/cu.usbmodem144301', 9600)


time.sleep(1)
ser.write('angle10;'.encode())
time.sleep(1)
ser.write('speed+255;'.encode())
ser.write('angle00;'.encode())
time.sleep(1)
ser.write('angle10;'.encode())
time.sleep(0.1)
ser.write('angle20;'.encode())
time.sleep(2)
ser.write('angle00;'.encode())
time.sleep(0.8)
ser.write('angle10;'.encode())
time.sleep(1)
ser.write('speed+000;'.encode())

