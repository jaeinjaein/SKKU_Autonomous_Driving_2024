import serial

t = serial.Serial('/dev/cu.usbmodem144301', 9600)

while True:
    t.write(('angle%02d;' % int(input())).encode())

