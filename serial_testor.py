import serial
from time import sleep

ser = None


def send_angle(value):
    ser.write(f'angle{value:02d};'.encode())


def send_speed(value):
    if value >= 0:
        ser.write(f'speed+{abs(value):03d};'.encode())
    else:
        ser.write(f'speed-{abs(value):03d};'.encode())


if __name__ == '__main__':
    ser = serial.Serial('/dev/cu.usbmodem1114401', 9600)
    sleep(3)
    # send_angle(0)
    # sleep(.1)
    # send_speed(50)
    # sleep(2)
    # send_angle(10)
    # sleep(1)
    # send_angle(20)
    # sleep(2.1)
    # send_angle(10)
    # sleep(1)
    # send_speed(0)
    # sleep(1)
    # send_speed(-50)
    # sleep(6.2)
    # send_speed(0)
    # sleep(0.5)

    send_speed(-50)
    sleep(3.5)
    send_speed(0)
    sleep(.1)
    send_angle(0)
    sleep(.5)
    send_speed(50)
    sleep(3)
    send_speed(0)
    sleep(1)
    send_angle(20)
    sleep(.4)
    send_speed(-50)
    sleep(9)
    send_speed(0)

    sleep(0.5)
    send_angle(10)
    sleep(0.5)
    send_speed(-50)
    sleep(2)
    send_speed(0)
