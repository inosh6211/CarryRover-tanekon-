from machine import I2C, Pin
from servo import Servos
import utime

i2c = I2C(1, scl=Pin(27), sda=Pin(26))
servos = Servos(i2c)

def moveServo():
    for angle in range(70, 20, -1):
        servos.position(0, degrees=angle)
        utime.sleep_ms(10)
        print("a")

if __name__ == '__main__':
    moveServo()
