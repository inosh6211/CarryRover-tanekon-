from machine import I2C, Pin
from servo import Servos
import time

i2c = I2C(1, scl=Pin(27), sda=Pin(26))
servos = Servos(i2c)

if __name__ == '__main__':
    servos.position(0, degrees = 0)
    servos.position(1, degrees = 0)
    servos.position(2, degrees = 180)
    servos.position(3, degrees = 0)
    servos.position(4, degrees = 0)
