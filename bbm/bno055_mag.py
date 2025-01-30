from machine import Pin, I2C
from bno055 import BNO055
import math
import time

i2c = I2C(0, sda = Pin(20), scl = Pin(21))
bno = BNO055(i2c)

if __name__ == '__main__':
    while True:
        time.sleep(0.5)
        mag_x, mag_y, _ = bno.mag()
        heading = math.atan2(mag_y, mag_x) * 180 / math.pi
        if heading < 0:
            heading += 360
    
        print(f'Compass Heading: {heading:.2f}°')
