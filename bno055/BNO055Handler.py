from machine import Pin, I2C
from bno055 import *
import math
import time

I2C0 = I2C(0, sda=Pin(20), scl=Pin(21))


class BNO055Handler:
    """
    [キャリブレーション]
    BNO055を完全に静止させる。
    BNO055を空中で8の字に回転させる
    """
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno055 = BNO055(self.i2c)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0
        
        while True:
            self.sys, self.gyro, self.accel, self.mag = self.bno055.cal_status()
            
            if self.gyro == 3 and self.mag == 3:
                print("BNO055 calibration completed")
                break
            
            else:
                print(f"Calibration gyro:{self.gyro}, mag:{self.mag}")
                
            time.sleep(1)
        
    def compute_euler(self):
        self.bno055.iget(QUAT_DATA)
        w = self.bno055.w / 16384
        x = self.bno055.x / 16384
        y = self.bno055.y / 16384
        z = self.bno055.z / 16384

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        yaw = math.atan2(t3, t4)

        self.roll = roll * 180 / math.pi
        self.pitch = pitch * 180 / math.pi
        self.yaw = yaw * 180 / math.pi

    def compute_heading(self):
        mag_x, mag_y, _ = self.bno055.mag()
        self.heading = math.atan2(mag_y, mag_x) * 180 / math.pi
        if self.heading < 0:
            self.heading += 360


if __name__ == '__main__':
    bno = BNO055Handler(i2c=I2C0)

    while True:
        bno.compute_euler()
        bno.compute_heading()
        print(f"roll:{bno.roll}, pitch:{bno.pitch}, yaw:{bno.yaw}")
        print(f"heading:{bno.heading}")

        time.sleep(0.1)
