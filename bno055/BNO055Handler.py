from machine import Pin, I2C
from bno055 import *
import math
import time

class BNO055Handler:
    def __init__(self, i2c, sda, scl):
        self.i2c = I2C(i2c, sda=Pin(sda), scl=Pin(scl))
        self.bno = BNO055(self.i2c)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0
        
    def get_calibration_status(self):
        self.sys, self.gyro, self.accel, self.mag = self.bno.cal_status()
        return self.bno.calibrated()
        """BNO055を完全に静止させる。
           BNO055を6方向に向けて静止させる
           BNO055を空中で8の字に回転させる"""
        
    def compute_euler(self):
        self.bno.iget(QUAT_DATA)
        w = self.bno.w / 16384
        x = self.bno.x / 16384
        y = self.bno.y / 16384
        z = self.bno.z / 16384

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
        mag_x, mag_y, _ = self.bno.mag()
        self.heading = math.atan2(mag_y, mag_x) * 180 / math.pi
        if self.heading < 0:
            self.heading += 360

if __name__ == '__main__':
    imu = BNO055Handler(i2c=0, sda=20, scl=21)

    while True:
        imu.compute_euler()
        imu.compute_heading()
        print(imu.get_calibration_status())
        print(f"Calibration Status sys:{imu.sys}, gyro{imu.gyro}, accel:{imu.accel}, mag:{imu.mag}")
        print(f"Roll: {imu.roll:.2f}°")
        print(f"Pitch: {imu.pitch:.2f}°")
        print(f"Yaw: {imu.yaw:.2f}°")
        print(f"Compass Heading: {imu.heading:.2f}°")

        time.sleep(0.1)
