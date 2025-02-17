from machine import Pin, I2C
from bno055 import *
from bme280 import BME280
import math
import time

class BNO055Handler:
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno055 = BNO055(self.i2c)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0
        
    def get_calibration_status(self):
        self.sys, self.gyro, self.accel, self.mag = self.bno055.cal_status()
        return self.bno055.calibrated()
        """BNO055を完全に静止させる。
           BNO055を6方向に向けて静止させる
           BNO055を空中で8の字に回転させる"""
        
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

def landing():
    count = 0
    bno.compute_euler()
    init_roll = bno.roll
    init_pressure = bme.pressure()
    start_time = time.time()

    while True:
        time.sleep(1)
        bno.compute_euler()
        current_roll = bno.roll
        current_pressure = bme.pressure()
        diff_roll = current_roll - init_roll
        diff_pressure = current_pressure - init_pressure
        init_roll = current_roll
        init_pressure = current_pressure
        
        if abs(diff_roll) < 0.1 and abs(diff_pressure) < 0.1:
          count += 1
        
        else:
          count = 0
        
        now = time.time()
        elapsed_time = (now - start_time) 
        if count == 3 or elapsed_time > 30:
            print("landing")
            break
        
        print(f"diff_roll:{diff_roll}, diff_pressure:{diff_pressure}")

if __name__ == '__main__':
    i2c_0 = I2C(0, sda=Pin(20), scl=Pin(21))
    bno = BNO055Handler(i2c=i2c_0)
    bme = BME280(i2c=i2c_0)

    landing()
