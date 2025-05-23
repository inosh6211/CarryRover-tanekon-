import machine
import time
import math
from bno055 import *
from bme280 import BME280
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))  


bno = BNO055(i2c)
bme = BME280(i2c = i2c)
def euler(axis):
    bno.iget(QUAT_DATA)
   

    w = (bno.w / 16384)
    x = (bno.x / 16384)
    y = (bno.y / 16384)
    z = (bno.z / 16384)

    ysqr = y * y

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.atan2(t0, t1);

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    if t2 > 1.0:
        t2 = 1.0
    
    elif t2 < -1.0:
        t2 = -1.0
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z) 
    yaw = math.atan2(t3, t4)

    roll *= 180/math.pi
    pitch *= 180/math.pi
    yaw *= 180/math.pi
    if axis == 0:
        return roll

def landing():
    j = 0
    
    time.sleep(0.1)
    init_roll = euler(0)
    _, pressure, _ = bme.read_compensated_data()
    pressure /= 25600
    init_pressure = pressure
    start_time = time.time()

    while True:
        time.sleep(0.1)
        current_roll = euler(0)
        _, pressure, _ = bme.read_compensated_data()
        pressure /= 25600
        current_pressure = pressure
        diff_roll = current_roll - init_roll
        diff_pressure = current_pressure - init_pressure
        init_roll = current_roll
        init_pressure = current_pressure
        
        if abs(diff_roll) < 0.1 and abs(diff_pressure) < 0.1:
          j += 1
        
        else:
          j = 0
        
        elapsed_time = (time.time() - start_time) 

        if j == 3 or elapsed_time > 30:
          print("landing")
          break
        
        print(f"roll:{diff_roll}")
        print(pressure)
        print(elapsed_time)
        
landing()
