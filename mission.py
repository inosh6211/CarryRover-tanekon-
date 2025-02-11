import bluetooth
import machine
import time
import math
from bno055 import *
from bme280 import BME280
import struct
from ble_simple_peripheral import BLESimplePeripheral
from ble_advertising import advertising_payload
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))  


bno = BNO055(i2c)
bme = BME280(i2c = i2c)
ble = bluetooth.BLE()

def euler():
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
    return roll, pitch, yaw

def euler_a(axis):
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

ble_service = BLESimplePeripheral(ble)


def start():
    _, pressure, _ = bme.read_compensated_data()
    pressure /= 25600
    init_pressure = pressure
    while True:
        time.sleep(0.1)
        roll, pitch, yaw = euler()
        _, pressure, _ = bme.read_compensated_data()
        pressure /= 25600
        current_pressure = pressure
        diff_pressure = current_pressure - init_pressure
        if abs(roll) > 45 and abs(roll) < 135 and diff_pressure < -0.5:
            print("start")
            password_data = "start success" + "\n"
            ble_service.send(password_data)
            break
        else:
            print(f"roll:{roll}")
            print(pressure)
            roll_data = str(roll).encode('utf-8') + "\n"
            ble_service.send(roll_data)
            pressure_data = str(pressure).encode('utf-8') + "\n"
            ble_service.send(pressure_data)  # データを送信
    
            
def released():
    j = 0

    while True:
        time.sleep(0.1)
        roll, pitch, yaw = euler()

        if abs(roll) < 45: 
          j += 1
        else: 
          j = 0
          
        if j >= 5:
            print("released")
            passworda_data = "released success" + "\n"
            ble_service.send(passworda_data)
            break
        print(f"roll:{roll}")
        roll_data = str(roll).encode('utf-8') + "\n"
        ble_service.send(roll_data)
            

def landing():
    j = 0
    
    time.sleep(0.1)
    init_roll = euler_a(0)
    _, pressure, _ = bme.read_compensated_data()
    pressure /= 25600
    init_pressure = pressure
    start_time = time.time()

    while True:
        time.sleep(0.1)
        current_roll = euler_a(0)
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
          passwordb_data = "landing success" + "\n"
          ble_service.send(passwordb_data)
          break
        
        print(f"roll:{diff_roll}")
        print(pressure)
        print(elapsed_time)
        roll_data = str(diff_roll).encode('utf-8') + "\n"
        ble_service.send(roll_data)
        pressure_data = str(pressure).encode('utf-8') + "\n"
        ble_service.send(pressure_data)  # データを送信


start()
released()
landing()
