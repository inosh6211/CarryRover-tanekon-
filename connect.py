import bluetooth
import math
import time
import bme280  # 例: BME280センサーを使っている場合
from bno055 import *
import struct
from ble_simple_peripheral import BLESimplePeripheral
from ble_advertising import advertising_payload
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21)) 
# センサーの設定
sensor = bme280.BME280(i2c = i2c)
bme = bme280.BME280(i2c = i2c)
bno = BNO055(i2c)
ble = bluetooth.BLE()
# 気圧データを取得する関数
def get_pressure():
    # センサーからデータを取得
    _, pressure, _ = bme.read_compensated_data()
    pressure /= 25600
    
    if pressure is None:
        print("Error: Failed to read pressure data.")
        return None  # エラー時にNoneを返す
    return pressure

def get_roll():
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
    if roll is None:
        print("Error: Failed to read roll data.")
        return None  # エラー時にNoneを返す
    return roll
    
ble_service = BLESimplePeripheral(ble)

# 気圧データの送信ループ
while True:
    pressure = get_pressure()  # 気圧データを取得
    if pressure is not None:
        print(f"Pressure: {pressure} hPa")
        pressure_data = str(pressure).encode('utf-8') + "\n"  # 気圧データをバイト形式に変換（float型を送信）
    
    else:
        print("Failed to get valid pressure data.")
    
    ble_service.send(pressure_data)  # データを送信
    time.sleep(0.5)  # 0.5秒毎に送信
    roll, pitch, yaw = get_roll()
    if roll is not None:
        print(f"roll: {roll}")
        roll_data = str(roll).encode('utf-8') + "\n"  # 気圧データをバイト形式に変換（float型を送信）
    
    else:
        print("Failed to get valid roll data.")
    
    ble_service.send(roll_data)  # データを送信
    time.sleep(0.5)  # 0.5秒毎に送信
    
