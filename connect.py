import bluetooth
import time
import bme280  # 例: BME280センサーを使っている場合
import struct
from ble_simple_peripheral import BLESimplePeripheral
from ble_advertising import advertising_payload
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21)) 
# センサーの設定
sensor = bme280.BME280(i2c = i2c)
bme = bme280.BME280(i2c = i2c)
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
    
ble_service = BLESimplePeripheral(ble)

# 気圧データの送信ループ
while True:
    pressure = get_pressure()  # 気圧データを取得
    if pressure is not None:
        print(f"Pressure: {pressure} hPa")
        pressure_data = str(pressure).encode('utf-8') + "_"  # 気圧データをバイト形式に変換（float型を送信）
    
    else:
        print("Failed to get valid pressure data.")
    
    ble_service.send(pressure_data)  # データを送信
    time.sleep(1)  # 1秒毎に送信
