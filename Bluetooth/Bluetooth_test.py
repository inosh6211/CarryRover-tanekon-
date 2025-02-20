from ble_simple_peripheral import BLESimplePeripheral
import bluetooth
import time

ble = bluetooth.BLE()
p = BLESimplePeripheral(ble, name="RPpicoW")

if __name__ == '__main__':
    while True:
        while p.is_connected():
            p.send("Hello World")
            time.sleep(1)
