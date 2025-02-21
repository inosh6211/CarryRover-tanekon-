from ble_simple_peripheral import BLESimplePeripheral
import bluetooth
import time

ble = bluetooth.BLE()
p = BLESimplePeripheral(ble, name="RPpicoW")
time.sleep(1)

if __name__ == '__main__':
    count = 0
    while True:
        if p.is_connected():
            p.send(str(count))
            count += 1
            time.sleep(1)
