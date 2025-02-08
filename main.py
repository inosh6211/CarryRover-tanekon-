import bluetooth
from ble_simple_peripheral import BLESimplePeripheral
import time

ble = bluetooth.BLE()
p = BLESimplePeripheral(ble)

def on_rx(v):
    print("RX", v)

p.on_write(on_rx)

i = 0
while True:
    if p.is_connected():
        # Short burst of queued notifications.
        for _ in range(3):
            data = str(i) + "_"
            print("TX", data)
            p.send(data)
            i += 1
    time.sleep(0.1)
