from machine import Pin, SPI
from ble_simple_peripheral import BLESimplePeripheral
import os
import time
import bluetooth
import sdcard
import uasyncio

DEVICE_NAME = "RPpicoW"
FILE_NAME = "CarryRover"
SPI1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
SPI1_CS = Pin(13, Pin.OUT)

class Logger:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.sd = None
        self.file_name = None
        self.sd_state = False
        self.ble = BLESimplePeripheral(bluetooth.BLE(), name=DEVICE_NAME)
        
        self.log_queue = []
        
        while not self.ble.is_connected():
            print("Waiting for Bluetooth connection...")
            time.sleep(1)
        print("Bluetooth connected")
        time.sleep(1)
        
        try:
            self.sd = sdcard.SDCard(self.spi, self.cs)
            os.mount(self.sd, "/sd")
            self.create_file()
            self.sd_state = True
        except Exception as e:
            print(f"SD card not detected: {e}")
            if self.ble.is_connected():
                self.ble.send(f"SD card not detected: {e}")
        time.sleep(1)
        
        uasyncio.create_task(self.logging_task())
    
    def create_file(self):
        counter = 0
        while True:
            file_name = f"/sd/{FILE_NAME}{counter}.txt"
            try:
                with open(file_name, "x") as f:
                    print(f"File created: {FILE_NAME}{counter}.txt")
                    if self.ble.is_connected():
                        self.ble.send(f"File created: {FILE_NAME}{counter}.txt")
                    self.file_name = file_name
                    return file_name
            except OSError:
                counter += 1
    
    async def logging_task(self):
        while True:
            if self.log_queue:
                message = self.log_queue.pop(0)
                self.write_log(message)
            else:
                await uasyncio.sleep(0.01)
    
    def write_log(self, message):
        print(message)
        if self.ble.is_connected():
            self.ble.send(message)
        if self.sd_state:
            try:
                with open(self.file_name, "a") as f:
                    f.write(message + "\n")
            except OSError as e:
                self.sd_state = False
                print(f"SD card write error: {e}")
    
    def message(self, message):
        self.log_queue.append(message)


async def main():
    log = Logger(SPI1, SPI1_CS)
    count = 0
    while True:
        log.message("Hello World " + str(count))
        print("Main ", count)
        count += 1
        await uasyncio.sleep(0.1)


if __name__ == '__main__':
    uasyncio.run(main())
