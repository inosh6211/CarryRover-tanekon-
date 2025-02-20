from machine import Pin, SPI
from ble_simple_peripheral import BLESimplePeripheral
import os
import time
import bluetooth
import sdcard
import _thread

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

        self.log_queue = []
        self.log_lock = _thread.allocate_lock()
        
        _thread.start_new_thread(self.logging_thread, ())

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

    def log_thread(self):
        while True:
            self.log_lock.acquire()
            if self.log_queue:
                message = self.log_queue.pop(0)
                self.log_lock.release()
                self.write_log(message)
            else:
                self.log_lock.release()
                time.sleep(0.01)

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
        self.log_lock.acquire()
        self.log_queue.append(message)
        self.log_lock.release()


if __name__ == "__main__":
    log = Logger(SPI1, SPI1_CS)
    
    while True:
        log.message("Hello World")
        time.sleep(1)
