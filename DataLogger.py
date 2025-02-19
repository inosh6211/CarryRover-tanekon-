from machine import Pin, SPI
from ble_simple_peripheral import BLESimplePeripheral
import os
import sdcard
import bluetooth
import time

DEVICE_NAME = "RPpicoW"
FILE_NAME = "CarryRover"

SPI1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
SPI1_CS = Pin(13, Pin.OUT)

class DataLogger:
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
        
    def create_file(self):
        counter = 0   
        while counter < 1000:
            file_name = f"/sd/{FILE_NAME}{counter}.txt"    
            try:
                with open(file_name, "x") as f:
                    print(f"File created: {file_name}")
                    self.file_name = file_name
                    return file_name
                
            except OSError:
                counter += 1  

        print("Error: Failed to create a log file after multiple attempts.")
        return None

    def write(self, sentence):
        print(sentence)
        
        if self.ble.is_connected():
            self.ble.send(sentence)
        
        if self.sd_state:
            try:
                with open(self.file_name, "a") as f:
                    f.write(sentence + "\n")
            except OSError as e:
                self.sd_state = False
                print(f"SD card write error: {e}")

if __name__ == "__main__":
    log = DataLogger(SPI1, SPI1_CS)
    
    while True:
        log.write("Hello World")
        time.sleep(1)
