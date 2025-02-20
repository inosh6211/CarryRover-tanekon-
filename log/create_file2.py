from machine import Pin, SPI
import os
import sdcard
import time

FILE_NAME = "CarryRover"

SPI1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
SPI1_CS = Pin(13, Pin.OUT)

def create_file():
    counter = 0
    while True:
        file_name = f"/sd/{FILE_NAME}{counter}.txt"
        try:
            with open(file_name, "x") as f:
                print(f"File created: {FILE_NAME}{counter}.txt")
                return file_name
            
        except OSError:
            counter += 1

if __name__ == "__main__":
    try:
        sd = sdcard.SDCard(SPI1, SPI1_CS)
        os.mount(sd, "/sd")
        file = create_file()
        sd_state = True

        with open(file, "a") as f:
            f.write("Hello World!")
        
    except Exception as e:
        print(f"SD card not detected: {e}")
