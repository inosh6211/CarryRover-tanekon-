from machine import Pin, SPI
import os, sdcard

FILE_NAME = "CarryRover"

def create_file():
    counter = 0
    while True:
        file_name = f"/sd/{FILE_NAME}{counter}.txt"
        try:
            with open(file_name, "x") as f:
                print(f"ファイル作成: {file_name}")
                return file_name
            
        except OSError as e:
            counter += 1

if __name__ == "__main__":
    spi1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
    sd = sdcard.SDCard(spi1, Pin(13))
    
    try:
        os.mount(sd, "/sd")
        new_file = create_file()
        
    except Exception as e:
        print("SDカードが認識できません")
