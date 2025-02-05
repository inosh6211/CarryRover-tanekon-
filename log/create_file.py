from machine import Pin, SPI
import os, sdcard

spi = SPI(1,sck=Pin(10), mosi=Pin(11), miso=Pin(12))#SPIの設定
sd = sdcard.SDCard(spi, Pin(13))
os.mount(sd, '/sd')

def create_file(file_name):#ファイル名を作成
    base_name = f"/sd/{file_name}.txt"
    counter = 0
    
    while True:
        try:
            os.stat(base_name)#ファイルが存在する場合
            base_name = f"/sd/{file_name}{counter}.txt"
            counter += 1
            return base_name
        except Exception as error:
            print(f"Error: {error}")
            base_name = f"/sd/{file_name}{counter}.txt"
            return base_name
            break

file_name = "log_test"
new_file_name = create_file(file_name)

try:
    fp = open(new_file_name, 'w')#ファイルが存在しない場合は新規作成
    fp.write('data_name')#ファイルに書き込み
    fp.write(',')
    fp.write('data')

    fp.close()#ファイルを閉じる
    print(f"file succesfully created: {new_file_name}")
except Exception as error:
    print(f"Error: {error}")
    
