from machine import Pin, SPI
import os, sdcard

spi = SPI(1,sck=Pin(10), mosi=Pin(11), miso=Pin(12))
sd = sdcard.SDCard(spi, Pin(13))
os.mount(sd, '/sd')

fp = open('/sd/test.txt', 'w')#ファイルが存在しない場合は新規作成
fp.write('test')#ファイルに書き込み

fp.close()#ファイルを閉じる
