#モジュールの読み込み（インポート）
from machine import Pin, I2C

#I2Cを利用するため、オブジェクト（i2c）を作成
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=400000)

#I2Cのアドレス確認（16進数）
for addr in i2c.scan():   
    print("I2C 16進法アドレス: ", hex(addr))
