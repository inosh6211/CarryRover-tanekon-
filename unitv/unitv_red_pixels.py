import sensor
import image
import time
from modules import ws2812
from machine import UART
from fpioa_manager import fm
from Maix import GPIO

#　LEDの設定
led = ws2812(8, 1)
led.set_led(0, (0, 255, 0))
led.display()

# UARTの設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)

# カメラの初期設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

# 赤色の閾値（LAB）
red_threshold = (20, 130, 15, 127, 15, 127)

while True:
    img = sensor.snapshot()
    img.replace(vflip = False, hmirror = True, transpose = True)
    blobs = img.find_blobs([red_threshold])
    red_pixels = 0

    if blobs:
        for blob in blobs:
            red_pixels += blob.pixels()
        print("赤色: {} ピクセル".format(red_pixels))
    else:
        print("なし")

    time.sleep(0.1)
