import sensor
import image
import time
from machine import UART
from Maix import GPIO
from fpioa_manager import fm
from modules import ws2812

# LEDの設定
led = ws2812(8, 1)
led.set_led(0, (0, 255, 0))
led.display()

#　UARTの設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)

# カメラの初期設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

# 赤色の閾値（LAB）
red_threshold = (20, 130, 40, 80, 40, 80)

while True:
    if uart.any()> 0 :
        uart.read()
        img = sensor.snapshot()
        img.replace(vflip=False, hmirror=True, transpose=True)
        blobs = img.find_blobs([red_threshold], pixels_threshold=10, area_threshold=10, merge=True, margin=10)

        pixels = 0
        centers = []
        data = []
        objects = 0

        if blobs:
            for blob in blobs:
                objects += 1
                pixels = blob.pixels()
                cx, cy = blob.cx(), blob.cy()
                centers.append((cx, cy))

                img.draw_rectangle(blob.rect(), color=(255, 0, 0))
                img.draw_cross(cx, cy, color=(0, 255, 0))
                img.draw_string(cx + 5, cy, "({}, {})".format(cx, cy), color=(255, 255, 255))

                data.append("{},{},{}".format(pixels, cx, cy))

                message = "${},".format(objects) + ",".join(data) + "\n"
        else:
            message = "$0\n"

        uart.write(message)
