import sensor
import image
import lcd
import time
import utime
from modules import ws2812
from machine import UART
from fpioa_manager import fm
from Maix import GPIO


lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
#sensor.set_vflip(True) # 上下反転
sensor.set_hmirror(True) # 左右反転
sensor.set_auto_gain(False)         # オートゲインをオフ
#sensor.set_auto_whitebal(0)         # オートホワイトバランスをオフ
#sensor.skip_frames(time=2000)       # 2秒間フレームをスキップして安定化
sensor.set_windowing((320,120))
sensor.run(1)

class_ws2812 = ws2812(8, 100)
BRIGHTNESS = 0x10

fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)

fm.register(19, fm.fpioa.GPIO1)
ButonA = GPIO(GPIO.GPIO1, GPIO.IN, GPIO.PULL_UP)
fm.register(18, fm.fpioa.GPIO2)
ButonB = GPIO(GPIO.GPIO2, GPIO.IN, GPIO.PULL_UP)

# RGB LEDを光らせる
def RGB_LED(r,g,b):
    a = class_ws2812.set_led(0,(r,g,b))
    a = class_ws2812.display()
    time.sleep(0.3)
    a = class_ws2812.set_led(0,(0,0,0))
    a = class_ws2812.display()
    time.sleep(0.3)
    a = class_ws2812.set_led(0,(r,g,b))
    a = class_ws2812.display()

# 起動インジケータとしてblueで点滅、最後に点灯
RGB_LED(0,0,BRIGHTNESS)
RGB_LED(0,0,BRIGHTNESS)
RGB_LED(0,0,BRIGHTNESS)

uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)

# 赤色のしきい値（色相・彩度・明度）
red_threshold = (20, 130, 15, 127, 15, 127)

# 画像の総ピクセル数を定数として定義
IMAGE_TOTAL_PIXELS = 640 * 480  # 画像の総ピクセル数を定数として定義

while False:
    uart.write('TEST\n')
    utime.sleep_ms(100)

while True:
    img = sensor.snapshot()

    if ButonA.value() == 0:
        class_ws2812.set_led(0, (50, 0, 10))
        class_ws2812.display()
        #time.sleep(0.5)

    if ButonB.value() == 0:
        class_ws2812.set_led(0, (0, 50, 50))
        class_ws2812.display()
       # time.sleep(0.5)

    # 赤色のピクセルを検出
    blobs = img.find_blobs([red_threshold])

    # 画像全体のピクセル数
    total_pixels = img.width() * img.height()

    # 赤色のピクセル数をカウント
    red_pixels = 0

    # 赤色のピクセルを数える
    for blob in blobs:
        # 各blobの範囲内のピクセルを確認
        for x in range(blob[0], blob[2]):
            for y in range(blob[1], blob[3]):
                # 赤色範囲内かどうかを確認
                pixel = img.get_pixel(x, y)
                r, g, b = pixel[0], pixel[1], pixel[2]
                if r > g and r > b:  # 赤色が他の色より強い場合
                    red_pixels += 1

                   # 検出した赤色領域に四角形を描画
                img.draw_rectangle(blob[0:4], color=(255, 0, 0))



    # 赤色なら１を表示
    if red_pixels>50:
        uart.write("1")
    else:
        uart.write("0")
        utime.sleep_ms(100)

    time.sleep(0.1)

