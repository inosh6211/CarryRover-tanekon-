import sensor
import image
import time
from modules import ws2812

#　LEDの設定
led = ws2812(8, 1)
led.set_led(0, (0, 255, 0))
led.display()

# カメラの初期設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

# 赤色の閾値（LAB）
red_threshold = (20, 130, 40, 80, 30, 80)

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
