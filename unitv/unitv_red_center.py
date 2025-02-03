import sensor
import image
import time
from modules import ws2812

# LEDの設定
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
red_threshold = (20, 130, 40, 80, 40, 80)

while True:
    img = sensor.snapshot()
    img.replace(vflip=False, hmirror=True, transpose=True)
    blobs = img.find_blobs([red_threshold], pixels_threshold=10, area_threshold=10, merge=True, margin=10)

    pixels = 0
    centers = []
    object = 0

    if blobs:
        for blob in blobs:
            object += 1
            pixels += blob.pixels()
            cx, cy = blob.cx(), blob.cy()
            centers.append((cx, cy))

            # 画像に赤色の領域を描画
            img.draw_rectangle(blob.rect(), color=(255, 0, 0))
            img.draw_cross(cx, cy, color=(0, 255, 0))
            img.draw_string(cx + 5, cy, "({}, {})".format(cx, cy), color=(255, 255, 255))

            print("物体{},ピクセル数：{}, 中心座標: {}".format(object, pixels, centers))
    else:
        print("なし")

    time.sleep(0.1)
