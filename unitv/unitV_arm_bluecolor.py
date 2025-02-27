import sensor
import image
import time
import math
from fpioa_manager import fm
from machine import UART
import lcd

# LCD 初期化
lcd.init()
lcd.rotation(2)

# UART 設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=1024)

# カメラ初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.skip_frames(time=2000)

# ROI 設定
ROI_X, ROI_Y, ROI_W, ROI_H = 60, 40, 200, 200
FOCAL_LENGTH = 220
TAG_SIZE = 22
IMG_W, IMG_H = sensor.width(), sensor.height()
CX, CY = IMG_W // 2, IMG_H // 2

# 色認識の閾値（LAB）
color_dict = {
    "red": ((20, 130, 40, 80, 40, 80), (255, 0, 0)),
    "blue": ((14, 37, 29, 81, -110, 33), (0, 0, 255)),
    "green": ((40, 61, -66, -26, -4, 60), (0, 255, 0)),
    "yellow": ((69, 100, -21, -9, 25, 48), (255, 255, 0)),
}

# 水色の閾値（LAB）追加
WATER_BLUE_THRESHOLD = (60, 90, -40, 0, -30, 0)  # LABの適切な範囲（調整可能）

print("Waiting for request from Raspberry Pi...")

while True:
    if uart.any():  # **Raspberry Pi からの信号を待つ**
        request = uart.read().decode('utf-8').strip()

        if request == "1":
            img = sensor.snapshot()
            img.replace(vflip=False, hmirror=True, transpose=True)

            # === AprilTag 検出 ===
            roi_img = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
            tags = roi_img.find_apriltags(families=image.TAG36H11)

            if tags:
                for tag in tags:
                    tag_id = tag.id()
                    cx = tag.cx() + ROI_X
                    cy = tag.cy() + ROI_Y
                    tag_width = tag.w()
                    tag_height = tag.h()

                    img.draw_rectangle(cx - 10, cy - 10, 20, 20, color=(255, 0, 0))
                    img.draw_cross(cx, cy, color=(0, 255, 0))

                    Z = -tag.z_translation()

                    X = ((cx - CX) * Z) / FOCAL_LENGTH
                    Y = ((cy - CY) * Z) / FOCAL_LENGTH

                    pitch_rad = math.atan2(Y, Z)
                    pitch_deg = math.degrees(pitch_rad)

                    #yaw_rad = math.atan2(X, Z)
                    #yaw_deg = math.degrees(pitch_rad)

                    # **AprilTag のデータを UART で送信**
                    tag_message = "TAG,{},{},{},{:.2f},{:.2f}".format(tag_id, cx, cy, Z, pitch_deg)
                    uart.write(tag_message + "\n")
                    print(tag_message)
            # 追加
            water_blue_blobs = img.find_blobs([WATER_BLUE_THRESHOLD], pixels_threshold=10, area_threshold=10, merge=True)
            water_blue_pixels = sum(blob.pixels() for blob in water_blue_blobs)

            # ピクセル数を送信
            pixel_message = "BLUE_PIXELS,{}".format(water_blue_pixels)
            uart.write(pixel_message + "\n")
            print(pixel_message)

            # === 色認識（物資検出） ===
            """for color_name, (threshold, draw_color) in color_dict.items():
                blobs = img.find_blobs([threshold], pixels_threshold=200, area_threshold=200, merge=True)
                if blobs:
                    largest_blob = max(blobs, key=lambda b: b.pixels())
                    x, y, w, h = largest_blob.rect()
                    cx_color = x + w // 2
                    cy_color = y + h // 2

                    img.draw_rectangle(largest_blob.rect(), color=draw_color)
                    img.draw_cross(cx_color, cy_color, color=(255, 255, 255))
                    img.draw_string(x + 5, y, color_name, color=(255, 255, 255))

                    # **色認識データを UART で送信**
                    color_message = "COLOR,{},{},{}".format(color_name, cx_color, cy_color)
                    uart.write(color_message + "\n")
                    print(color_message)"""

            lcd.display(img)  # 画像をLCDに表示
            time.sleep(0.1)  # 送信間隔を調整
