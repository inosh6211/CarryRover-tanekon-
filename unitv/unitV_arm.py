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
sensor.set_auto_gain(False)  # 自動ゲインをオフ
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

while True:
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

            if tag_width > 0:
                Z = (FOCAL_LENGTH * TAG_SIZE) / tag_width
            else:
                Z = -1  # エラー処理

            X = ((cx - CX) * Z) / FOCAL_LENGTH
            Y = ((cy - CY) * Z) / FOCAL_LENGTH

            pitch_rad = math.atan2(Y, Z)
            pitch_deg = math.degrees(pitch_rad)

            yaw_rad = math.atan2(X, Z)
            yaw_deg = math.degrees(yaw_rad)

            # **AprilTag のデータを UART で送信**
            tag_message = "TAG,{},{},{},{:.2f},{:.2f},{:.2f}".format(tag_id, cx, cy, Z, pitch_deg, yaw_deg)

            time.sleep(0.05)  # 送信前に少し待つ
            uart.write(tag_message + "\n")
            time.sleep(0.05)  # 送信後に待つ

            print(tag_message)  # シリアルモニタで確認用

    # === 色認識（物資検出） ===
    for color_name, (threshold, draw_color) in color_dict.items():
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

            time.sleep(0.05)
            uart.write(color_message + "\n")
            time.sleep(0.05)

            print(color_message)  # シリアルモニタで確認

    lcd.display(img)
    time.sleep(0.1)

