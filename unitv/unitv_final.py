import sensor
import image
import time
import math
from fpioa_manager import fm
from machine import UART
import lcd

# LCD初期化
lcd.init()
lcd.rotation(2)

# UART設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=1024)

# カメラ初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  
sensor.set_framesize(sensor.QVGA)    # 320x240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

# **ROI（関心領域）を中央部分に設定**
ROI_X, ROI_Y, ROI_W, ROI_H = 80, 60, 160, 120  # 64K ピクセル以下

# **色認識の閾値（LAB）**
color_dict = {
    "red": ((20, 130, 40, 80, 40, 80), (255, 0, 0)),
    "orange": ((30, 79, 102, 13, -12, 78), (255, 165, 0)),
    "pink": ((75, 97, 28, 7, -9, 8), (255, 192, 203)),
    "green": ((42, 63, -82, -49, 32, 103), (0, 255, 0))
}

while True:
    img = sensor.snapshot()
    img.replace(vflip=False, hmirror=True, transpose=True)

    # **AprilTag 検出 (ROIのみ)**
    roi_img = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
    tags = roi_img.find_apriltags(families=image.TAG36H11)

    if tags:
        for tag in tags:
            id = tag.id()
            cx = tag.cx() + ROI_X
            cy = tag.cy() +ROI_Y
            tag_width = tag.w()
            tag_height = tag.h()
            
            img.draw_rectangle(cx - 10, cy - 10, 20, 20, color=(255, 0, 0))
            img.draw_cross(cx, cy, color=(0, 255, 0))

             # Z の計算（三角測量）
            if tag_width > 0:
                Z = (FOCAL_LENGTH * TAG_SIZE) / tag_width  # mm単位
            else:
                Z = -1  # 検出エラー

            message = "Apriltag, ID:{}, X:{}, Y:{}".format(id, cx, cy)
            uart.write(message + "\n")
            print(message)  # コンソール出力

    else:
        print("No AprilTag detected")
        uart.write("Apriltag, None\n")

    # **色認識（画像全体）**
    detected_objects = []
    for color_name, (threshold, draw_color) in color_dict.items():
        blobs = img.find_blobs([threshold], pixels_threshold=200, area_threshold=200, merge=True)
        if blobs:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            x, y, w, h = largest_blob.rect()
            img.draw_rectangle(largest_blob.rect(), color=draw_color)
            img.draw_cross(x + w // 2, y + h // 2, color=(255, 255, 255))
            img.draw_string(x + 5, y, color_name, color=(255, 255, 255))

            detected_objects.append("{}: X={}, Y={}".format(color_name, x + w // 2, y + h // 2))

    if detected_objects:
        message = "Color detected: " + ", ".join(detected_objects)
        uart.write(message + "\n")
        print(message)  # コンソール出力
    else:
        print("No color detected")
        uart.write("Color, None\n")

    # **LCD に画像を表示**
    lcd.display(img)
    time.sleep(0.1)
