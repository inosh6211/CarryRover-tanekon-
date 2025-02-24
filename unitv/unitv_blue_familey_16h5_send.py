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
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
#sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)
#sensor.set_auto_exposure(True)  # 露出調整（白飛びを防ぐ）
#sensor.set_contrast(2)  # コントラストを上げる
#sensor.set_brightness(-2)  # 明るさを下げる

#ROI設定
ROI_X, ROI_Y, ROI_W, ROI_H = 60, 40, 200, 200
FOCAL_LENGTH = 250
TAG_SIZE = 20

# 色認識の閾値（LAB）
color_dict = {
    "red": ((20, 130, 40, 80, 40, 80), (255, 0, 0)),
    #"black": ((0, 15, -128, 127, -125, 127), (0, 0, 0)),
    "blue": ((14, 37, 29, 81, -110, 33), (0, 0, 255)),
    "green": ((40, 61, -66, -26, -4, 60), (0, 255, 0)),
    "yellow":((69, 100, -21, -9, 25, 48),(255,255,0)),
    #"white":((96, 100, -128, 127, -128, 127),(255,255,255))
}

while True:
    img = sensor.snapshot()
    img.replace(vflip=False, hmirror=True, transpose=True)

    # AprilTag
    roi_img = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
    tags = roi_img.find_apriltags(families=image.TAG16H5)

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
                Z = -1

            message = "{},{},{},{}".format(tag_id, cx, cy, Z)
            uart.write(message + "\n")
            print(message)


    # 色認識
    detected_objects = []
    for color_name, (threshold, draw_color) in color_dict.items():
        blobs = img.find_blobs([threshold], pixels_threshold=200, area_threshold=200, merge=True)
        if blobs:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            x, y, w, h = largest_blob.rect()
            img.draw_rectangle(largest_blob.rect(), color=draw_color)
            img.draw_cross(x + w // 2, y + h // 2, color=(255, 255, 255))
            img.draw_string(x + 5, y, color_name, color=(255, 255, 255))

            message = "Color,{},{},{}".format(color_name, x + w // 2, y + h // 2)
            uart.write(message + "\n")
            print(message)

    lcd.display(img)
    time.sleep(0.1)
