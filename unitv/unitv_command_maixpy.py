import sensor
import image
import time
import math
from fpioa_manager import fm
from machine import UART

# UART設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=1024)

# カメラ初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False,rgb_gain_db = (50, 75, 90))    # 現地調整
sensor.set_auto_exposure(-2)    # 露光(現地調整)
sensor.set_brightness(False)    # 輝度
sensor.set_contrast(False)      # コントラスト
sensor.set_saturation(False)
sensor.skip_frames(time=2000)


# 色の閾値（LAB）(現地調整)
thresholds = {
    "P": (14, 56, -73, 55, -121, 15),  # 紫
    "R": (33, 72, -41, 118, 49, 127),  # 赤
    "B": (18, 55, 7, 108, -128, -12)  # 青
}

# ROI設定
ROI_X, ROI_Y, ROI_W, ROI_H = 60, 40, 200, 200
FOCAL_LENGTH_PX = 210.70  # 焦点距離 (px)
TAG_SIZE = 30  # タグのサイズ (mm)

while True:
    img = sensor.snapshot()
    img.replace(vflip=False, hmirror=True, transpose=True)
    #img.chrominvar()#ホワイトバルの調整をしたいとき

    if uart.any():
        command = uart.read().decode('utf-8').strip()

        # 色検出
        if command in thresholds:
            img.chrominvar()#現地のコードとして使いたいとき
            threshold = thresholds[command]
            blobs = img.find_blobs([threshold], pixels_threshold=100, area_threshold=10, merge=True, margin=10)

            if blobs:
                largest_blob = max(blobs, key=lambda b: b.pixels())
                message = "{},{},{},{}\n".format(command, largest_blob.pixels(), largest_blob.cx(), largest_blob.cy())
            else:
                message = "{},0,0,0\n".format(command)

            uart.write(message)
            print(message)

        # AprilTag検出
        elif command == "T":
            tags = img.find_apriltags(roi=(ROI_X, ROI_Y, ROI_W, ROI_H),families=image.TAG16H5)
            tag_data = ["T"]

            for tag in tags:
                tag_data.append("{},{},{},{},{:.2f},{:.2f}".format(
                tag.id(), tag.cx(), tag.cy(), abs(tag.z_translation()),
                math.degrees(tag.x_rotation()), math.degrees(tag.y_rotation())
                ))
                
            message = ",".join(tag_data) + "\n"
            uart.write(message)
            print(message)

    time.sleep(0.01)
    
