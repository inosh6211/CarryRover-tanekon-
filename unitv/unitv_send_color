import sensor
import image
import time
import math
from fpioa_manager import fm
from machine import UART

#UART設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=1024)

#カメラ初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)        # ゲイン
sensor.set_auto_whitebal(False)    # ホワイトバランス
sensor.set_auto_exposure(False)    # 露光調整
sensor.set_brightness(False)       # 輝度
sensor.set_contrast(False)         # コントラスト
sensor.set_saturation(False)       # 彩度

#ROI設定
ROI_X, ROI_Y, ROI_W, ROI_H = 60, 40, 200, 200
FOCAL_LENGTH_PX = 210.70
TAG_SIZE = 30

# 色の閾値（LAB）
threshold = [
    (0, 100, 0, 127, 43, 127),    # 赤
    (38, 50, -55, -8, -30, 1),    # 青
    (20, 120, 0, 40, -60, -20)    # 紫
]

while True:
    img = sensor.snapshot()
    img.replace(vflip=False, hmirror=True, transpose=True)

    while uart.any():
        data = uart.read().decode('utf-8').strip()

        #色認識
        if data == 0:
            img.chrominvar()
            data_c = ["c"]
            for color in range(3):
                blobs = img.find_blobs(
                    [threshold[color]],
                    pixels_threshold=100,
                    area_threshold=10,
                    merge=True,
                    margin=10
                )
        
                if blobs:
                    largest_blob = max(blobs, key=lambda b: b.pixels())
                    data_c.extend([
                        color,
                        largest_blob.pixels(),
                        largest_blob.cx(),
                        largest_blob.cy()
                    ])
                else:
                    data_c.extend([color, 0, 0, 0])
        
                message = ",".join(map(str, data_c))
        
            uart.write(message + "\n")
            print(message + "\n")

    time.sleep(0.1)
