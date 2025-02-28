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
sensor.set_auto_whitebal(False)    # ホワイトバランス#もとはFalse
sensor.set_auto_exposure(False)    # 露光調整
sensor.set_brightness(False)       # 輝度
sensor.set_contrast(False)         # コントラスト
sensor.set_saturation(False)       # 彩度

#ROI設定
ROI_X, ROI_Y, ROI_W, ROI_H = 0, 50, 240, 260
FOCAL_LENGTH_PX = 210.70
TAG_SIZE = 30



#Apriltag認識
while True:
    img = sensor.snapshot()
    img.replace(vflip=True, hmirror=False, transpose=True)
    
    while uart.any():
        data = uart.read().decode('utf-8').strip()
        if data == "t":
            data_t = [0] * 50
            img = sensor.snapshot()
            img.replace(vflip=True, hmirror=False, transpose=True)
            tags = img.find_apriltags(families=image.TAG16H5,roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
            if tags:
                for tag in tags:
                        data_t[5 * tag.id() + 0] = 1
                        data_t[5 * tag.id() + 1] = tag.cx() - (ROI_W / 2)
                        data_t[5 * tag.id() + 2] = tag.cy() - (ROI_H / 2)
                        data_t[5 * tag.id() + 3] = tag.z_translation()
                        data_t[5 * tag.id() + 4] = tag.y_rotation()
            message = ",".join(map(str, data_t))
            uart.write("t," + message + "\n")
            print("t," + message + "\n")
    
    time.sleep(0.01)
            message = ",".join(map(str, data_t))
            uart.write("t," + message + "\n")
            print("t," + message + "\n")
    
    time.sleep(0.1)
