import sensor
import image
import lcd
import time
import math
from machine import UART
from fpioa_manager import fm
from Maix import GPIO

# LCDの初期化
lcd.init()
lcd.rotation(2)

# UARTのピン設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=256)

# カメラの初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(0)  # 左右反転
sensor.set_auto_gain(False)  # オートゲインオフ
sensor.skip_frames(time=2000)

clock = time.clock()

# カメラパラメータ（キャリブレーション値）
FOCAL_LENGTH = 97.5  # カメラの焦点距離
IMG_W, IMG_H = sensor.width(), sensor.height()
CX, CY = IMG_W // 2, IMG_H // 2  # 画像中心

# AprilTagの物理サイズ (mm)
TAG_SIZE = 87

# ROI（AprilTag認識用のエリア）を設定
#上1/4をカットして、下3/4だけ処理
#ROI_X, ROI_Y = 0, IMG_H // 4
#ROI_W, ROI_H = IMG_W, IMG_H - ROI_Y

#右1/4をカットして、左3/4だけ処理
#ROI_X, ROI_Y = 0, 0
#ROI_W, ROI_H = IMG_W * 3 // 4, IMG_H

# 左1/5をカットして、右4/5だけ処理
ROI_X = IMG_W // 5
ROI_Y = 0
ROI_W = IMG_W - ROI_X
ROI_H = IMG_H


while True:
    img = sensor.snapshot()

    # AprilTag の検出
    tags = img.find_apriltags(roi=(ROI_X, ROI_Y, ROI_W, ROI_H), families=image.TAG36H11)

    for tag in tags:
        id = tag.id()
        cx = tag.cx()
        cy = tag.cy()
        tag_width = tag.w()
        tag_height = tag.h()

        # Z の計算（三角測量）
        if tag_width > 0:
            Z = (FOCAL_LENGTH * TAG_SIZE) / tag_width  # mm単位
        else:
            Z = -1  # 検出エラー

        # X, Y のカメラ座標系への変換
        X = ((cx - CX) * Z) / FOCAL_LENGTH
        Y = ((cy - CY) * Z) / FOCAL_LENGTH

        # 俯角 (Pitch) = tan⁻¹(Y/Z)
        pitch_rad = math.atan2(Y, Z)
        pitch_deg = math.degrees(pitch_rad)

        # 仰角 (Yaw) = tan⁻¹(X/Z)
        yaw_rad = math.atan2(X, Z)
        yaw_deg = math.degrees(yaw_rad)

        # UARTで送信
        data = "{},{},{},{:.2f},{:.2f},{:.2f}".format(id, cx, cy, Z, pitch_deg, yaw_deg)
        uart.write(data + "\n")

        # Z, Pitch, Yaw の値を print
        print("ID: {}, X: {:.2f}, Y: {:.2f}, Distance: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(
            id, X, Y, Z, pitch_deg, yaw_deg))

    lcd.display(img)
