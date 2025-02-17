import sensor
import image
import lcd
import time
from machine import UART
from fpioa_manager import fm
from Maix import GPIO

# LCDの初期化
lcd.init()
lcd.rotation(2)

# UARTのピン設定（UnitVのGPIO34をTX、GPIO35をRXに割り当て）
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=256)

# カメラの初期設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # RGBカラーモード
sensor.set_framesize(sensor.QVGA)  # QVGA解像度（320x240）
sensor.set_hmirror(0)  # 左右反転
sensor.set_auto_gain(False)  # オートゲインオフ
sensor.skip_frames(time=2000)  # 安定化のための待機

clock = time.clock()

# AprilTagを認識するIDの範囲（0～7の8枚）
VALID_TAG_IDS = set(range(8))

# ROI（AprilTag認識用のエリア）を設定
#上をカット
IMG_W, IMG_H = sensor.width(), sensor.height()  # 画像の幅と高さ
ROI_X, ROI_Y = 0, IMG_H // 4  # 上1/4をカットして、下3/4だけ処理
ROI_W, ROI_H = IMG_W, IMG_H - ROI_Y

# 右をカット
#IMG_W, IMG_H = sensor.width(), sensor.height()  # 画像の幅と高さ
#ROI_X, ROI_Y = 0, 0  # 左1/4をカット、右3/4を対象
#ROI_W, ROI_H = IMG_W * 3 // 4, IMG_H  # 幅を調整して左1/4をカット

while True:
    clock.tick()
    img = sensor.snapshot()  # 画像をキャプチャ

    # AprilTagの検出はROI内で実行
    tags = img.find_apriltags(roi=(ROI_X, ROI_Y, ROI_W, ROI_H), families=image.TAG36H11)

    for tag in tags:
        tag_id = tag.id()
        if tag_id in VALID_TAG_IDS:
            print("AprilTag detected: ID={}".format(tag_id))  # 検出結果をコンソールに表示

            # UARTでIDを送信
            uart.write("{}\n".format(tag_id))

            # タグを描画（ROIを考慮）
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
            img.draw_string(tag.cx(), tag.cy(), "ID:{}".format(tag_id), color=(0, 255, 255), scale=2)

    # 画像をLCDに表示（画像全体を表示）
    lcd.display(img)
