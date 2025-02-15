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
sensor.set_pixformat(sensor.GRAYSCALE)  # グレースケール
sensor.set_framesize(sensor.QVGA)  # QVGA解像度
sensor.set_hmirror(True)  # 左右反転
sensor.set_auto_gain(False)  # オートゲインオフ
sensor.skip_frames(time=2000)  # 安定化のための待機

clock = time.clock()

# 認識するAprilTagのID範囲（0～7の8枚）
VALID_TAG_IDS = set(range(8))

while True:
    clock.tick()
    img = sensor.snapshot()
    tags = img.find_apriltags(families=image.TAG36H11)

    for tag in tags:
        tag_id = tag.id()
        if tag_id in VALID_TAG_IDS:
            print(f"AprilTag detected: ID={tag_id}")

            # UARTでIDを送信
            uart.write(f"{tag_id}\n")

            # タグを描画
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
            img.draw_string(tag.cx(), tag.cy(), f"ID:{tag_id}", color=(0, 255, 255), scale=2)

    # 画像をLCDに表示
    lcd.display(img)
