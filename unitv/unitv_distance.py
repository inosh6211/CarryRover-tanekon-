import sensor
import image
import time
import lcd
import math
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

# カメラセンサーを初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160*120
sensor.set_hmirror(0)  # 左右反転
sensor.set_auto_gain(False)  # オートゲインオフ
sensor.skip_frames(time=2000)

clock = time.clock()

# カメラ行列（mtx）と歪み係数（dist）を手動で設定（OpenCVでキャリブレーションした結果）
mtx = [[250.89, 0, 177.63],
                [0, 250.69, 94.85],
                [0, 0, 1]]

dist = [-0.3438, -0.3478, 0.0139, -0.0107, 0.7112]

# AprilTagの実際のサイズ（mm）
H = 82  # Apriltagの一辺の長さ（例）

# AprilTagを認識するIDの範囲（0～7の8枚）
VALID_TAG_IDS = set(range(8))

# ROI（AprilTag認識用のエリア）を設定
# 上をカット
IMG_W, IMG_H = sensor.width(), sensor.height()  # 画像の幅と高さ
ROI_X, ROI_Y = 0, IMG_H // 4  # 上1/4をカットして、下3/4だけ処理
ROI_W, ROI_H = IMG_W, IMG_H - ROI_Y

while True:
    clock.tick()
    # 画像をキャプチャ
    img = sensor.snapshot()

    # AprilTagの検出はROI内で実行
    tags = img.find_apriltags(roi=(ROI_X, ROI_Y, ROI_W, ROI_H), families=image.TAG36H11)

    for tag in tags:
        # タグのIDと位置を表示
        print("Tag ID:", tag.id)

        # タグのコーナー情報から高さを計算
        corners = tag.corners()  # メソッドを呼び出してコーナーを取得
        top_left = corners[0]
        bottom_left = corners[3]
        tag_height = math.sqrt((bottom_left[1] - top_left[1]) ** 2 + (bottom_left[0] - top_left[0]) ** 2)

        # 焦点距離 (fx または fy) をカメラ行列から取得
        fx = mtx[0][0]  # カメラ行列のfocal length
        distance = H * fx / tag_height  # 距離の計算式

        # 結果をコンソールとLCDに表示
        print("Distance = {}".format(distance))

    # LCDに表示
    lcd.display(img)
