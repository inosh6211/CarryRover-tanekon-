# =arm_control.pyとセット=
import sensor
import image
import lcd
import time
from Maix import GPIO
from fpioa_manager import fm
from modules import ws2812
from machine import UART

# LED 
led = ws2812(8, 1)
led.set_led(0, (0, 255, 0))
led.display()

# === UART 初期化（Raspberry Pi との通信） ===
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)

# LCD の初期化
lcd.init()

# カメラの初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # カラー画像
sensor.set_framesize(sensor.QVGA)  # 320x240 
sensor.set_auto_gain(False)  # 自動ゲインオフ
sensor.set_auto_whitebal(False)  # 自動ホワイトバランスオフ
sensor.skip_frames(time=2000)  # 設定が安定するまで待機

# 物資の色範囲（色検出）
red_threshold = (30, 80, 20, 60, 40, 80)  # orange色の閾値

print("UnitV Ready. Waiting for objects...")  # 起動確認用めっさげ

frame_count = 0
log_interval = 10  # 何フレームごとにログを表示するか

# メインループ
while True:
    img = sensor.snapshot()  # 画像取得
    img.replace(vflip=False, hmirror=True, transpose=True)

    # 物資の色を検出
    blobs = img.find_blobs([red_threshold], pixels_threshold=200, area_threshold=200, merge=True)

    if blobs:  # 物体が検出された場合のみ処理を進める
        try:
            # 一番大きな物体を選択
            largest_blob = max(blobs, key=lambda b: b.pixels())

            # 物体の座標情報を取得
            x, y, w, h = largest_blob.rect()
            x_center = x + w // 2  # 物体の中心 X 座標
            y_center = y + h // 2  # 物体の中心 Y 座標
            pixels = largest_blob.pixels()  # 物体のピクセル数

            # LCD に結果を表示
            img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))  # 物体を囲む
            img.draw_cross(x_center, y_center, color=(0, 255, 0))  # 中心に十字マーク
            lcd.display(img)

            # Raspberry Pi にデータ送信（物体の中心座標）
            data_str = "${},{},{},{}\n".format(1, pixels, x_center, y_center)
            uart.write(data_str)

            # **一定フレームごとにログを表示**
            if frame_count % log_interval == 0:
                print("Blob detected: x={}, y={}, w={}, h={}".format(x, y, w, h))
                print("Center coordinates: x_center={}, y_center={}".format(x_center, y_center))
                print("Sending data: {}".format(data_str))  # 送信データの確認

            frame_count += 1  # フレームカウンターを更新

        except Exception as e:
            print("Error while processing blob: ", str(e))

    else:
        # 物体が見つからなかった場合、"$0\n" を送信
        uart.write("$0\n")
        if frame_count % log_interval == 0:
            print("No object detected.")

    frame_count += 1  # フレームカウンターを更新
    time.sleep(0.1)  # カメラのフレームレートを維持
