import sensor
import lcd
import time

# LCDの初期化
lcd.init()

# カメラの初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)  # 設定が安定するまで待機

# メインループ
while True:
    img = sensor.snapshot()  # 画像を取得
    lcd.display(img)  # LCDに表示
    time.sleep(0.05)
