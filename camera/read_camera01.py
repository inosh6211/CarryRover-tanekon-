from machine import UART, Pin
import time

# UARTの初期化
# UART1を使用、ボーレートはカメラと合わせて115200
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

# メインループ
while True:
    if uart1.any():  # UARTに受信データがあるか確認
        data = uart1.read().decode('utf-8')  # データを読み取り、文字列にデコード
        print("Received:", data.strip())  # 受信データをコンソールに表示
    time.sleep(0.1)  # CPU負荷を下げるため少し待機
