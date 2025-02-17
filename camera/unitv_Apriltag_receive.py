import machine
import utime

# UARTの初期設定（PicoのUART1を使用）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)  # TX: GPIO4, RX: GPIO5

while True:
    if uart.any():  # 受信データがあるか確認
        data = uart.readline()  # 1行（ID）を読み取る
        if data:
            try:
                tag_id = int(data.strip())  # バイト列を整数に変換
                print(f"Received AprilTag ID: {tag_id}")
            except ValueError:
                print("Invalid data received:", data.decode('utf-8'))  # エラーデータを表示

    utime.sleep(0.1)  # 少し待機
