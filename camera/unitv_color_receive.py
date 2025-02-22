import machine
import utime

# UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

print("Raspberry Pi Pico W Ready. Waiting for data...")

while True:
    if uart.any():  # 受信データがある場合
        try:
            data = uart.read().decode('utf-8').strip()  # 受信データをデコードして整形
            
            if data == "$0":
                print("No object detected.")
            else:
                objects = data.split(";")  # オブジェクト情報をセミコロンで分割（MaixPy側と統一）
                for obj in objects:
                    parts = obj.split(":")
                    if len(parts) == 2:
                        color = parts[0].strip()
                        coords = parts[1].strip()
                        coord_parts = coords.split(",")
                        if len(coord_parts) == 2:
                            try:
                                x, y = map(int, coord_parts)
                                print("Detected {} at ({}, {})".format(color, x, y))
                            except ValueError:
                                print("Invalid coordinate format: {}".format(coords))
        except Exception as e:
            print("Error processing data:", e)
    
    utime.sleep(0.1)
