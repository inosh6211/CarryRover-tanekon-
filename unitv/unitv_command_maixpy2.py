import sensor
import image
import time
import math
from fpioa_manager import fm
from machine import UART
from modules import ws2812

led = ws2812(8, 1)

# UART設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=1024)

# カメラ初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False,rgb_gain_db = (120, 100, 120))    # 現地調整
sensor.set_auto_exposure(-2)    # 露光(現地調整)
sensor.set_brightness(False)    # 輝度
sensor.set_contrast(False)      # コントラスト
sensor.set_saturation(False)
sensor.skip_frames(time=2000)


# 色の閾値（LAB）(現地調整)
thresholds = [
    (28, 100, 17, 127, 23, 127),  # 赤
    (11, 72, -52, 127, -128, -15),  # 青
    (18, 55, 7, 108, -128, -12)  # 紫
]

# ROI設定
ROI = [(0, 50, 240, 260), (0, 0, 240, 260)]  # (ROI_X, ROI_Y, ROI_W, ROI_H)

# マイコンとの接続確認
led.set_led(0, (0, 255, 0))
led.display()

while True:
    if uart.any():
        message = uart.readline().decode('utf-8').strip()
        if message == "1":
            led.set_led(0, (0, 0, 0))
            led.display()
            uart.write("1\n")
            break

        time.sleep(0.1)

while True:
    img = sensor.snapshot()
    img.replace(vflip=False, hmirror=True, transpose=True)
    #img.chrominvar()#ホワイトバルの調整をしたいとき

    if uart.any():
        command = uart.readline().decode('utf-8').strip()

        # 色検出
        if command == "C":
            img.chrominvar()  # 現地のコードとして使いたいとき
            color_data = ["C"]
            for color in range(3):
                blobs = img.find_blobs(
                    [thresholds[color]],
                    pixels_threshold=100,
                    area_threshold=10,
                    merge=True,
                    margin=10
                )

                if blobs:
                    largest_blob = max(blobs, key=lambda b: b.pixels())
                    color_data.extend([color, largest_blob.pixels(), largest_blob.cx(), largest_blob.cy()])

            message = ",".join(map(str, color_data)) + "\n"
            uart.write(message)
            print(message)

        # AprilTag検出(通常時)
        elif command == "T0":
            tags = img.find_apriltags(roi=ROI[0])
            tag_data = ["T"]

            for tag in tags:
                tag_data.extend([
                    tag.id(),
                    tag.cx(),
                    tag.cy(),
                    abs(tag.z_translation()),
                    math.degrees(tag.x_rotation()),
                    math.degrees(tag.y_rotation())
                ])

            message = ",".join(map(str, tag_data)) + "\n"
            uart.write(message)
            print(message)


        # AprilTag検出(物資保持時)
        elif command == "T1":
            tags = img.find_apriltags(roi=ROI[1])
            tag_data = ["T"]

            for tag in tags:
                tag_data.extend([
                    tag.id(),
                    tag.cx(),
                    tag.cy(),
                    abs(tag.z_translation()),
                    math.degrees(tag.x_rotation()),
                    math.degrees(tag.y_rotation())
                ])

            message = ",".join(map(str, tag_data)) + "\n"
            uart.write(message)
            print(message)

    time.sleep(0.01)
