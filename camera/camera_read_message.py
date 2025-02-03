from machine import UART, Pin
import time

uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

def read_camera():
    uart1.write("1\n")

    while uart1.any() == 0:
        pass

    message = ""

    while uart1.any() > 0:
        message += uart1.read().decode('utf-8').strip()

    if message.startswith("$"):
        data = message[1:].split(",")
        
        if data[0] == "0":
            return None

        objects_count = int(data[0])
        data = list(map(int, data[1:]))

        max_pixels = 0
        target_cx = 0
        target_cy = 0

        for i in range(objects_count):
            pixels = data[i * 3]
            cx = data[i * 3 + 1]
            cy = data[i * 3 + 2]

            if pixels > max_pixels:
                max_pixels = pixels
                target_cx = cx
                target_cy = cy

        return max_pixels, target_cx, target_cy

while True:
    camera_data = read_camera()
    if camera_data is None:
        pixels, cx, cy = 0, 0, 0  # デフォルト値
    else:
        pixels, cx, cy = camera_data

    print(f"ピクセル数: {pixels}, 中心座標: ({cx}, {cy})")
    time.sleep(0.1)
