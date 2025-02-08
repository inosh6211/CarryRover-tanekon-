from machine import I2C, Pin, UART
from servo import Servos
import time
import math

# === 初期セットアップ ===
i2c = I2C(1, scl=Pin(27), sda=Pin(26))
servos = Servos(i2c)
servos.pca9685.freq(50)

# === サーボ制御関数 ===
def control_servo(index, angle):
    servos.position(index, degrees=angle)

# === UART 設定 (UnitVとの通信) ===
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

print("Sending start signal to UnitV...")
uart1.write("start\n")  # UnitV に "start" を送信
time.sleep(2)  # UnitV の起動を待つ
print("Start signal sent")

# === アームの長さ (mm) ===
L1 = 70  # 上腕
L2 = 120 # 前腕
L3 = 130 # 手首

# === カメラ情報 ===
IMG_WIDTH = 240
IMG_HEIGHT = 320
FOCAL_LENGTH = 2.04
SENSOR_WIDTH = 2.7552

# === カメラのピクセル座標を実空間座標に変換 ===
def pixel_to_real(x_pixel, y_pixel, obj_width):
    obj_real_width = 20  # 物体の実際の幅 (mm)
    distance = (FOCAL_LENGTH * obj_real_width * IMG_WIDTH) / (obj_width * SENSOR_WIDTH)

    x_real = (x_pixel - IMG_WIDTH / 2) * (distance / FOCAL_LENGTH)
    y_real = (IMG_HEIGHT / 2 - y_pixel) * (distance / FOCAL_LENGTH)

    return distance, x_real, y_real

# === 逆運動学 (x_t, y_t) を元にアームの角度を計算 ===
def inverse_kinematics(x_t, y_t):
    x_w = x_t - L3 * math.cos(math.radians(0))
    y_w = y_t - L3 * math.sin(math.radians(0))
    d = (x_w**2 + y_w**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    if abs(d) > 1:
        raise ValueError("Target is out of range")

    theta_1 = math.degrees(math.atan2(y_w, x_w) - math.atan2(L2 * math.sin(math.acos(d)), L1 + L2 * math.cos(math.acos(d))))
    theta_2 = math.degrees(math.acos(d))
    theta_3 = -(theta_1 + theta_2)

    return theta_1, theta_2, theta_3

# === UnitV からデータを受信  ===
def read_camera():
    uart1.write("1\n")  # UnitV にデータリクエスト
    time.sleep(0.1)  # 少し待機

    while uart1.any() == 0:
        pass

    message = ""
    while uart1.any() > 0:
        message += uart1.read().decode('utf-8').strip()

    print(f"Received raw message: '{message}'")  # 🔍 受信データを確認

    if message.startswith("$"):
        data = message[1:].split(",")  # `$` を除去して `,` で分割
        print(f"Parsed data: {data}")  # 🔍 分割したデータを表示

        if data[0] == "0":  # 物体が見つからない場合
            return None, None, None

        try:
            objects_count = int(data[0])  # 最初の要素は物体数
            data = list(map(int, data[1:]))  # 残りのデータを `int` に変換
        except ValueError as e:
            print(f"Data conversion error: {e}")  # 🔍 変換エラーの内容を表示
            return None, None, None

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

    return None, None, None  # 不正なデータの場合は `None` を返す



# === アームの初期位置設定 ===
control_servo(0, 0)  # 基部
control_servo(1, 180)  # 肩
control_servo(2, 10) # 肘
control_servo(3, 0)  # 手首
control_servo(4, 0)  # ハンド

# === メインループ ===
while True:
    pixels, x_pixel, y_pixel = read_camera()

    if pixels is not None:
        print(f"Detected object at ({x_pixel}, {y_pixel}), size {pixels}px")

        # カメラ座標を実空間座標に変換
        distance, x_t, y_t = pixel_to_real(x_pixel, y_pixel, pixels)

        # 徐々にアームを動かす
        for step in range(0, 100, 10):
            x_step = x_t * (step / 100)
            y_step = y_t * (step / 100)

            try:
                theta_1, theta_2, theta_3 = inverse_kinematics(x_step, y_step)
                control_servo(1, theta_1)  # 肩
                control_servo(2, theta_2)  # 肘
                control_servo(3, theta_3)  # 手首

                time.sleep(0.5)
            
            except ValueError:
                print("Target out of range.")

        # 物体が近づいたら把持動作
        if distance < 50:
            control_servo(4, 30)  # ハンドを閉じる
            time.sleep(1)

    time.sleep(0.1)

