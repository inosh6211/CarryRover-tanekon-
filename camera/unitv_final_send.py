import machine
import utime

# UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

# カメラキャリブレーション結果
TAG_SIZE = 20
FOCAL_LENGTH_X = 250.8939245
FOCAL_LENGTH_Y = 250.6935671
CENTER_X = 177.63441089
CENTER_Y = 94.85498261
DIST_COEFFS = [-0.34382066, -0.34788885, 0.01396397, -0.01068236, 0.71124219]

# 歪み補正の簡易モデル
def undistort_point(x, y):
    norm_x = (x - CENTER_X) / FOCAL_LENGTH_X
    norm_y = (y - CENTER_Y) / FOCAL_LENGTH_Y
    r2 = norm_x**2 + norm_y**2
    radial_distortion = 1 + DIST_COEFFS[0] * (r2**2) + DIST_COEFFS[1] * (r2**4) + DIST_COEFFS[4] * (r2**6)
    corrected_x = norm_x * radial_distortion * FOCAL_LENGTH_X + CENTER_X
    corrected_y = norm_y * radial_distortion * FOCAL_LENGTH_Y + CENTER_Y
    return corrected_x, corrected_y

while True:
    if uart.any():  
        try:
            line = uart.readline()
            if line:
                try:
                    line = line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    print("UART decoding error")
                    continue
            else:
                continue
            data = line.split(',')

            # AprilTag
            if len(data) == 4 and data[0] != "NoTag":
                tag_id = int(data[0])
                cx, cy = float(data[1]), float(data[2])
                distance = float(data[3])
                
                corrected_cx, corrected_cy = undistort_point(cx, cy)
                corrected_distance = (FOCAL_LENGTH_X * TAG_SIZE) / corrected_cx
                
                pitch_rad = math.atan2(corrected_cy, corrected_distance)
                pitch = math.degrees(pitch_rad)
                yaw_rad = math.atan2(corrected_cx, corrected_distance)
                yaw = math.degrees(yaw_rad)
            
                print(f"AprilTag ID: {tag_id}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}, Distance: {corrected_distance:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

            # 色認識データの処理
            elif len(data) == 4 and data[0] == "Color" and data[1] != "None":
                color_name = data[1]
                cx, cy = float(data[2]), float(data[3])
                corrected_cx, corrected_cy = undistort_point(cx, cy)
                
                print(f"Detected Color: {color_name}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}")

        except Exception as e:
            print(f"Error: {e}")

    utime.sleep(0.1)
