import machine
import utime

# UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

# カメラキャリブレーション結果
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
    if uart.any():  # 受信バッファにデータがあるか確認
        try:
            # 1行のデータを受信
            line = uart.readline()
            if line:
                # 文字列に変換（decode）して前後の空白や改行を削除
                line = line.decode('utf-8').strip()

                # カンマ区切りで分割
                data = line.split(',')

                # AprilTagデータの処理
                if len(data) == 6 and data[0].isdigit():
                    tag_id = int(data[0])    # ID
                    cx = float(data[1])      # X座標
                    cy = float(data[2])      # Y座標
                    distance = float(data[3])  # 距離 (カメラ側の計算済みZ)
                    pitch = float(data[4])     # ピッチ角
                    yaw = float(data[5])       # ヨー角

                    # 手動で歪み補正
                    undistorted_cx, undistorted_cy = undistort_point(cx, cy)
                    corrected_distance = distance  

                    # 受信データの補正後の表示
                    print(f"AprilTag ID: {tag_id}, Corrected X: {undistorted_cx:.2f}, Corrected Y: {undistorted_cy:.2f}, "
                          f"Distance: {corrected_distance:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
                
                # 色認識データの処理
                elif len(data) == 4 and data[0] == "color":
                    color_name = data[1]  # 色の名前 
                    cx = float(data[2])   # X座標
                    cy = float(data[3])   # Y座標

                    # 手動で歪み補正
                    undistorted_cx, undistorted_cy = undistort_point(cx, cy)

                    # 色認識データの表示
                    print(f"Detected Color: {color_name}, Corrected X: {undistorted_cx:.2f}, Corrected Y: {undistorted_cy:.2f}")

        except Exception as e:
            print(f"Error: {e}")

    utime.sleep(0.1)  # 100ms待機
