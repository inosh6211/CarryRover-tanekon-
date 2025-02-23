import machine
import utime

# UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

# カメラキャリブレーション結果
#FOCAL_LENGTH_X = 128.03139673
#FOCAL_LENGTH_Y = 133.17879223
#CENTER_X = 145.52128861
#CENTER_Y = 56.5631214
#DIST_COEFFS = [-0.60993323, 0.36441004, 0.03539258, 0.00115286, -0.08996402]

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

                # データのフォーマットが正しいか確認
                if len(data) == 6:
                    tag_id = int(data[0])    # ID
                    cx = float(data[1])      # X座標
                    cy = float(data[2])      # Y座標
                    distance = float(data[3])  # 距離
                    pitch = float(data[4])     # ピッチ角
                    yaw = float(data[5])       # ヨー角

                    # 手動で歪み補正
                    undistorted_cx, undistorted_cy = undistort_point(cx, cy)

                    # 距離補正（焦点距離を考慮）
                    corrected_distance = (FOCAL_LENGTH_X * 87) / undistorted_cx

                    # 受信データの補正後の表示
                    print(f"ID: {tag_id}, Corrected X: {undistorted_cx:.2f}, Corrected Y: {undistorted_cy:.2f}, "
                          f"Corrected Distance: {corrected_distance:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        except Exception as e:
            print(f"Error: {e}")

    utime.sleep(0.1)  # 100ms待機



