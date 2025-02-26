import machine
import utime
import math

#UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

#キャリブレーション結果
TAG_SIZE = 30
FOCAL_LENGTH_X = 210.70  
FOCAL_LENGTH_Y = 210.70

CENTER_X = 100 # ROI_W / 2
CENTER_Y = 100 # ROI_H / 2
DIST_COEFFS = [-0.34155103, 0.25499062, -0.00517389, -0.00731524, -0.17990041]

#歪み補正
def undistort_point(x, y):
    norm_x = (x - CENTER_X) / FOCAL_LENGTH_X
    norm_y = (y - CENTER_Y) / FOCAL_LENGTH_Y
    r2 = norm_x**2 + norm_y**2
    radial_distortion = 1 + DIST_COEFFS[0] * r2 + DIST_COEFFS[1] * (r2**2) + DIST_COEFFS[4] * (r2**3)
    
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

            #AprilTag認識
            if len(data) == 4 and data[0] != "Color":
                tag_id = int(data[0])
                cx, cy = float(data[1]), float(data[2])
                distance = float(data[3]) 
                
                corrected_cx, corrected_cy = undistort_point(cx, cy)
                corrected_distance = distance
                
                pitch_rad = math.atan2((corrected_cy - CENTER_Y) / FOCAL_LENGTH_Y, 1)
                pitch = math.degrees(pitch_rad)
                yaw_rad = math.atan2((corrected_cx - CENTER_X) / FOCAL_LENGTH_X, 1)
                yaw = math.degrees(yaw_rad)
            
                print(f"AprilTag ID: {tag_id}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}, Distance: {corrected_distance:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

            #色認識
            elif len(data) == 4 and data[0] == "Color":
                color_name = data[1]
                cx, cy = float(data[2]), float(data[3])
                corrected_cx, corrected_cy = undistort_point(cx, cy)
                
                print(f"Detected Color: {color_name}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}")

        except Exception as e:
            print(f"Error: {e}")

    utime.sleep(0.1)
