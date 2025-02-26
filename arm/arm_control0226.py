from machine import I2C, Pin, UART
from servo import Servos
import time
import math

# === アーム制御クラス ===
class ArmController:
    def __init__(self):
        # I2C & サーボ初期設定
        self.i2c = I2C(1, scl=Pin(27), sda=Pin(26))
        self.servos = Servos(self.i2c)
        self.servos.pca9685.freq(50)
        
        # アームのリンク長
        self.L1 = 70  # 上腕
        self.L2 = 120 # 前腕
        self.L3 = 130 # 手首
        
        # サーボ角度の初期値
        self.current_angles = [90, 170, 0, 10, 10]  # [基部, 上腕, 前腕, 手首, ハンド]

        # 初期状態へ設定
        self.reset_arm()
    
    def reset_arm(self):
        """ アームを初期状態にセット """
        print("Setting arm to initial position...")
        for i, angle in enumerate(self.current_angles):
            self.move_servo_smoothly(i, angle)
        print(f"Initial angles: {self.current_angles}")

    def move_servo(self, index, angle):
        """ サーボを即座に指定角度に移動 """
        angle = max(0, min(180, angle))
        self.servos.position(index, degrees=int(angle))
        self.current_angles[index] = int(angle)
        

    def move_servo_smoothly(self, index, target_angle, step=2, delay=0.1):
        """ サーボを少しずつ滑らかに移動 """
        current_angle = self.current_angles[index]
        step = step if target_angle > current_angle else -step

        for angle in range(current_angle, target_angle, step):
            angle = max(0, min(180, angle))
            self.servos.position(index, degrees=int(angle))
            self.current_angles[index] = int(angle)
            print(f"Servo {index} moving... {int(angle)}°")
            time.sleep(delay)
            

        self.servos.position(index, degrees=int(target_angle))
        self.current_angles[index] = int(target_angle)
        print(f"Servo {index} reached target {target_angle}°")

    def expand_mode(self):
        """ アームを拡張 """
        print("\nMoving to EXPAND MODE...")
        self.move_servo_smoothly(2, 80)
        self.move_servo_smoothly(0, 0)        
        self.move_servo_smoothly(3, 90)
        print(f"Angles in EXPAND MODE: {self.current_angles}\n")
        
    def search_mode(self):
        """ 探知状態"""
        print("fixing posture")
        self.move_servo_smoothly(1, 110)
        self.move_servo_smoothly(2, 150)
        self.move_servo_smoothly(3, 150)
        print(f"Angles in SEARCH MODE")

    def scan_for_target(self, camera):
        """ 手首の角度を変えながら物資を探す """
        print("Scanning for target...")

        for theta_3 in range(self.current_angles[3], 0, -5):  # 手首を徐々に下げる
            self.move_servo_smoothly(3, theta_3)
            time.sleep(0.1)

           

            if camera.uart.any():  
                message = camera.uart.read().decode('utf-8').strip()
                print(f"Received raw data: {message}")  # デバッグ出力

                parts = message.split(",")

                if parts[0] == "TAG":
                    try:
                        tag_id = int(parts[1])
                        cx = int(parts[2])
                        cy = int(parts[3])
                        Z = float(parts[4])
                        pitch = float(parts[5])
                        yaw = float(parts[6])
                        print(f"物資発見: ID={tag_id}, 距離={Z}mm, Pitch={pitch}°, Yaw={yaw}°")
                        return Z, pitch, yaw  # 見つかったら距離・角度を返す
                    except Exception as e:
                        print(f"Error parsing message: {e}")
                        continue  
            else:
                print("No data received yet.")  #デバッグ出力

        print("Target not found.")
        return None, None, None  # 物資が見つからなかった場合



    def detect_mode(self, yaw):
        """ 物資発見時 (手首部を物資方向に調整) """
        new_theta_3 = self.current_angles[3] + yaw
        self.move_servo_smoothly(3, new_theta_3)

    def grip(self, close=True):
        """ ハンドの開閉 """
        self.move_servo_smoothly(4, 20 if close else 50)

    def approach_target(self, x_t, y_t):
        """ 物資に近づく """
        d = math.sqrt(x_t**2 + y_t**2)

        L_12 = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(math.radians(self.current_angles[2])))
        theta3_prime = math.degrees(math.acos((self.L2**2 + L_12**2 - self.L1**2) / (2 * self.L2 * L_12)))
        theta5 = math.degrees(math.acos((L_12**2 + self.L3**2 - d**2) / (2 * L_12 * self.L3)))

        target_theta_3 = theta5 + theta3_prime
        print(f"Moving wrist to target θ3: {target_theta_3:.2f}°")
        
        # 手首の調整
        self.move_servo_smoothly(3, target_theta_3)

        # 徐々に上腕を曲げて近づく
        print("Moving upper arm towards the target...")
        for angle in range(self.current_angles[1], 180, -5):
            self.move_servo_smoothly(1, angle)

# === カメラデータ受信クラス ===
class CameraReceiver:
    def __init__(self):
        self.uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
        print("Sending start signal to UnitV...")
        self.uart.write("start\n")
        time.sleep(2)
        print("Start signal sent")

    def read_camera(self):
        """ UnitV からデータを受信 """
        self.uart.write("1\n")  # UnitV にデータリクエスト
        time.sleep(0.1)

        while self.uart.any() == 0:
            pass

        message = self.uart.read().decode('utf-8').strip()  # 受信データのデコード & 余分な改行削除
        print(f"Received raw data: {message}")

        

        # 期待されるフォーマットか確認
        if message.startswith("TAG"):
            parts = message.split(",")

            if len(parts) == 7:  # データ数
                try:
                    tag_id = int(parts[1])
                    cx = int(parts[2])
                    cy = int(parts[3])
                    Z = float(parts[4])
                    pitch = float(parts[5])
                    yaw = float(parts[6])

                    print(f"Parsed TAG data: ID={tag_id}, cx={cx}, cy={cy}, Z={Z}, Pitch={pitch}, Yaw={yaw}")
                    return ("TAG", tag_id, cx, cy, Z, pitch, yaw)
                except ValueError as e:
                    print(f"Data conversion error: {e}")
                    return None

        

        else:
            print(f"Error: Unexpected message format: {message}")
            return None

# === メイン動作シーケンス ===
class MainSequence:
    def __init__(self):
        self.arm = ArmController()
        self.camera = CameraReceiver()

    def run(self):
        """ メイン動作 """
        self.arm.expand_mode()
        self.arm.search_mode()

        # 手首を動かしながら物資を探す
        d, pitch, yaw = self.arm.scan_for_target(self.camera)

        if d is None:
            print("No target found. Stopping.")
            return

        # 手首の微調整
        self.arm.detect_mode(yaw)

        # 物資に近づく
        self.arm.approach_target(0, d)

        # 把持
        self.arm.grip(close=True)

# === 実行 ===
if __name__ == "__main__":
    sequence = MainSequence()
    sequence.run()

