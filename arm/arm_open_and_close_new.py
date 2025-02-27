from machine import I2C, Pin, UART
from servo import Servos
import time
import math

# === アーム制御クラス ===
class ArmController:
    def __init__(self):
        self.i2c = I2C(1, scl=Pin(27), sda=Pin(26))
        self.servos = Servos(self.i2c)
        self.servos.pca9685.freq(50)
        self.CX=120
        self.CY=160

        # arm length
        self.L1 = 70  # 上腕
        self.L2 = 120 # 前腕
        self.L3 = 100 # 手首

        # initial angles
        self.current_angles = [0, 110, 120, 150, 0]  # [基部, 上腕, 前腕, 手首, ハンド]

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

    def move_servo_smoothly(self, index, target_angle, step=1, delay=0.025):
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
        self.move_servo_smoothly(2, 70)
        self.move_servo_smoothly(0, 0)        
        self.move_servo_smoothly(3, 90)
        print(f"Angles in EXPAND MODE: {self.current_angles}\n")
        
    
    
    def search_mode(self):
        """ 探知状態 """
        print("Moving to SEARCH MODE...")
        self.move_servo_smoothly(1, 110)
        self.move_servo_smoothly(2, 120)
        self.move_servo_smoothly(3, 150)
        self.move_servo_smoothly(4, 60)
        print(f"Angles in SEARCH MODE: {self.current_angles}")

    def scan_for_target(self, camera):
        """ 手首の角度を変えながら物資を探す """
        print("Scanning for target...")

        for theta_3 in range(self.current_angles[3], 0, -5):  # 手首を徐々に下げる
            self.move_servo_smoothly(3, theta_3)
            time.sleep(0.1)

            print("Waiting for UART data...")
            data = camera.read_camera()
            if data:
                print(f"Received Data: {data}")
                return data

        print("Target not found.")
        return None

        print("Target not found.")
        return None, None, None, None


    def detect_mode(self, cx, pitch):
        """ 物資発見時 (手首部を物資方向に調整) """
        new_theta_3 = self.current_angles[3] + int(pitch)
        self.move_servo_smoothly(3, new_theta_3)
        print(f"new_theta_3: {new_theta_3}")
        
        offset_x = cx - self.CX  # 物資の x 座標が画面の中心からどれだけズレているか

        
        threshold = 3  # 許容される cx の誤差ピクセル数

        if abs(offset_x) > threshold:
            correction_angle = int(abs(offset_x) * 0.1)  # 調整する角度

            if offset_x > 0:  # 物資が画面の右側にある場合 (cx > CX)
                new_theta_0 = self.current_angles[0] - correction_angle  # θ0 を減少
                direction = "right → decreasing θ0"
            else:  # 物資が画面の左側にある場合 (cx < CX)
                new_theta_0 = self.current_angles[0] + correction_angle  # θ0 を増加
                direction = "left → increasing θ0"

            # サーボ角度の範囲制限
            new_theta_0 = max(0, min(180, 0))
            self.move_servo_smoothly(0, new_theta_0)

            print(f"Adjusted θ0: {new_theta_0}° (cx={cx}, offset_x={offset_x}, {direction})")
        else:
            print("almost ok")


    def zero_mode(self):
        """ 基部の移動 """
        self.move_servo_smoothly(0, 60)

    def nonzero_mode(self):
        """ 基部の移動 """
        self.move_servo_smoothly(0, 0)

    def closer(self):
        """ 上腕と前腕の移動 """
        for theta_1 in range(self.current_angles[1], 30, -1):
            self.move_servo_smoothly(1, theta_1)
            
        for theta_2 in range(self.current_angles[2], 160, 1):
            self.move_servo_smoothly(2, theta_2)
        
    def further(self):
        """ 上腕と前腕の移動 """
        for theta_2 in range(self.current_angles[2], 120, -1):
            self.move_servo_smoothly(2, theta_2)
            
        for theta_1 in range(self.current_angles[1], 110, 1):
            self.move_servo_smoothly(1, theta_1)

    def grip(self, close=True):
        """ ハンドの開閉 """
        self.move_servo_smoothly(4,0 if close else 60)

    def gripa(self, opened=True):
        """ ハンドの開閉 """
        self.move_servo_smoothly(4,60 if opened else 0)

    def approach_target(self, Z, camera):
        """ 物資に近づく """
        
        x_t = -self.L1 * math.cos(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.cos(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              (self.L3 + Z ) * math.cos(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))

        y_t = self.L1 * math.sin(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.sin(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) - \
              (self.L3 + Z ) * math.sin(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))

        #print(f"a = {-self.L1 * math.cos(math.radians(170 - self.current_angles[1]))}")
        print(f"x_t={x_t}, y_t={y_t}")
        
        d = math.sqrt(x_t**2 + y_t**2)
        
        print(f"d={d}")

        L_12 = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(math.radians(self.current_angles[2])))
        
        #print(f"L_12={L_12}")
        theta3_prime = math.degrees(math.acos((self.L2**2 + L_12**2 - self.L1**2) / (2 * self.L2 * L_12)))
#       デバッグ
        #print(f"theta3'={theta3_prime}")
        acos_val1 = (self.L2**2 + L_12**2 - self.L1**2) / (2 * self.L2 * L_12)
        acos_val2 = (L_12**2 + self.L3**2 - d**2) / (2 * L_12 * self.L3)

        #print(f"acos_val1={acos_val1},acos_val2={acos_val2}")
        
        
        theta5 = math.degrees(math.acos((L_12**2 + self.L3**2 - d**2) / (2 * L_12 * self.L3)))
        #print(f"theta5={theta5}")
        target_theta_3 = theta5 + theta3_prime
        print(f"Moving wrist to target θ3: {target_theta_3:.2f}°")
        
        
        self.move_servo_smoothly(3, int(target_theta_3+5))

        # 徐々に上腕を曲げて近づく
        print("Moving upper arm towards the target...")
        for theta_1 in range(self.current_angles[1], 0, -1):
            self.move_servo_smoothly(1, theta_1)
            print(f"distance={Z}")
            camera_data = camera.read_camera()
            
            

            if camera_data is None :
                print(f"stop! (θ1={theta_1}°)")
                break
            
            else:
                print("No valid data received, skipping iteration.")

                        
                        

                        


            # === カメラデータ受信クラス ===
class CameraReceiver:
    def __init__(self):
        self.uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
        print("CameraReceiver initialized. Sending start signal to UnitV...")
        self.uart.write("start\n")
        time.sleep(2)
        print("Start signal sent")

    def read_camera(self):
        """ UnitV にリクエストを送り、データを受信 """
        self.uart.write("1\n")  # データ要求
        time.sleep(0.1)

        if self.uart.any():
            message = self.uart.read().decode('utf-8').strip()
            print(f"Received raw data: {message}")

            lines = message.split("\n")
            for line in lines:
                parts = line.split(",")
                if len(parts) == 6 and parts[0] == "TAG":
                    try:
                        tag_id = int(parts[1])
                        cx = int(parts[2])
                        cy = int(parts[3])
                        Z = float(parts[4])
                        pitch = float(parts[5])
                        

                        print(f"Parsed TAG data: ID={tag_id}, cx={cx}, cy={cy}, Z={Z}, Pitch={pitch}")
                        return ("TAG", tag_id, cx, cy, Z, pitch)
                    except ValueError as e:
                        print(f"Error parsing TAG data: {e}")

                

        print("No valid data received.")
        return None

# === メイン動作シーケンス ===
class MainSequence:
    def __init__(self):
        self.arm = ArmController()
        self.camera = CameraReceiver()
    
    def run(self):
        """ メイン動作 """
        self.arm.zero_mode()
        self.arm.closer()
        self.arm.gripa(opened=True)
        self.arm.further()
        self.arm.nonzero_mode()
        self.arm.search_mode()
        
        data = self.arm.scan_for_target(self.camera)
        
        if data is None:
            print("No target found. Stopping.")
            return

        if data[0] == "TAG":
            _, tag_id, cx, cy, Z, pitch = data
            self.arm.detect_mode(cx,pitch)
            self.arm.approach_target(Z, self.camera)
            self.arm.grip(close=True)

        print("Finished scanning and adjusting.")


    


# === 実行 ===
if __name__ == "__main__":
    sequence = MainSequence()
    sequence.run()
