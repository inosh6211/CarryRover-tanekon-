from machine import I2C, Pin, UART
from servo import Servos
import time
import math

# --- 各種設定 ---
I2C1 = I2C(1, scl=Pin(27), sda=Pin(26))
UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))


# === CameraReceiverクラス ===
class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart
        time.sleep(2)

        # UnitVとの接続確認
        """while True:
            self.uart.write("1\n")
            while not self.uart.any():
                print("Waiting for UnitV signal...")
                time.sleep(0.5)

            message = self.uart.readline().decode('utf-8').strip()
            if message == "1":
                print("UnitV connected")
                break
            time.sleep(0.1)"""

    def read_camera(self):
        message = ""
        while True:
            if self.uart.any():
                char = self.uart.read(1).decode('utf-8')
                if char == "\n":
                    break
                message += char
                time.sleep(0.01)
        print(message)
        return message.split(',')

    def read_tags(self, mode):
        self.tag_detected = [0] * 10
        self.tag_cx = [0] * 10
        self.tag_cy = [0] * 10
        self.tag_distance = [0] * 10
        self.tag_roll = [0] * 10
        self.tag_pitch = [0] * 10

        self.uart.write(f"T{mode}\n")
        data = self.read_camera()

        if data[0] == "T":
            
            if (len(data) - 1) % 6 == 0 and len(data) > 1:
                num_tags = (len(data) - 1) // 6
                for i in range(num_tags):
                    tag_id = int(data[i * 6 + 1])
                    if tag_id in [0,1]:
                        self.tag_distance[tag_id] = 0.8136 * float(data[i * 6 + 4]) + 1.9753
                    else:
                        self.tag_distance[tag_id] = float(data[i * 6 + 4])
                    self.tag_detected[tag_id] = 1
                    self.tag_cx[tag_id] = int(data[i * 6 + 2])
                    self.tag_cy[tag_id] = int(data[i * 6 + 3])
                    self.tag_distance[tag_id] = float(data[i * 6 + 4])
                    self.tag_roll[tag_id] = float(data[i * 6 + 5])
                    self.tag_pitch[tag_id] = float(data[i * 6 + 6])
                    print(f"distance:{self.tag_distance[tag_id]}")
                    
# === ArmControllerクラス ===
class ArmController:
    def __init__(self, i2c):
        self.servos = Servos(i2c)
        self.servos.pca9685.freq(50)
        self.init_angles = [60, 120, 100, 150, 80]
        
        self.current_angles = self.init_angles[:]

    def move_servo(self, index, angle):
        angle = max(0, min(180, angle))
        self.servos.position(index, angle)
        self.current_angles[index] = angle

    def move_smoothly(self, index, target_angle, delay=0.025):
        step = 1 if target_angle > self.current_angles[index] else -1
        for angle in range(self.current_angles[index], target_angle, step):
            self.move_servo(index, angle)
            time.sleep(delay)
        self.move_servo(index, target_angle)

    def reset_position(self):
        for i, angle in enumerate(self.init_angles):
            self.move_smoothly(i, angle)

    def place_object(self):
        self.move_smoothly(0, 120)
        self.move_smoothly(1, 140)
        self.move_smoothly(3, 120)
        self.move_smoothly(2, 70)
               
        self.move_smoothly(4, 80)

    def search_position(self):
        self.move_smoothly(1, 90)
        self.move_smoothly(2, 100)
        self.move_smoothly(3, 110)
        self.move_smoothly(0, 60)
        self.move_smoothly(4, 80)

    def search_and_grab(self, cam, target_id):
        for angle in range(self.current_angles[3], 30, -2):
            self.move_servo(3, angle)
            cam.read_tags(0)
            
            if cam.tag_detected[target_id]:
                break

        while True:
            cam.read_tags(0)
            if cam.tag_cx[target_id] < 110:
                self.move_smoothly(0, self.current_angles[0] + 1)
            elif cam.tag_cx[target_id] > 130:
                self.move_smoothly(0, self.current_angles[0] - 1)
            else:
                break
            
        while True:
            miss_count = 0
            
            while True:
                cam.read_tags(0)
                
                if  cam.tag_detected[target_id] and cam.tag_distance[target_id] < 4:
                    print(cam.tag_distance[target_id])
                    self.move_servo(3, self.current_angles[3]+10)
                    self.move_servo(4, 0)
                    break
                               
                if cam.tag_detected[target_id]:
                    if cam.tag_cy[target_id] > 180:
                        self.move_servo(3, self.current_angles[3] - 1)
                    elif cam.tag_cy[target_id] < 140:
                        self.move_servo(3, self.current_angles[3] + 1)
                    else:
                        self.move_smoothly(1, self.current_angles[1] - 1)
                                       
                
                else:
                    miss_count += 1
                
                if miss_count > 5:
                    print("target out of range")
                    return False

                time.sleep(0.2)

# === MainSequenceクラス ===
class MainSequence:
    def __init__(self):
        self.arm = ArmController(I2C1)
        self.cam = CameraReceiver(UART1)
        
    def run(self):
        # 初期位置
        #self.arm.reset_position()
        #time.sleep(1)
        """self.arm.search_position()
        print(f"search position")
        time.sleep(1)"""
                                
        #self.arm.move_smoothly(4,0)
        #ime.sleep(2)
        
        # 地上局2の正面認識
        """while True:
            self.cam.read_tags(1)
            if self.cam.tag_detected[6]:
                break
            time.sleep(0.1)
            
        # 物資1を配置
        self.arm.place_object()"""

        # 物資2を探して回収
        self.arm.reset_position()
        time.sleep(0.1)
        #self.arm.search_position()
        while True:
            result = self.arm.search_and_grab(self.cam, 1)
            if not result:
                print("retry")
                self.arm.search_position()
            else:
                print("catch")
                break
            
        

#main
if __name__ == "__main__":
    main = MainSequence()
    main.run()

