from machine import I2C, Pin, UART
from servo import Servos
import time
import math

# --- 各種設定 ---
I2C1 = I2C(1, scl=Pin(27), sda=Pin(26))
UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart
        time.sleep(0.5)
        
        """while True:
            self.uart.write("1\n")
            while not self.uart.any():
                print("waiiitng for unitv signal...")
                time.sleep(0.5)
                
            message = self.uart.readline().decode('utf-8').strip()
            if message == "1":
                print("unitV connected")
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
    
    def read_color(self):
            # 0: Red, 1: Blue, 2: Purple
            self.color_pixels = [0] * 3
            self.color_cx = [0] * 3
            self.color_cy = [0] * 3
            
            self.uart.write("C\n")
            data = self.read_camera()
            
            if data[0] == "C":
                if (len(data) % 4) - 1 == 0:
                    if len(data) > 1:
                        for i in range((len(data) - 1) / 4):
                            color = int(data[i * 4 + 1])
                            self.color_pixels[color] = int(data[i * 4 + 2])
                            self.color_cx[color] = int(data[i * 4 + 3])
                            self.color_cy[color] = int(data[i * 4 + 4])
    
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
                        self.tag_distance[tag_id] = 0.8136 * float(data[i * 6 + 4]) + 1.9753 - 2
                    #elif tag_id in [2,3,4,5,6,7,8,9]:
                    #    self.tag_distance[tag_id] = 
                    else:
                        self.tag_distance[tag_id] = float(data[i * 6 + 4])
                    self.tag_detected[tag_id] = 1
                    self.tag_cx[tag_id] = int(data[i * 6 + 2])
                    self.tag_cy[tag_id] = int(data[i * 6 + 3])
                    self.tag_roll[tag_id] = float(data[i * 6 + 5])
                    self.tag_pitch[tag_id] = float(data[i * 6 + 6])
                    print(f"distance:{10*self.tag_distance[tag_id]}")
                    time.sleep(0.1)
    
class Armcontroller:
    def __init__(self,i2c):
        self.servos = Servos(i2c)
        self.servos.pca9685.freq(50)
        self.L1 = 80
        self.L2 = 120
        self.L3 = 100
        self.Z = 0
        self.init_angles = [180, 170, 0, 0, 0]
        self.current_angles = self.init_angles[:]
        
        
    def move_servo(self, index, angle):
        angle=max(0,min(180,angle))
        self.servos.position(index, angle)
        self.current_angles[index] = angle
        
    def move_smoothly(self, index, target_angle, delay=0.025):
        if target_angle > self.current_angles[index]:
            step = 1
        else:
            step = -1
        for angle in range(self.current_angles[index], target_angle, step):
            self.move_servo(index, angle)
            time.sleep(delay)
        self.move_servo(index, target_angle)
        print(f"current = {self.current_angles[index]}")
        
    def reset_position(self):
        for i, angle in enumerate(self.init_angles):
            self.move_smoothly(i, angle)
                 
    def search_position1(self):
        self.move_smoothly(2, 110)
        self.move_smoothly(1, 90)
        self.move_smoothly(0, 80)
        self.move_smoothly(1, 170)
        self.move_smoothly(3, 110)
        self.move_smoothly(4, 80)
        
    def detect_object(self, cam, tag_id):
        """
        pitch = cam.tag_pitch[tag_id]
        print(f"Pitch = {pitch}")
        new_theta3 = self.current_angles[3] + pitch"""
        while True:
            cam.read_tags(0)
            if cam.tag_detected[tag_id]:
                CY = cam.tag_cy[tag_id]
                
                if CY > 180:
                    self.move_smoothly(3, self.current_angles[3] - 1)
                elif CY < 140:
                    self.move_smoothly(3, self.current_angles[3] + 1)
                else:
                    break
                
                                   
            
        
        #self.move_smoothly(3, int(new_theta3))
        time.sleep(0.1)
        while True:   
            cam.read_tags(0)
            if cam.tag_detected[tag_id]:
                self.Z = 10*cam.tag_distance[tag_id]
                print(f"distance = {self.Z}")
                break
        
        
    def moving_position(self):
        self.move_smoothly(1, 170)
        self.move_smoothly(2, 90)
        self.move_smoothly(3, 100)
        self.move_smoothly(0, 90)
        
    def place_object(self):
        self.move_smoothly(0, 120)
        self.move_smoothly(1, 120)
        self.move_smoothly(4, 80)
        
    def search_position2(self):
        self.move_smoothly(1, 110)
        self.move_smoothly(2, 100)
        self.move_smoothly(3, 110)
        self.move_smoothly(0, 60)       
                
    def search_for_target(self, cam, tag_id):
        for angle in range(self.current_angles[3], 0, -1):
            self.move_servo(3, angle)
            cam.read_tags(0)
            
            if cam.tag_detected[tag_id]:
                break

        while True:
            cam.read_tags(0)
            if cam.tag_cx[tag_id] < 110:
                self.move_smoothly(0, self.current_angles[0] + 1)
            elif cam.tag_cx[tag_id] > 130:
                self.move_smoothly(0, self.current_angles[0] - 1)
            else:
                break
        
    def inverse_kinematics(self, cam, tag_id):
        """ 物資に近づく """
        

        print(f"Z = {self.Z}")  # デバッグ用出力

        
        x_t = -self.L1 * math.cos(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.cos(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              (self.L3 + self.Z - 3) * math.cos(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))

        y_t = self.L1 * math.sin(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.sin(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) - \
              (self.L3 + self.Z - 3) * math.sin(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))
        
        
        print(f"l1 = {self.L1 * math.sin(math.radians(170 - self.current_angles[1]))}")
        print(f"l2 = {self.L2 * math.sin(math.radians(self.current_angles[2] - (170 - self.current_angles[1])))}")
        print(f"fai = {(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3])}")
        print(f"l3 =  {-(self.L3 + self.Z ) * math.sin(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))}")
        
        
        
        
        print(f"x_t={x_t}, y_t={y_t}")
        
        d = math.sqrt(x_t**2 + y_t**2)
        
        print(f"distance={d}")

        L_12 = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(math.radians(self.current_angles[2])))
        
        print(f"L_12={L_12}")
        theta3_prime = math.degrees(math.acos((self.L2**2 + L_12**2 - self.L1**2) / (2 * self.L2 * L_12)))
#       デバッグ
        #print(f"theta3'={theta3_prime}")
        #acos_val1 = (self.L2**2 + L_12**2 - self.L1**2) / (2 * self.L2 * L_12)
        #acos_val2 = (L_12**2 + self.L3**2 - d**2) / (2 * L_12 * self.L3)

        #print(f"acos_val1={acos_val1},acos_val2={acos_val2}")
        
        
        theta5 = math.degrees(math.acos((L_12**2 + self.L3**2 - d**2) / (2 * L_12 * self.L3)))
        print(f"theta5={theta5}")
        target_theta_3 = theta5 + theta3_prime
        print(f"Moving wrist to target θ3: {target_theta_3:.2f}°")
        
        
        self.move_smoothly(3, int(target_theta_3))
        time.sleep(1)
        
        x_b = -self.L1 * math.cos(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.cos(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              self.L3 * math.cos(math.radians((target_theta_3 + (self.current_angles[2] - (170 - self.current_angles[1]))) - 180))
        
        y_b = self.L1 * math.sin(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.sin(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              self.L3 * math.sin(math.radians((target_theta_3 + (self.current_angles[2] - (170 - self.current_angles[1]))) - 180))
        
        print(f"x_b={x_b}, y_b={y_b}")
        a = (x_t - x_b)**2 + (y_t - y_b)**2
        print(f"A = {a}")
        b = 2*d**2 - ((x_t - x_b)**2 + (y_t - y_b)**2)
        print(f"B = {b}")
        c = (2*d**2 - ((x_t - x_b)**2 + (y_t - y_b)**2)) / (2*d**2)
        print(f"C = {c}")
        
        
        
        phi = int(math.degrees(math.acos((2*d**2 - ((x_t - x_b)**2 + (y_t - y_b)**2)) / (2*d**2))))
        print(f"Φ = {phi}, angle_change = {self.current_angles[1] - phi+ 10}")
        
        self.move_smoothly(1, self.current_angles[1] - phi+5)
        
        # 徐々に上腕を曲げて近づく
        print("Moving upper arm towards the target...")
        
    
    def search_and_grab(self, cam, tag_id):
        miss_count = 0
        for angle in range(self.current_angles[3], 40, -1):
            self.move_servo(3, angle)
            cam.read_tags(0)
            print(cam.tag_detected[tag_id])
            if cam.tag_detected[tag_id]:
                break
                print(f"kenchi")
               
            while True:
                cam.read_tags(0)
                
                if  cam.tag_detected[tag_id] and cam.tag_distance[tag_id] < 4:
                    print(cam.tag_distance[tag_id])
                    self.move_servo(3, self.current_angles[3]+10)
                    self.move_smoothly(4, 0)
                    catch = True
                    return True
                
                               
                if cam.tag_detected[tag_id]:
                    
                    if cam.tag_cy[tag_id] > 180:
                        self.move_servo(3, self.current_angles[3] - 1)
                    elif cam.tag_cy[tag_id] < 140:
                        self.move_servo(3, self.current_angles[3] + 1)
                    else:
                        self.move_smoothly(1, self.current_angles[1] - 1)
                                       
                
                else:
                    miss_count += 1
                
                if miss_count > 50:
                    print("target out of range")
                    return False

                time.sleep(0.2)
            
                
                         
            
    def fix_wrist(self, cam, index):
        
        next_direction = 1
        while True:
            cam.read_tags(index)
            detect = cam.tag_detected[index]
            
            if detect:
                self.search_and_grab(index)
                break
                
            else:
                
                next_angle = self.current_angles[3] + (next_direction * 5)
                
                if next_angle >= 150:
                    next_angle = 30
                    next_direction = -1
                elif next_angle <= 30:
                    next_angle = 150
                    next_direction = 1
                    
                self.move_smoothly(3, next_angle)
                time.sleep(1)
                
                
        #search for targetの中のCyを修正する関数
        """catch = False    
        while True:
            cam.read_tags(0)
            miss_count = 0
            if  cam.tag_detected[target_id] and cam.tag_distance[target_id] < 4:
                print(cam.tag_distance[target_id])
                self.move_servo(3, self.current_angles[3]+10)
                self.move_smoothly(4, 0)
                catch = True
                return True
            
                           
            if cam.tag_detected[target_id]:
                
                if cam.tag_cy[target_id] > 180:
                    self.move_servo(3, self.current_angles[3] - 1)
                elif cam.tag_cy[target_id] < 140:
                    self.move_servo(3, self.current_angles[3] + 1)
                else:
                    self.move_smoothly(1, self.current_angles[1] - 1)
                                   
            
            else:
                miss_count += 1
            
            if miss_count > 50:
                print("target out of range")
                return False"""

            
                
            
    
    
        
class Mainsequence:
    def __init__(self):
        self.arm = Armcontroller(I2C1)
        self.cam = CameraReceiver(UART1)
    
    """def angle_fit(self, target_id):
        search_direction = 1
        while True:
            result = self.arm.search_and_grab(self.cam, target_id)
            if result:
                print("catch")
                return True
            else:
                print("retry")
                next_angle = self.arm.current_angles[0] + (20 * search_direction)
                if next_angle >= 180:
                    next_angle = 180
                    search_direction = -1
                elif next_angle <= 0:
                    next_angle = 0
                    search_direction = 1

                self.arm.move_servo(0, next_angle)
                time.sleep(0.5)
                self.arm.search_position2()  # 再探索位置"""

      
    
    def run(self):
        self.arm.reset_position()
        print(f"set initial position")
        time.sleep(1)

        self.arm.search_position1()
        print(f"set search position")
        time.sleep(1)
        
        while True:
            print("a")
            self.cam.read_tags(0)
            print(self.cam.tag_detected[2])
            if self.cam.tag_detected[2] and self.cam.tag_distance[2]<20:
                break
            time.sleep(0.5)
            print(self.cam.tag_detected[2])
        
        self.arm.search_position2()
        print(f"set search position2")
        
        self.arm.search_for_target(self.cam, 1)
        time.sleep(0.5)
        print(f"detect")
        
        #pitch = self.cam.tag_pitch[0]
        self.arm.detect_object(self.cam, 1)
        print(f"moving serov3...")
        time.sleep(0.2)
        
        print(f"next caluculating")
        self.arm.inverse_kinematics(self.cam, 1)
        print(f"finish approach")
        time.sleep(1)
        
        self.arm.move_smoothly(4,0)
       
        """self.arm.move_smoothly(4,0)
        self.arm.fix_wrist(self.cam, 1)
        
        self.arm.moving_position()
        print(f"moving position")
        time.sleep(1)
        
        while True:
            self.cam.read_tags(1)
            if self.cam.tag_detected[6] and self.cam.tag_distance[6]<20:
                break
            print(f"tag6 is detected")
            time.sleep(0.5)
            
            
        self.arm.place_object()
        time.sleep(1)
        print(f"set object")
        
        self.arm.search_position2()
        time.sleep(0.1)
        print(f"search position2")
        
        self.arm.search_for_target(self.cam, 1)
        time.sleep(0.5)
        
        pitch = self.cam.tag_pitch[1]
        self.arm.detect_object(pitch, 0)
        time.sleep(0.2)
        
        self.arm.inverse_kinematics(self.cam, 1)
        
        self.arm.fix_wrist(self.cam, 0)
        
        print(f"next moving")
            
        self.arm.moving_position()
        
        while True:
            self.cam.read_tags(1)
            if self.cam.tag_detected[2] and 10<self.cam.tag_distance[2]<20:
                break
            time.sleep(0.5)
            print(f"tag2 is detected")
            
        self.arm.place_object()
        time.sleep(1)
        print(f"complete")"""
    
    
    
if __name__ == "__main__":
    main = Mainsequence()
    main.run()
        
    
        

