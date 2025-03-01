from machine import I2C, Pin, UART
from servo import Servos
import time
import math

I2C1 = I2C(1, scl=Pin(27), sda=Pin(26))
UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))


class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart

    def read_camera(self):
        message = ""
        while not self.uart.any():
            time.sleep(0.01)
                
        while self.uart.any() > 0: 
            message += self.uart.read().decode('utf-8').strip()
            
        data = message.split(',')
        
        return data

    def read_color(self):
        self.uart.write("c\n")
        data = self.read_camera()
        
        if data[0] == "c":
            self.color_pixels = []
            self.color_cx = []
            self.color_cy = []
            
            for color in range(3):
                self.color_pixels.append(float(data[color*3 + 1]))
                self.color_cx.append(float(data[color*3 + 2]))
                self.color_cy.append(float(data[color*3 + 3]))

    def read_tag(self):
        self.uart.write("t\n")    
        data = self.read_camera()
        
        if data[0] == "t":
            self.tag_detected = []
            self.tag_cx = []
            self.tag_cy = []
            self.tag_distance = []
            self.tag_pitch = []
            
            for tag_id in range(10):
                self.tag_detected.append(int(data[tag_id*5 + 1]))
                self.tag_cx.append(float(data[tag_id*5 + 2]))
                self.tag_cy.append(float(data[tag_id*5 + 3]))
                self.tag_distance.append(abs(float(data[tag_id*5 + 4])))
                self.tag_pitch.append(float(data[tag_id*5 + 5]))
                
                
class ArmController:
    def __init__(self, i2c):
        self.servos = Servos(i2c)
        self.servos.pca9685.freq(50)
        self.CX=120
        self.CY=160

        # アーム長
        self.L1 = 70  # 上腕
        self.L2 = 120 # 前腕
        self.L3 = 100 # 手首
        
        # アームの初期位置
        self.init_angles = [110, 170, 0, 10, 0]
        self.current_angles = self.init_angles[:]
        """
        for i in range(5):
            self.servos.position(i, self.init_angles[i])
        """
    
    def move_smoothly(self, index, target_angle):
        current_angle = self.current_angles[index]
        
        while current_angle != target_angle:
            if current_angle < target_angle:
                current_angle += 1
            elif current_angle > target_angle:
                current_angle -= 1
                
            self.servos.position(index, current_angle)
            time.sleep(0.02)
            
            self.current_angles[index] = current_angle
  

if __name__ == "__main__":
    cam = CameraReceiver(UART1)
    arm = ArmController(I2C1)
    
    arm.servos.position(0, 0)
    arm.servos.position(1, 130)
    arm.servos.position(2, 80)
    arm.servos.position(3, 140)
    arm.servos.position(4, 100)
    
    time.sleep(1)
    arm.current_angles = [0, 130, 80, 140, 100]
    arm.move_smoothly(2, 80)
    arm.move_smoothly(1, 130)
    arm.move_smoothly(0, 0)
    arm.move_smoothly(3, 140)
    arm.move_smoothly(4, 100) 
    
    current_angle3 = arm.current_angles[3]
    for angle in range(current_angle3, 30, -2):
        arm.servos.position(3, angle)
        arm.current_angles[3] = angle
        cam.read_tag()
        if cam.tag_detected[0]:
            if cam.tag_cy[0] <= 160:
                break
        
    if cam.tag_cx[0] < 120:
        print("左！")
        current_angle0 = arm.current_angles[0]
        for angle in range(current_angle0, 180, 1):
            arm.servos.position(0, angle)
            arm.current_angles[0] = angle
            cam.read_tag()
            if cam.tag_detected[0]:
                print(cam.tag_cx[0])
                if cam.tag_cx[0] >= 120:
                    print(cam.tag_cx[0])
                    print(cam.tag_distance[0])
                    break
            time.sleep(0.1)
                
    elif cam.tag_cx[0] > 120:
        print("右！")
        current_angle0 = arm.current_angles[0]
        for angle in range(current_angle0, 180, 1):
            arm.servos.position(0, angle)
            arm.current_angles[0] = angle
            cam.read_tag()
            if cam.tag_detected[0]:
                if cam.tag_cx[0] >= 120:
                    print(cam.tag_cx[0])
                    print(cam.tag_distance[0])
                    break
            time.sleep(0.1)
    
    current_angle1 = arm.current_angles[1]
    for angle in range(current_angle1, 0, -1):
        arm.servos.position(1, angle)
        arm.current_angles[1] = angle
        cam.read_tag()
        print(cam.tag_detected[0])
        print(cam.tag_distance[0])
        print(cam.tag_cy[0])
        if cam.tag_distance[0] < 4:
            break
        
        elif cam.tag_cy[0] > 180:
            current_angle3 = arm.current_angles[3]
            for angle in range(current_angle3, 0, -1):
                arm.servos.position(3, angle)
                arm.current_angles[3] = angle
                cam.read_tag()
                print(cam.tag_detected[0])
                print(cam.tag_cy[0])
                if cam.tag_cy[0] <= 160:
                    break
                
                time.sleep(0.2)
                
        elif cam.tag_cy[0] < 140:
            current_angle3 = arm.current_angles[3]
            for angle in range(current_angle3, 180, 1):
                arm.servos.position(3, angle)
                arm.current_angles[3] = angle
                cam.read_tag()
                print(cam.tag_detected[0])
                print(cam.tag_cy[0])
                if cam.tag_cy[0] >= 160:
                    break
                
                time.sleep(0.2)
                
        time.sleep(0.2)
        
    arm.move_smoothly(3, arm.current_angles[3] + 10) 
    arm.move_smoothly(4, 0) 
