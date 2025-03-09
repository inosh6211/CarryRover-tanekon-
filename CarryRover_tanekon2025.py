from machine import Pin, PWM, I2C, SPI, UART, Timer
from bno055 import *
from bme280 import BME280
from micropyGPS import MicropyGPS
from servo import Servos
from ble_simple_peripheral import BLESimplePeripheral
import os
import time
import math
import bluetooth
import sdcard

# シリアル通信のピン設定
I2C0 = I2C(0, sda=Pin(20), scl=Pin(21))
I2C1 = I2C(1, scl=Pin(27), sda=Pin(26))
SPI1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
SPI1_CS = Pin(13, Pin.OUT)
UART0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

FUSING_GPIO = Pin(8, Pin.OUT, value = 0) # 溶断回路のピン設定

# モータードライバピン設定
AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
BIN1 = Pin(19, Pin.OUT)
BIN2 = Pin(22, Pin.OUT)
PWMB = PWM(Pin(28))
PWMB.freq(1000)
STBY = Pin(9, Pin.OUT, value=1)

# GPS
EARTH_RADIUS = 6378137  # 地球の半径(m)
PDOP = 5                # PDOPの閾値
STATION = [
    (35.9180565, 139.9086761),  # 地上局0の経度緯度
    (35.9180565, 139.9086761)  # 地上局１の経度緯度
]

# 制御ログ
DEVICE_NAME = "RPpicoW"   # Bluetoorhのデバイス名
FILE_NAME = "CarryRover"  # ログを保存するファイル名

# ゲイン
KP_YAW      = 0.1
KP_CAMERA = 0.05

#　モーターのrpmを現地で測定
rpm = 30


class Logger:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.sd = None
        self.file_name = None
        self.sd_state = False
        self.ble = bluetooth.BLE()
        self.p = BLESimplePeripheral(self.ble, name=DEVICE_NAME)

        while not self.p.is_connected():
            print("Waiting for Bluetooth connection...")
            time.sleep(1)

        self.ble_print("Bluetooth connected!")
            
        time.sleep(1)
        try:
            self.sd = sdcard.SDCard(self.spi, self.cs)
            os.mount(self.sd, "/sd")
            self.create_file()
            self.sd_state = True
        except Exception as e:
            self.ble_print(f"SD card not detected: {e}")

        time.sleep(1)

    def create_file(self):
        counter = 0
        while True:
            file_name = f"/sd/{FILE_NAME}{counter}.txt"
            try:
                with open(file_name, "x") as f:
                    self.ble_print(f"File created: {FILE_NAME}{counter}.txt")         
                    self.file_name = file_name
                    break
            except OSError:
                counter += 1
                
    def ble_print(self, message):
        print(message)
        if self.p.is_connected():
            self.p.send(message)

    def sd_write(self, message):
        print(message)
        if self.p.is_connected():
            self.p.send(message)
            
        try:
            with open(self.file_name, "a") as f:
                f.write(message + "\n")
        except OSError as e:
            self.ble_print(f"SD card not detected: {e}")


class BNO055Handler:
    """
    [キャリブレーション]
    BNO055を完全に静止させる。
    BNO055を空中で8の字に回転させる
    """
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno055 = BNO055(self.i2c)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        
        while True:
            sys, gyro, accel, mag = self.bno055.cal_status()
            
            if gyro == 3 and mag == 3:
                log.ble_print("BNO055 calibration completed")
                break
            
            else:
                log.ble_print(f"BNO055 calibration gyro:{gyro}, mag:{mag}")
                
            time.sleep(1)
        
    def compute_euler(self):
        self.bno055.iget(QUAT_DATA)
        w = self.bno055.w / 16384
        x = self.bno055.x / 16384
        y = self.bno055.y / 16384
        z = self.bno055.z / 16384

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        yaw = math.atan2(t3, t4)

        self.roll = roll * 180 / math.pi
        self.pitch = pitch * 180 / math.pi
        self.yaw = yaw * 180 / math.pi

    def compute_heading(self):
        mag_x, mag_y, _ = self.bno055.mag()
        self.heading = math.atan2(mag_y, mag_x) * 180 / math.pi
        if self.heading < 0:
            self.heading += 360
    
    def get_accel(self):
        self.accel_x, self.accel_y, self.accel_z = self.bno055.accel()


class GPS:
    def __init__(self, uart):
        self.uart = uart
        self.micropygps = MicropyGPS(9, 'dd')
        self.updated_formats = set()
        self.lat = 0
        self.lon = 0
        self.distance = 0
        self.azimuth = 0
        
        while not self.update_data(0, 0):
            self.read_nmea()
            log.ble_print("Waiting for GPS signal...")
            time.sleep(1)
            
        log.ble_print("GPS signal received!")

    def read_nmea(self):
        while self.uart.any() > 0:
            sentence = self.uart.read()
            if sentence is None:
                return
            try:
                for char in sentence.decode('utf-8'):
                    gps_format = self.micropygps.update(char)
                    if gps_format:
                        self.updated_formats.add(gps_format)
            except Exception as e:
                log.ble_print(f"Error: {e}")

    def compute_distance(self, goal_lat, goal_lon):
        lat0, lon0, lat1, lon1 = map(math.radians, [self.lat, self.lon, goal_lat, goal_lon])
        dlat = lat1 - lat0
        dlon = lon1 - lon0
        a = math.sin(dlat / 2)**2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        self.distance = EARTH_RADIUS * c

    def compute_azimuth(self, goal_lat, goal_lon):
        lat0, lon0, lat1, lon1 = map(math.radians, [self.lat, self.lon, goal_lat, goal_lon])
        dlon = lon1 - lon0
        x = math.sin(dlon) * math.cos(lat1)
        y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
        azimuth_rad = math.atan2(x, y)
        self.azimuth = (math.degrees(azimuth_rad) + 360) % 360

    def update_data(self, goal_lat, goal_lon):
        if {'GNGGA', 'GNGSA'} <= self.updated_formats:
            self.updated_formats.clear()
            if self.micropygps.pdop < PDOP and self.micropygps.satellites_in_use > 3:
                try:
                    lat = self.micropygps.latitude[0]
                    lon = self.micropygps.longitude[0]
                    if lat is not None and lon is not None:
                        if self.lat != lat or self.lon != lon:
                            self.lat = lat
                            self.lon = lon
                            self.compute_distance(goal_lat, goal_lon)
                            self.compute_azimuth(goal_lat, goal_lon)
                            return True
                except IndexError as e:
                    log.ble_print(f"Error: {e}")
            
            else:
                log.ble_print("GPS low accuracy")
                log.ble_print(f"PDOP: {self.micropygps.pdop}, Satellites: {self.micropygps.satellites_in_use}")
                
        return False

class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart
        
        time.sleep(2)

        # UnitVとの接続確認
        while True:
            self.uart.write("1\n")
            
            while not self.uart.any():
                print("Waiting for Unitv signal...")
                time.sleep(0.5)
                
            message = self.uart.readline().decode('utf-8').strip()
            
            if message == "1":
                print("UnitV connected")
                break
            
            time.sleep(0.1)

    def read_camera(self):
        message = ""
        
        while True:
            if self.uart.any():
                char = self.uart.read(1).decode('utf-8')
                if char == "\n":
                    break
                message += char
        
        return message.split(',')
    
    def detect_para(self):
        self.para_pixels = 0
        self.para_cx = 0
        self.para_cy = 0
        
        self.uart.write("P\n")
        data = self.read_camera()
        
        if len(data) == 4:
            if data[0] == "P":
                self.para_pixels = int(data[1])
                self.para_cx = int(data[2])
                self.para_cy = int(data[3])

    def read_color(self):
        self.color_pixels = [0] * 2
        self.color_cx = [0] * 2
        self.color_cy = [0] * 2
        self.aspect_ratio = [0] * 2
        self.color_rotation = [0] * 2
        
        self.uart.write("C\n")
        data = self.read_camera()
        
        if data[0] == "C":
            if (len(data) - 1) % 6 == 0:
                if len(data) > 1:
                    for i in range((len(data) - 1) // 6):
                        color = int(data[i * 6 + 1])
                        self.color_pixels[color] = int(data[i * 6 + 2])
                        self.color_cx[color] = int(data[i * 6 + 3])
                        self.color_cy[color] = int(data[i * 6 + 4])
                        self.aspect_ratio[color] = float(data[i * 6 + 5])
                        self.color_rotation[color] = float(data[i * 6 + 6])
    
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
            if len(data) > 1:
                if (len(data) - 1) % 6 == 0:
                    for i in range((len(data) - 1) // 6):
                        tag_id = int(data[i * 6 + 1])
                        if tag_id < 10:
                            self.tag_detected[tag_id] = 1
                            self.tag_cx[tag_id] = int(data[i * 6 + 2])
                            self.tag_cy[tag_id] = int(data[i * 6 + 3])
                            self.tag_distance[tag_id] = float(data[i * 6 + 4])
                            self.tag_roll[tag_id] = float(data[i * 6 + 5])
                            self.tag_pitch[tag_id] = float(data[i * 6 + 6])


class ArmController:
    def __init__(self, i2c):
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
                
                time.sleep(0.1)
                while True:
                    cam.read_tags(0)
                    if cam.tag_detected[tag_id]:
                        self.Z = 10*cam.tag_distance[tag_id]
                        break
                            
                
        
    def moving_position(self):
        self.move_smoothly(1, 170)
        self.move_smoothly(2, 90)
        self.move_smoothly(3, 100)
        self.move_smoothly(0, 90)
        
    def back_position(self):
        self.move_smoothly(3,170)
        self.move_smoothly(2,170)
        
    def place_object(self):
        self.move_smoothly(0, 120)
        self.move_smoothly(1, 120)
        self.move_smoothly(4, 80)
        
    def search_position2(self):
        self.move_smoothly(1, 110)
        self.move_smoothly(2, 120)
        self.move_smoothly(3, 170)
        self.move_smoothly(0, 60)
        
    def search_for_target(self, cam, tag_id):
        found = False
        for angle in range(self.current_angles[3], 50, -1):
            self.move_servo(3,angle)
            cam.read_tags(0)
            
            if cam.tag_detected[tag_id]:
                found = True
                break
                time.sleep(0.2)
        if not found:
            self.search_position2()

            search_direction = 1
            search_angle_step = 2
            max_search_range = 60  
            fix_angle = 0

            for _ in range(int(max_search_range / search_angle_step)):  
                cam.read_tags(0)

                if cam.tag_detected[tag_id]:  
                    found = True
                    break


                fix_angle += search_angle_step
                fix_angle = min(fix_angle, max_search_range) 
                
                next_angle = self.current_angles[0] + (search_direction * fix_angle)

                next_angle = max(0, min(180, next_angle))

                self.move_smoothly(0, next_angle)
                search_direction *= -1  

                time.sleep(3)  # 探索間隔を少し開ける

        if found and cam.tag_detected[tag_id]:
            while True:
                cam.read_tags(0)
                if cam.tag_cx[tag_id] < 110:
                    self.move_smoothly(0, self.current_angles[0] + 1)
                elif cam.tag_cx[tag_id] > 130:
                    self.move_smoothly(0, self.current_angles[0] - 1)
                else:
                    break
                
    def inverse_kinematics(self, cam, tag_id):
        
        x_t = -self.L1 * math.cos(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.cos(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              (self.L3 + self.Z - 3) * math.cos(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))

        y_t = self.L1 * math.sin(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.sin(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) - \
              (self.L3 + self.Z - 3) * math.sin(math.radians(180 - (self.current_angles[2] - (170 - self.current_angles[1])) - self.current_angles[3]))
               
        d = math.sqrt(x_t**2 + y_t**2)

        L_12 = math.sqrt(self.L1**2 + self.L2**2 - 2 * self.L1 * self.L2 * math.cos(math.radians(self.current_angles[2])))
        theta3_prime = math.degrees(math.acos((self.L2**2 + L_12**2 - self.L1**2) / (2 * self.L2 * L_12)))
        
        theta5 = math.degrees(math.acos((L_12**2 + self.L3**2 - d**2) / (2 * L_12 * self.L3)))
        target_theta_3 = theta5 + theta3_prime
        
        self.move_smoothly(3, int(target_theta_3))
        time.sleep(1)
        
        x_b = -self.L1 * math.cos(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.cos(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              self.L3 * math.cos(math.radians((target_theta_3 + (self.current_angles[2] - (170 - self.current_angles[1]))) - 180))
        
        y_b = self.L1 * math.sin(math.radians(170 - self.current_angles[1])) + \
              self.L2 * math.sin(math.radians(self.current_angles[2] - (170 - self.current_angles[1]))) + \
              self.L3 * math.sin(math.radians((target_theta_3 + (self.current_angles[2] - (170 - self.current_angles[1]))) - 180))

        
        phi = int(math.degrees(math.acos((2*d**2 - ((x_t - x_b)**2 + (y_t - y_b)**2)) / (2*d**2))))
        
        self.move_smoothly(1, self.current_angles[1] - phi+5)
    
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
                    self.move_servo(3, self.current_angles[3]+5)
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
#motor
def forward(rate_a, rate_b):
    rate_a = max(0, min(rate_a, 100))
    rate_b = max(0, min(rate_b, 100))

    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(int(65535 * rate_a / 100))
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(int(65535 * rate_b / 100))
    
def backward(rate_a, rate_b):
    rate_a = max(0, min(rate_a, 100))
    rate_b = max(0, min(rate_b, 100))

    AIN1.on()
    AIN2.off()
    PWMA.duty_u16(int(65535 * rate_a / 100))
    BIN1.off()
    BIN2.on()
    PWMB.duty_u16(int(65535 * rate_b / 100))

def turn_right(rate):
    rate = max(0, min(rate, 100))

    AIN1.on()
    AIN2.off()
    PWMA.duty_u16(int(65535 * rate / 100))
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(int(65535 * rate / 100))
    
def turn_left(rate):
    rate = max(0, min(rate, 100))

    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(int(65535 * rate / 100))
    BIN1.off()
    BIN2.on()
    PWMB.duty_u16(int(65535 * rate / 100))

# 停止
def stop():
    AIN1.off()
    AIN2.off()
    BIN1.off()
    BIN2.off()
    
def soutai_turn(angle, init_yaw):#diffは右回り正の,init_yawからの角度の差を示し、angleはその中のdiffの角度をさし、そこに向かって回転する #angleは右回り正で０から360
    current_yaw = (-bno.yaw + 360) % 360
    diff = ((current_yaw - init_yaw + 360) % 360)
    print(((angle - diff + 180) % 360) - 180)#((x - y + 360) % 360)はx,yが右回り正、0から360の時ｙをきじゅんとしてｘと角度差の角度差を0から360に変換する
    if ((angle - diff + 180) % 360) - 180 > 0:#angleはたどり着きたい角度のinit_yawから右回り正のやつ
        while True:
            #print(diff)
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
            diff = ((current_yaw - init_yaw + 360) % 360)
            turn_right(20)
            #if angle-diff < -1:
                #turn_left(20)
            print(((angle - diff + 180) % 360) - 180)
            if ((angle - diff + 180) % 360) - 180 <= 0:
                stop()
                break
            time.sleep(0.01)
    elif ((angle - diff + 180) % 360) - 180 < 0:#angleはたどり着きたい角度のinit_yawから右回り正のやつ
        while True:
            #print(diff)
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
            diff = ((current_yaw - init_yaw + 360) % 360)
            turn_left(20)
            #if angle-diff < -1:
                #turn_left(20)
            if ((angle - diff + 180) % 360) - 180 >= 0:
                print(((angle - diff + 180) % 360) + 180)
                stop()
                break
            time.sleep(0.01)



def straight_ward(ward, t):#　t = distance / (135 * math.pi * (rpm /60))で、、現地で求めたrpmより、ｔを求めてから使う
    start = time.ticks_ms()
    bno.compute_euler()
    init_yaw = (-bno.yaw + 360) % 360
                
    if ward == "f":
         while True:
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360
            diff = ((current_yaw - init_yaw + 180) % 360) - 180
            rate_a = KP_YAW * diff + rpm
            rate_b = -KP_YAW * diff + rpm
            forward(rate_a, rate_b)
            #print(f"L{diff}")
            now = time.ticks_ms()
            if (now - start) / 1000 >= t:
                stop()
                break
            time.sleep(0.01)
                
    elif ward == "b":
        while True:
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360
            diff = ((current_yaw - init_yaw + 180) % 360) - 180
            rate_a = -KP_YAW * diff + rpm
            rate_b = KP_YAW * diff + rpm
            backward(rate_a, rate_b)
            #print(f"L{diff}")
            now = time.ticks_ms()
            if (now - start) / 1000 >= t:
                stop()
                break
            time.sleep(0.01)   

# スタート判定
def start():
    init_pressure = bme.pressure()
    
    while True:
        current_pressure = bme.pressure()
        diff_pressure = current_pressure - init_pressure
        bno.compute_euler()
        log.sd_write(f"Roll angle: {bno.roll}, pressure change: {diff_pressure}")
        
        if abs(bno.roll) > 45 and abs(bno.roll) < 135 and diff_pressure < -1:
            log.sd_write("Mission start!")
            break
        
        time.sleep(0.1)

# 放出判定
def released():
    count = 0
    
    while True:
        bno.compute_euler()
        roll = bno.roll
        log.sd_write(f"Roll angle: {roll}")
        
        if abs(roll) < 45:
          count += 1
        else:
          count = 0
          
        if count >= 5:
            log.sd_write("Released")
            break
        
        time.sleep(0.1)

# 着地判定
def landing():
    count = 0
    bno.compute_euler()
    init_roll = bno.roll
    init_pressure = bme.pressure()
    start_time = time.time()

    while True:
        bno.compute_euler()
        current_roll = bno.roll
        current_pressure = bme.pressure()
        diff_roll = current_roll - init_roll
        diff_pressure = current_pressure - init_pressure
        init_roll = current_roll
        init_pressure = current_pressure
        log.sd_write(f"Roll angle change: {diff_roll}, pressure change: {diff_pressure}")
        
        if abs(diff_roll) < 1 and abs(diff_pressure) < 0.1:
          count += 1
        else:
          count = 0
        
        now = time.time()
        elapsed_time = (now - start_time)
        
        if count == 5 or elapsed_time > 30:
            log.sd_write("Landing")
            break
        
        time.sleep(0.5)

# 溶断
def fusing():
    FUSING_GPIO.on()
    time.sleep(0.3)
    FUSING_GPIO.off()
    log.sd_write("Fusing completed")
    
    # サブキャリア脱出
    forward(100, 100)
    time.sleep(1)
    forward(30, 30)
    time.sleep(2)
    stop()

def avoid_para():
    cam.detect_para()
    if cam.para_pixels > 0:
        if cam.para_pixels > 10000:  # パラシュートが近いとき
            backward(30, 30)
            time.sleep(1)
            stop()
            
        if 0 <= cam.para_cx <= 120:
            turn_right(30)
            time.sleep(3)
            stop()
            forward(30, 30)
            time.sleep(3)
            stop()
            turn_left(30)
            time.sleep(3)
            stop()
            forward(30, 30)
            time.sleep(3)
            stop()
            
        elif 120 < cam.para_cx <= 240:
            turn_left(30)
            time.sleep(3)
            stop()
            forward(30, 30)
            time.sleep(3)
            stop()
            turn_right(30)
            time.sleep(3)
            stop()
            forward(30, 30)
            time.sleep(3)
            stop()

# GPS誘導(station_num=0で地上局0への誘導、station_num=1で地上局1への誘導)
def gps_guidance(station_num):        
    goal_lat, goal_lon = STATION[station_num]

    while not gps.update_data(goal_lat, goal_lon):
        gps.read_nmea()
        log.ble_print("Waiting for GPS signal...")
        time.sleep(1)
    
    gps.read_nmea()
    gps.update_data(goal_lat, goal_lon)
    bno.compute_heading()
    bno.compute_euler()
    azimuth_error = ((gps.azimuth - bno.heading + 180) % 360) - 180
    log.sd_write(f"Distance: {gps.distance}, Azimuth: {azimuth_error}")
    
    if abs(azimuth_error) > 20:
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        angle = (azimuth_error + 360) % 360
        soutai_turn(angle, init_yaw)

    avoid_para()
        
    while True:
        gps.read_nmea()
        gps.update_data(goal_lat, goal_lon)
        bno.compute_heading()
        bno.compute_euler()
        azimuth_error = ((gps.azimuth - bno.heading + 180) % 360) - 180
        log.sd_write(f"Distance: {gps.distance}, Azimuth: {azimuth_error}")
        
        cam.read_color()
        
        # 地上局検知
        if cam.color_pixels[station_color]  > 300 and cam.aspect_ratio[station_color] > 1.5 and 1 < cam.color_rotation[station_color] < 2:
            log.sd_write("GPS guidance completed")
            stop()
            break 
        
        if abs(azimuth_error) > 20:
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            angle = (azimuth_error + 360) % 360
            soutai_turn(angle, init_yaw)
        
        else:
            forward(30, 30)
        
        time.sleep(0.01)

# 色認識による誘導(index=0で地上局0への誘導、index=1で地上局1への誘導)
def color_guidance(index):
        
    while True:
        cam.read_color()
        
        if cam.color_pixels[index] == 0:
            turn_right(20)# 旋回
            time.sleep(0.2)
            stop()
            
        else:
            cx = cam.color_cx[index]
            rpm_a = max(0, min(-KP_CAMERA * (cx - 120) + 30, 100))
            rpm_b = max(0, min(KP_CAMERA * (cx - 120) + 30, 100))
            forward(rpm_a, rpm_b)
                            
        if cam.color_pixels[index] > 10000:
            stop()
            break
        
        time.sleep(0.1)

def find_apriltag(index, tag_mode):
    tag_id = [2, 6]
    ka = None
    
    while True:
        cam.read_tags(tag_mode)
        log.ble_print(f"{cam.tag_detected[index]}")
        if cam.tag_detected[tag_id[index]]:
            ka = (cam.tag_pitch[tag_id[index]] + 180) % 360 - 180
            corrected_distance = 424.115 + (2.5032 * cam.tag_distance[tag_id[index]] - 1.1803)
            if ka is not None and ka > 0:
                log.ble_print("tag detected. L")
                #10cmバックさせる
                back_distance =100
                t_1 = back_distance / (135 * math.pi * (rpm /60))
                straight_ward("b", t_1)
                log.ble_print("a")
                
                bno.compute_euler()
                init_yaw = (-bno.yaw + 360) % 360
                soutai_turn(90, init_yaw)
                log.ble_print("b")
                go_distance = (corrected_distance + back_distance )* abs(math.tan(math.radians(ka)))
                t_2 = go_distance / (135 * math.pi * (rpm /60))
                straight_ward("f", t_2)
                soutai_turn(360 - ka, init_yaw)
                
                t = 250 / (135 * math.pi * (rpm /60))
                straight_ward("b", t)
                cx = cam.tag_cx[tag_id[index]]
                print(cx)
                rpm_a = max(0, min(-KP_CAMERA * (cx - 120) + 30, 100))
                rpm_b = max(0, min(KP_CAMERA * (cx - 120) + 30, 100))
                forward(rpm_a, rpm_b)
                time.sleep(4)
                stop()
                break
                log.ble_print("c")
          
            
            elif ka is not None and ka < 0:
                #10cmバックさせる
                log.ble_print("tag detected. R")
                back_distance =100
                t_1 = back_distance / (135 * math.pi * (rpm /60))
                straight_ward("b", t_1)
                log.ble_print("d")
                
                bno.compute_euler()
                init_yaw = (-bno.yaw + 360) % 360
                soutai_turn(270, init_yaw)
                log.ble_print("e")
                go_distance = (corrected_distance + back_distance )* abs(math.tan(math.radians(ka)))
                t_2 = go_distance / (135 * math.pi * (rpm /60))
                straight_ward("f", t_2)
                soutai_turn(abs(ka), init_yaw)
                
                t = 250 / (135 * math.pi * (rpm /60))
                straight_ward("b", t)
                cx = cam.tag_cx[tag_id[index]]
                print(cx)
                rpm_a = max(0, min(-KP_CAMERA * (cx - 120) + 30, 100))
                rpm_b = max(0, min(KP_CAMERA * (cx - 120) + 30, 100))
                forward(rpm_a, rpm_b)
                time.sleep(4)
                stop()
                break
        
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        soutai_turn(70, init_yaw)
        forward(30, 30)
        time.sleep(0.5)
        stop()
        
        while True:
            cam.read_color()
            log.ble_print(f"{cam.color_pixels[index]}, {cam.color_cx[index]}")
            if cam.color_pixels[index] > 0 and cam.aspect_ratio[index] > 1.5 and 1 < cam.color_rotation[index] < 2:
                if cam.color_cx[index] >= 120:
                    break
                
            turn_left(30)
            time.sleep(0.1)
            stop()               

    
   
# 物資回収(index=0で地上局0での制御、index=1で地上局1での制御)
def collect_material(index):
    arm.search_position2()
    time.sleep(1)
    arm.search_for_target(cam,index)
    time.sleep(0.5)
    arm.detect_object(cam, index)
    arm.inverse_kinematics(cam, index)
    time.sleep(1)
    arm.moving_position()
    time.sleep(1)
    arm.back_position()

if __name__ == "__main__":
    log = Logger(SPI1, SPI1_CS)
    bno = BNO055Handler(I2C0)
    bme = BME280(i2c=I2C0)
    gps = GPS(UART0)
    cam = CameraReceiver(UART1)
    arm = ArmController(I2C1)
    
    
    log.sd_write("Setup completed")
    
    time.sleep(5)
    
    try:
        start()
        released()
        landing()
        fusing()
        gps_guidance(0)
        color_guidance(0)
        find_apriltag(0, 0)
        collect_material(0)
        gps_guidance(1)
        color_guidance(1)
        find_apriltag(1, 1)
        arm.place_object()
        collect_material(1)
        color_guidance(0)
        gps_guidance(0)
        color_guidance(0)
        find_apriltag(0, 0)
        arm.place_object()
        
        
        log.sd_write("Mission completed!")
        
    finally:
        stop()
        time.sleep(1)

