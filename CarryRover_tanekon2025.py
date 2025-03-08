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

# エンコーダピン設定
OUTA_A = Pin(3, Pin.IN)
OUTB_A = Pin(2, Pin.IN)
OUTA_B = Pin(7, Pin.IN)
OUTB_B = Pin(6, Pin.IN)


rpm = 50                # モーターのrpmを現地で測定
PPR         = 3         # パルス数 (PPR = CPR / 4)
GEAR_RATIO  = 297.92    # ギア比
FREQ        = 20        # タイマー割り込みの周波数 [Hz]
KP_RPM      = 0.2       # RPM制御の比例ゲイン
KP_YAW      = 0.1

# モーター状態
STOP       = 0 
FORWARD    = 1
TURN_R     = 2
TURN_L     = 3
BACKWARD   = 4

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

# 画像誘導
RED    = 0
BLUE   = 1
PURPLE = 2

KP_CAMERA = 0.05


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
        t0 = time.ticks_ms()
        motor.disable_irq()
        print(message)
        if self.p.is_connected():
            self.p.send(message)
            
        try:
            with open(self.file_name, "a") as f:
                f.write(message + "\n")
        except OSError as e:
            self.ble_print(f"SD card not detected: {e}")
            
        t1 = time.ticks_ms()
        dt = time.ticks_diff(t1, t0)
        motor.adjust_start_time(dt)
        motor.enable_irq()


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
                print("BNO055 キャリブレーション完了")
                break
            
            else:
                print(f"キャリブレーション gyro:{gyro}, mag:{mag}")
                
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
    
def soutai_turn(angle, init_yaw):#diffは右回り正の,init_yawからの角度の差を示し、angleはその中のdiffの角度をさし、そこに向かって回転する
    while True:                    #angleは右回り正で０から360
        current_yaw = (-bno.yaw + 360) % 360
        diff = ((current_yaw - init_yaw + 360) % 360)#((x - y + 360) % 360)はx,yが右回り正、0から360の時ｙをきじゅんとしてｘと角度差の角度差を0から360に変換する
        if ((angle - diff + 360) % 360) <= 180:#angleはたどり着きたい角度のinit_yawから右回り正のやつ
            while True:
                #print(diff)
                bno.compute_euler()
                current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
                diff = ((current_yaw - init_yaw + 360) % 360)
                turn_right(70)
                if angle-diff < -1:
                    turn_right(70)
                if abs(angle-diff) <= 1:
                    stop()
                    break
                time.sleep(0.01)
        elif ((angle - diff + 360) % 360) > 180:
            while True:
                print(diff)
                bno.compute_euler()
                current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
                diff = ((current_yaw - init_yaw + 360) % 360)
                turn_right(70)
                if angle-diff > 1:
                    turn_right(70)
                if abs(angle-diff) <= 1:
                    stop()
                    break
                time.sleep(0.01)
        if abs(angle-diff) <= 1:#358to2とかで359とかでとまったときにdiff >= 90認定されないように
            print("stop")
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
            diff = ((current_yaw - init_yaw + 540) % 360) - 180
            rate_a = -KP_YAW * diff + rpm
            rate_b = KP_YAW * diff + rpm
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
            diff = ((current_yaw - init_yaw + 540) % 360) - 180
            rate_a = KP_YAW * diff + rpm
            rate_b = -KP_YAW * diff + rpm
            backward(rate_a, rate_b)
            #print(f"L{diff}")
            now = time.ticks_ms()
            if (now - start) / 1000 >= t:
                stop()
                break
            time.sleep(0.01)  







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
        """
        while True:
            self.uart.write("1\n")
            
            while not self.uart.any():
                print("Waiting for Unitv signal...")
                time.sleep(0.5)
                
            message = self.uart.readline().decode('utf-8').strip()
            
            if message == "1":
                print("Unitv connected")
                break
            
            time.sleep(0.1)
            """

    def read_camera(self):
        message = ""
        
        while True:
            if self.uart.any():
                char = self.uart.read(1).decode('utf-8')
                if char == "\n":
                    break
                message += char
                
                time.sleep(0.01)
            
        return message.split(',')

    def read_color(self):
        # 0: Red, 1: Blue, 2: Purple
        self.color_pixels = [0] * 3
        self.color_cx = [0] * 3
        self.color_cy = [0] * 3
        
        self.uart.write("C\n")
        data = self.read_camera()
        
        if data[0] == "C":
            if (len(data) - 1) % 4 == 0:
                if len(data) > 1:
                    for i in range((len(data) - 1) // 4):
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
        
        time.sleep(0.1)  # カメラ側の応答時間を確保
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
        self.move_smoothly(2, 120)
        self.move_smoothly(3, 170)
        self.move_smoothly(0, 50)
        
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
                    if cam.tag_cx[tag_id] < 110:
                        self.move_smoothly(0, self.current_angles[0] + 1)
                        print(f"move left")
                    elif cam.tag_cx[tag_id] > 130:
                        self.move_smoothly(0, self.current_angles[0] - 1)
                        print(f"move right")
                    else:
                        break
                    catch = False    
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
                     
    def angle_fit(self, tag_id):
        search_direction = 1
        while True:
            result = self.search_and_grab(cam, tag_id)
            if result:
                print("catch")
                return True
            else:
                self.search_position1()
                print("retry")
                next_angle = self.current_angles[0] + (20 * search_direction)
                if next_angle >= 180:
                    next_angle = 180
                    search_direction = -1
                elif next_angle <= 0:
                    next_angle = 0
                    search_direction = 1

                self.move_servo(0, next_angle)
                time.sleep(0.5)
                self.search_position2()  # 再探索位置
    
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

#　パラシュート回避
def avoid_para():
    while True:
        cam.read_color()
        
        log.sd_write(f"Pixels: {cam.color_pixels}, Center: ({cam.color_cx}, {cam.color_cy})")
        time.sleep(0.1)

        if 0 < cam.color_pixels[2] <= 15000:
            if 0 <= cam.color_cx[2] <= 120:
                motor.update_rpm(30,30)
                motor.run(TURN_R)
            elif 120 < cam.color_cx[2] <= 240:
                motor.update_rpm(30,30)
                motor.run(TURN_L)
                    
        elif 15000 < cam.color_pixels[2]:#パラシュートが近いとき
            motor.stop()
            time.sleep(10)
                    
        else: 
            motor.update_rpm(30,30)
            motor.run(FORWARD)

# 目的地以外の地上局を避ける
def avoid_station():
    motor.enable_irq()
    motor.update_rpm(30, 30)
    motor.run(BACKWARD)
    time.sleep(1)
    motor.run(BACKWARD)
    time.sleep(1)
    motor.run(BACKWARD)
    time.sleep(1)
    motor.stop()
    
# GPS誘導(index=0で地上局0への誘導、index=1で地上局1への誘導)
def gps_guidance(index):
    station_color = [RED, BLUE]
    goal_lat, goal_lon = STATION[index]
    motor.enable_irq()
    
    motor.straight_forward_t(1, 30)
    
    while not gps.update_data(goal_lat, goal_lon):
        gps.read_nmea()
        log.ble_print("Waiting for GPS signal...")
        time.sleep(1)
    
    while True:
        gps.read_nmea()
        gps.update_data(goal_lat, goal_lon)
        bno.compute_heading()
        bno.compute_euler()
        azimuth_error = ((gps.azimuth - bno.heading + 180) % 360) - 180
        log.sd_write(f"Distance: {gps.distance}, Azimuth: {azimuth_error}")
        
        motor.run(TURN_R)
        
        if azimuth_error > 20:
            motor.update_rpm(10, 10)
            motor.run(TURN_R)
            
        elif azimuth_error < -20:
            motor.update_rpm(10, 10)
            motor.run(TURN_L)
        else:
            break
    
    avoid_para()
        
    while True:
        gps.read_nmea()
        gps.update_data(goal_lat, goal_lon)
        bno.compute_heading()
        bno.compute_euler()
        azimuth_error = ((gps.azimuth - bno.heading + 180) % 360) - 180
        log.sd_write(f"Distance: {gps.distance}, Azimuth: {azimuth_error}")
        
        cam.read_color()
        
        if cam.color_pixels[station_color[index - 2]] > 4000:
            log.ble_print("Detect another station!")
            avoid_station()
            
        if gps.distance < 10 and cam.color_pixels[station_color[index]]  > 200:
            log.sd_write("GPS guidance completed")
            motor.stop()
            break 
        
        if azimuth_error > 20:
            motor.update_rpm(10, 10)
            motor.run(TURN_R)
            
        elif azimuth_error < -20:
            motor.update_rpm(10, 10)
            motor.run(TURN_L)
        
        else:
            motor.update_rpm(30, 30)
            motor.run(FORWARD)
        
        time.sleep(0.1)

# 色認識による誘導(index=0で地上局0への誘導、index=1で地上局1への誘導)
def color_guidance(index):
        
    while True:
        cam.read_color()
        
        if cam.color_pixels[index] == 0:
            turn_right(70)# 旋回 
        else:
            cx = cam.color_cx[index]
            rpm_a = max(0, min(-KP_CAMERA * (cx - 120) + 30, 100))
            rpm_b = max(0, min(KP_CAMERA * (cx - 120) + 30, 100))
            forward(rpm_a, rpm_b)
                            
        if cam.color_pixels[index] > 10000:
            stop()
            break
        
        time.sleep(0.1)

# エイプリルタグに正対するプログラム
def apriltag_alignment(index):       
    while True:
        cam.read_tags(index)
        detected_id = None
        # 0～9 のタグIDをすべてチェック
        for i in range(10):  
            if cam.tag_detected[i]:
                detected_id = i
                break
                  # 最初に見つかったタグを使用
        
        if detected_id is not None:  
            corrected_distance = 424.115 + (2.5032 * cam.tag_distance[detected_id] - 1.1803)
            ka = cam.tag_pitch[detected_id]
            print(f"Tag {detected_id}: Distance = {corrected_distance}, Pitch = {ka}")
            # タグが検出されたのでループ終了（以降、正対などの処理に進む）
              
        else:
            print("No tag detected.")
            # タグが見つからなければ、右旋回して再探索
            turn_right(70)
            corrected_distance = None  
            ka = None
        
        time.sleep(0.5)

        if ka is not None and 10 <= ka <= 180:
            #20cmバックさせる
            back_distance =2000
            t_1 = back_distance / (135 * math.pi * (rpm /60))
            straight_ward("b", t_1)
            print("a")
            
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            soutai_turn(90 - ka , init_yaw)
            print("b")
            
            if corrected_distance is not None:
                go_distance = (corrected_distance + back_distance )* abs(math.sin(math.radians(ka)))
                t_2 = go_distance / (135 * math.pi * (rpm /60))
                straight_ward("f", t_2)
                soutai_turn(360 - ka, init_yaw)
                print("c")
                
                
            
        elif ka is not None and 180 <= ka <= 350:
            #20cmバックさせる
            back_distance =2000
            t_1 = back_distance / (135 * math.pi * (rpm /60))
            straight_ward("b", t_1)
            print("d")
            
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            soutai_turn( 90 - ka , init_yaw)
            print("e")
            
            if corrected_distance is not None:
                go_distance = (corrected_distance + back_distance )* abs(math.sin(math.radians(ka)))
                t_2 = go_distance / (135 * math.pi * (rpm /60))
                straight_ward("f", t_2)
                soutai_turn(ka, init_yaw)
                print("f")
                
# エイプリルタグ認識による誘導(index=0で地上局0への誘導、index=1で地上局1への誘導)
def apriltag_guidance(index):
    station_tag = [[2, 3, 5, 4], [6, 7, 9, 8]]
    target_ids = station_tag[index]
    # コーンの半径（mm）
    CONES_RADIUS = 300
    detected_id = None           
            
    # 直進：タグまでの距離が20cmになるまで前進   
    while True:
        cam.read_tags(index)
        if not cam.tag_detected[detected_id]:
            turn_right(70)
            time.sleep(0.5)
            continue
        
        if cam.tag_distance[detected_id] <= 20:
            stop()
        else:
            straight_ward("f", 0.5)
            
        time.sleep(0.05)
        
    # タグID による動作
    if detected_id in [2, 6]:
        print(f"Tag ID {detected_id}: In position. No further maneuver needed.")
    
    elif detected_id in [4, 8]:
        log.sd_write(f"Tag ID {detected_id}")
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        soutai_turn(90, init_yaw)  # 右90°旋回

        go_distance = CONES_RADIUS + 2000
        t = go_distance / (135 * math.pi * (rpm /60))
        straight_ward("f", t)
        soutai_turn(0, init_yaw)  # 左90°旋回
        
        straight_ward("f", t)
        soutai_turn(270, init_yaw)  # 左90°旋回
        
    elif detected_id in [3, 7]:
        log.sd_write(f"Tag ID {detected_id}")
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        soutai_turn(270, init_yaw)  # 左90°旋回

        go_distance = CONES_RADIUS + 2000
        t = go_distance / (135 * math.pi * (rpm /60))
        straight_ward("f", t)
        soutai_turn(0, init_yaw)  # 右90°旋回
        
        straight_ward("f", t)
        soutai_turn(90, init_yaw)  # 右90°旋回
        
    elif detected_id in [5, 9]:
        log.sd_write(f"Tag ID {detected_id}")
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        
        soutai_turn(90, init_yaw)  # 右90°旋回
        
        straight_ward("f", t)
        soutai_turn(0, init_yaw)  # 左90°旋回
        
        go_distance = 2 * CONES_RADIUS + 2000
        t = go_distance / (135 * math.pi * (rpm /60))
        straight_ward("f", t)
        soutai_turn(270, init_yaw)  # 左90°旋回

        straight_ward("f", t)
        soutai_turn(180, init_yaw)  # 左90°旋回
            
            
    
    
    
   
# 物資回収(index=0で地上局0での制御、index=1で地上局1での制御)
def collect_material(index):
    arm.search_position2()
    
    arm.angle_fit(index)
    time.sleep(0.5)
        
    arm.moving_position()
    time.sleep(1)
    

if __name__ == "__main__":
    log = Logger(SPI1, SPI1_CS)
    bno = BNO055Handler(I2C0)
    bme = BME280(I2C0)
    gps = GPS(UART0)
    cam = CameraReceiver(UART1)
    arm = ArmController(I2C1)
    
    
    log.sd_write("Setup completed")
    
    time.sleep(2)
    
    try:
        start()
        released()
        landing()
        fusing()
        gps_guidance(0)
        color_guidance(0)
        apriltag_alignment(0)
        apriltag_guidance(0)
        collect_material(0)
        gps_guidance(1)
        color_guidance(1)
        apriltag_alignment(1)
        apriltag_guidance(1)
        arm.place_object()
        color_guidance(0)
        gps_guidance(0)
        color_guidance(0)
        apriltag_alignment(0)
        apriltag_guidance(0)
        arm.place_object()
        
        
        log.sd_write("Mission completed!")
        
    finally:
        stop()
        time.sleep(1)
