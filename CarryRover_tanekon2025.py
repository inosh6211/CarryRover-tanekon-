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
        

class Motor:
    def __init__(self):
        self.AIN1 = AIN1
        self.AIN2 = AIN2
        self.PWMA = PWMA
        self.BIN1 = BIN1
        self.BIN2 = BIN2
        self.PWMB = PWMB
        self.STBY = STBY
        self.OUTA_A = OUTA_A
        self.OUTB_A = OUTB_A
        self.OUTA_B = OUTA_B
        self.OUTB_B = OUTB_B

        # 制御用変数
        self.rate_a = 0
        self.rate_b = 0
        self.target_rpm_a = 0
        self.target_rpm_b = 0
        self.pulse_count_a = 0
        self.pulse_count_b = 0
        self.direction_a = 0
        self.direction_b = 0
        self.start_time = time.ticks_ms()
        self.state = 0
        
        self.timer = Timer()  # タイマーオブジェクトを生成
        

    def pulse_counter_a(self, pin):
        if self.OUTA_A.value() == self.OUTB_A.value():
            self.direction_a = 1
        else:
            self.direction_a = -1
        self.pulse_count_a += self.direction_a

    def pulse_counter_b(self, pin):
        if self.OUTA_B.value() == self.OUTB_B.value():
            self.direction_b = 1
        else:
            self.direction_b = -1
        self.pulse_count_b += self.direction_b

    def run(self, state):
        if self.state != state:
            self.stop()
            self.state = state

        if self.state == FORWARD:
            self.AIN1.off()
            self.AIN2.on()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.on()
            self.BIN2.off()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))

        elif self.state == TURN_R:
            self.AIN1.on()
            self.AIN2.off()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.on()
            self.BIN2.off()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))

        elif self.state == TURN_L:
            self.AIN1.off()
            self.AIN2.on()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.off()
            self.BIN2.on()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))

        elif self.state == BACKWARD:
            self.AIN1.on()
            self.AIN2.off()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.off()
            self.BIN2.on()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))

    def stop(self):
        self.AIN1.off()
        self.AIN2.off()
        self.BIN1.off()
        self.BIN2.off()
        self.rate_a = 20
        self.rate_b = 20
        self.state = 0
    
    def run_straight(distance, rpm):  # 距離はm単位
        start = time.ticks_ms()
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        motor.update_rpm(rpm, rpm)
        while True:
            motor.run(FORWARD)
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360
            diff = ((current_yaw - init_yaw + 180) % 360) - 180
            rpm_a = -KP_YAW * diff + rpm
            rpm_b = KP_YAW * diff + rpm
            motor.update_rpm(rpm_a, rpm_b)
            motor.run(FORWARD)
            now = time.ticks_ms()
            if (now - start_time) / 1000 >= distance / (0.135 * math.pi() * rpm / 60) :
                break
                    
            time.sleep(0.1)
    
    # 正：右回転、負：左回転
    def turn_by_angle(angle, rpm):
        bno.compute_euler()
        init_yaw = (-bno.yaw + 360) % 360
        motor.update_rpm(rpm, rpm)
        while True:
            if angle > 0
                motor.run(TURN_R)
            else:
                motor.run(TURN_L)
                
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360
            diff = ((current_yaw - init_yaw + 180) % 360) - 180
                
            if (angle > 0 and diff >= angle) or (angle < 0 and diff <= angle):
                motor.stop()
                break
            
            time.sleep(0.1)

    def compute_rpm(self, pulse_count, interval):
        if interval <= 0:
            return 0
        rpm = abs((pulse_count * 60) / (PPR * GEAR_RATIO * interval))
        return rpm

    def update_speed(self, timer):        
        if self.state == 0:
            self.start_time = time.ticks_ms()
            self.pulse_count_a = 0
            self.pulse_count_b = 0
        else:
            now = time.ticks_ms()
            interval = time.ticks_diff(now, self.start_time) / 1000
            rpm_a = self.compute_rpm(self.pulse_count_a, interval)
            rpm_b = self.compute_rpm(self.pulse_count_b, interval)
            self.rate_a += KP_RPM * (self.target_rpm_a - rpm_a)
            self.rate_b += KP_RPM * (self.target_rpm_b - rpm_b)
            self.rate_a = min(max(self.rate_a, 0), 100)
            self.rate_b = min(max(self.rate_b, 0), 100)

            self.run(self.state)

            self.start_time = now
            self.pulse_count_a = 0
            self.pulse_count_b = 0

    def update_rpm(self, target_rpm_a, target_rpm_b):
        self.target_rpm_a = target_rpm_a
        self.target_rpm_b = target_rpm_b

    def adjust_start_time(self, dt):
        self.start_time = time.ticks_add(self.start_time, dt)
        
    def inverted(self):
        bno.get_accel()
        pitch = bno.pitch
        accel_z = bno.accel_z
        if accel_z < 0 or pitch < -20:
            self.update_rpm(1000, 1000)
            self.run(FORWARD)
            time.sleep(0.5)
            self.stop()

    def enable_irq(self):
        self.OUTA_A.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_B.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)
        self.timer.init(mode=Timer.PERIODIC, freq=FREQ, callback=self.update_speed)

    def disable_irq(self):
        self.OUTA_A.irq(handler=None)
        self.OUTA_B.irq(handler=None)
        self.timer.deinit()


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
        
        """
        # UnitVとの接続確認
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
        data = self.read_camera()

        if data[0] == "T":
            if len(data) > 1:
                if (len(data) - 1) % 6 == 0:
                    for i in range((len(data) - 1) // 6):
                        tag_id = int(data[i * 6 + 1])
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
        
        # アームの初期位置
        self.init_angles = [110, 170, 0, 10, 0]
        self.current_angles = self.init_angles[:]
        
        for i in range(5):
            self.servos.position(i, self.init_angles[i])
            
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
    
    def move_servo(self, index, target_angle):
        arm.servos.position(index, target_angle)
        arm.current_angles[index] = target_angle
    
    
def start():
    init_pressure = bme.pressure()
    
    while True:
        current_pressure = bme.pressure()
        diff_pressure = current_pressure - init_pressure
        bno.compute_euler()
        
        if abs(bno.roll) > 45 and abs(bno.roll) < 135 and diff_pressure < -1:
            log.sd_write("Mission start!")
            break
        else:
            log.sd_write(f"Roll angle: {bno.roll}, pressure change: {diff_pressure}")
        
        time.sleep(0.1)

def released():
    count = 0
    
    while True:
        bno.compute_euler()
        roll = bno.roll
        
        if abs(roll) < 45:
          count += 1
        else:
          count = 0
          
        if count >= 5:
            log.sd_write("Released")
            break
        
        log.sd_write(f"Roll angle: {roll}")
        time.sleep(0.1)

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
        
        if abs(diff_roll) < 1 and abs(diff_pressure) < 0.1:
          count += 1
        else:
          count = 0
        
        now = time.time()
        elapsed_time = (now - start_time)
        
        if count == 5 or elapsed_time > 30:
            log.sd_write("Landing")
            break
        
        log.sd_write(f"Roll angle change: {diff_roll}, pressure change: {diff_pressure}")
        time.sleep(0.5)

def fusing():
    FUSING_GPIO.on()
    time.sleep(0.3)
    FUSING_GPIO.off()
    log.sd_write("Fusing completed")

def avoid_para():
    while True:
        camera_data = read_camera()
        if camera_data is None:
            pixels, cx, cy = 0, 0, 0  # デフォルト値
        else:
            pixels, cx, cy = camera_data

        print(f"Pixels: {pixels}, Center: ({cx}, {cy})")
        time.sleep(0.1)

        if 0 < pixels <= 10000:
            if 0 <= cx <= 120:
                motor.update_rpm(30,30)
                motor.run(TURN_RIGHT)
            elif 120 < cx <= 240:
                motor.update_rpm(30,30)
                motor.run(TURN_LEFT)
            
        elif 10000 < pixels: # パラシュートが近いとき
            motor.stop()
            time.sleep(5)
            
        else: 
            motor.update_rpm(30,30)
            motor.run(FORWARD)

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
    
def gps_guidance(index):
    station_color = [RED, BLUE]
    goal_lat, goal_lon = STATION[index]
    motor.enable_irq()
    
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
    
def color_guidance(index):
    motor.enable_irq()
        
    while True:
        cam.read_color()
        
        if cam.color_pixels[index] == 0:
            motor.update_rpm(10, 10)
            motor.run(TURN_R)  # 旋回 
        else:
            cx = cam.color_cx[index]
            rpm_a = max(0, min(-KP_CAMERA * (cx - 120) + 30), 100)
            rpm_b = max(0, min(KP_CAMERA * (cx - 120) + 30), 100)
            motor.update_rpm(rpm_a, rpm_b)
            motor.run(FORWARD)
                       
        if cam.color_pixels[index] > 4000:
            motor.update_rpm(30,30)
            motor.run(FORWARD)
            time.sleep(0.5)
            motor.stop()
            motor.disable_irq()
            break
        
        time.sleep(0.1)

def apriltag_guidance(index):
    station_tag = [[2, 3, 5, 4], [6, 7, 9, 8]]
    motor.enable_irq()
    
    while True:
        cam.read_tags(0)
        
        while not any(cam.tag_detected[i] for i in station_tag[index]):
            bno.compute_euler()
            init_yaw = bno.yaw
            motor.update_rpm(15, 15)
            motor.run(TURN_R)  # 旋回
            time.sleep(0.3)
            motor.stop()
            cam.read_tags(0)
            
        if cam.tag_detected[6]:
            cx = cam.tag_cx[6]
            rpm_a = constrain(-KP_CAMERA * (cx - 120) + 20 , 0, 100)
            rpm_b = constrain(KP_CAMERA * (cx - 120) + 20 , 0, 100)
            motor.update_rpm(rpm_a, rpm_b)
            motor.run(FORWARD)
            
            if cam.tag_distance[6] < 8:
                motor.stop()
                motor.disable_irq()
                break
    
def collect_material():
    """物資回収の関数を書く"""

def place_material():
    """物資設置の関数を書く"""
    

if __name__ == "__main__":
    log = Logger(SPI1, SPI1_CS)
    bno = BNO055Handler(I2C0)
    bme = BME280(I2C0)
    motor = Motor()
    gps = GPS(UART0)
    cam = CameraReceiver(UART1)
    arm = ArmController(I2C1)
    
    log.sd_write("Setup completed")
    
    try:
        start()
        relaesed()
        landing()
        fusing()
        avoid_para()
        gps_guidance(0)
        color_guidance(0)
        apriltag_guidance(0)
        # アームによる物資回収
        gps_guidance(1)
        color_guidance(1)
        apriltag_guidance(1)
        # アームによる物資設置
        # アームによる物資回収
        gps_guidance(0)
        color_guidance(0)
        apriltag_guidance(0)
        # アームによる物資設置
        
    finally:
        motor.stop()
        motor.disable_irq()
        time.sleep(1)


