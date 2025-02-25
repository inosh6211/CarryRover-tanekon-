from machine import Pin, PWM, UART, I2C, SPI, Timer
from micropyGPS import MicropyGPS
from ble_simple_peripheral import BLESimplePeripheral
from bno055 import *
import math
import time
import os
import bluetooth
import sdcard

PPR = 3              # PPR = CPR / 4
GEAR_RATIO = 297.92  # ギア比
FREQ = 20            # タイマー割り込みの周波数[hz]
KP_RPM = 0.2         # P制御の比例ゲイン

EARTH_RADIUS = 6378137                       # 地球の半径(m)
GOAL_LAT, GOAL_LON = 35.9180565, 139.9086761 # 閲覧室

DEVICE_NAME = "RPpicoW"
FILE_NAME = "CarryRover"

I2C0 = I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))
SPI1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
SPI1_CS = Pin(13, Pin.OUT)
UART0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

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

# モーターの状態
STOP     = 0
FORWARD  = 1
TURN_R   = 2
TURN_L   = 3
BACKWARD = 4


class Logger:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.sd = None
        self.file_name = None
        self.sd_state = False
        self.ble = BLESimplePeripheral(bluetooth.BLE(), name=DEVICE_NAME)

        while not self.ble.is_connected():
            print("Waiting for Bluetooth connection...")
            time.sleep(1)

        print("Bluetooth connected")
        time.sleep(1)
        try:
            self.sd = sdcard.SDCard(self.spi, self.cs)
            os.mount(self.sd, "/sd")
            self.create_file()
            self.sd_state = True
        except Exception as e:
            print(f"SD card not detected: {e}")
            if self.ble.is_connected():
                self.ble.send(f"SD card not detected: {e}")

        time.sleep(1)

    def create_file(self):
        counter = 0
        while True:
            file_name = f"/sd/{FILE_NAME}{counter}.txt"
            try:
                with open(file_name, "x") as f:
                    print(f"File created: {FILE_NAME}{counter}.txt")
                    
                    if self.ble.is_connected():
                        self.ble.send(f"File created: {FILE_NAME}{counter}.txt")
                    self.file_name = file_name
                    return file_name
            except OSError:
                counter += 1

    def log(self, message):
        t0 = time.ticks_ms()
        motor.disable_irq()
        print(message)
        if self.ble.is_connected():
            self.ble.send(message)
            
        try:
            with open("/sd/data.txt", "a") as f:
                f.write(message + "\n")
        except OSError as e:
            print("SD card write error:", e)
            
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
        
        while True:
            self.sys, self.gyro, self.accel, self.mag = self.bno055.cal_status()
            
            if self.gyro == 3 and self.mag == 3:
                print("calibration")
                break
            
            else:
                print(f"gyro:{self.gyro}, mag:{self.mag}")
                
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

        # エンコーダ割り込みの設定（OUTA_A と OUTA_B に設定）
        self.OUTA_A.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_B.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)

        # タイマーオブジェクトを生成
        self.timer = Timer()

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

    def read_nmea(self):
        if self.uart.any() > 0:
            sentence = self.uart.read()
            if sentence is None:
                return
            try:
                for char in sentence.decode('utf-8'):
                    gps_format = self.micropygps.update(char)
                    if gps_format:
                        self.updated_formats.add(gps_format)
            except Exception as e:
                print(f"Error: {e}")

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
            if self.micropygps.pdop < 3 and self.micropygps.satellites_in_use > 3:
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
                    print(f"Error: {e}")
            
            else:
                print("Low accuracy")
                
        return False


def gps_guidance(goal_lat, goal_lon):
    motor.enable_irq()
    
    while not gps.update_data(goal_lat, goal_lon):
        gps.read_nmea()
        print("Wait GPS signal...")
        time.sleep(1)
            
    while True:
        gps.read_nmea()
        gps.update_data(goal_lat, goal_lon)
        bno.compute_heading()
        bno.compute_euler()
        azimuth_error = ((gps.azimuth - bno.heading + 180) % 360) - 180
        log.log(f"Distance:{gps.distance}, Azimuth:{azimuth_error}")
        
        if gps.distance <= 5:
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

if __name__ == '__main__':
    try:
        log = Logger(SPI1, SPI1_CS)
        motor = Motor()
        gps = GPS(UART0)
        bno = BNO055Handler(i2c=I2C0)
        
        gps_guidance(GOAL_LAT, GOAL_LON)
        
    finally:
        motor.stop()
        motor.disable_irq()
        print("stopped!")
        time.sleep(1)
