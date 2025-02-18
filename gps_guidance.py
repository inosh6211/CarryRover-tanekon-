from machine import Pin, PWM, UART, I2C, Timer
from micropyGPS import MicropyGPS
from bno055 import *
import math
import time

PPR = 3              # PPR = CPR / 4
GEAR_RATIO = 297.92  # ギア比
FREQ = 20            # タイマー割り込みの周波数[hz]
KP_RPM = 0.1         # P制御の比例ゲイン

EARTH_RADIUS = 6378137                       # 地球の半径(m)
GOAL_LAT,GOAL_LON = 35.9186300, 139.9081696  # 7号館

class BNO055Handler:
    def __init__(self, i2c):
        self.i2c = i2c
        self.bno055 = BNO055(self.i2c)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0
        
    def get_calibration_status(self):
        """
        BNO055を完全に静止させる。
        BNO055を6方向に向けて静止させる
        BNO055を空中で8の字に回転させる
        """
        self.sys, self.gyro, self.accel, self.mag = self.bno055.cal_status()
        return self.bno055.calibrated()
        
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
    """
    [モーターの状態:state]
    STOP = 0
    FORWARD = 1
    TURN_RIGHT = 2
    TURN_LEFT  = 3
    BACKWARD = 4
    """
    def __init__(self, ain1, ain2, pwma, bin1, bin2, pwmb, stby, outa1, outb1, outa2, outb2):
        # モーター用ピンの設定 (Aが右, Bが左)
        self.AIN1 = Pin(ain1, Pin.OUT)
        self.AIN2 = Pin(ain2, Pin.OUT)
        self.PWMA = PWM(Pin(pwma))
        self.PWMA.freq(1000)
        self.BIN1 = Pin(bin1, Pin.OUT)
        self.BIN2 = Pin(bin2, Pin.OUT)
        self.PWMB = PWM(Pin(pwmb))
        self.PWMB.freq(1000)
        self.STBY = Pin(stby, Pin.OUT, value=1)

        # エンコーダ用ピンの設定
        self.OUTA_1 = Pin(outa1, Pin.IN)
        self.OUTB_1 = Pin(outb1, Pin.IN)
        self.OUTA_2 = Pin(outa2, Pin.IN)
        self.OUTB_2 = Pin(outb2, Pin.IN)

        # モーター制御用変数の設定
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

        # エンコーダの割り込み設定
        self.OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)
        
        # タイマーオブジェクトを生成
        self.timer = Timer()
    
    def pulse_counter_a(self, pin):
        if self.OUTA_1.value() == self.OUTB_1.value():
            self.direction_a = 1
        else:
            self.direction_a = -1
        self.pulse_count_a += self.direction_a

    def pulse_counter_b(self, pin):
        if self.OUTA_2.value() == self.OUTB_2.value():
            self.direction_b = 1
        else:
            self.direction_b = -1
        self.pulse_count_b += self.direction_b

    def forward(self):
        if self.state != 1:
            self.stop()
            
        self.AIN1.off()
        self.AIN2.on()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.on()
        self.BIN2.off()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = 1

    def turn_right(self):
        if self.state != 2:
            self.stop()
            
        self.AIN1.on()
        self.AIN2.off()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.on()
        self.BIN2.off()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = 2

    def turn_left(self):
        if self.state != 3:
            self.stop()
        self.AIN1.off()
        self.AIN2.on()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.off()
        self.BIN2.on()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = 3

    def backward(self):
        if self.state != 4:
            self.stop()
        self.AIN1.on()
        self.AIN2.off()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.off()
        self.BIN2.on()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = 4

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
        now = time.ticks_ms()
        interval = (now - self.start_time) / 1000
        rpm_a = self.compute_rpm(self.pulse_count_a, interval)
        rpm_b = self.compute_rpm(self.pulse_count_b, interval)
        self.rate_a += KP_RPM * (self.target_rpm_a - rpm_a)
        self.rate_b += KP_RPM * (self.target_rpm_b - rpm_b)
        self.rate_a = min(max(self.rate_a, 0), 100)
        self.rate_b = min(max(self.rate_b, 0), 100)
        self.start_time = now
        self.pulse_count_a = 0
        self.pulse_count_b = 0
    
    def update_rpm(self, target_rpm_a, target_rpm_b):
        self.target_rpm_a = target_rpm_a
        self.target_rpm_b = target_rpm_b

    # 割り込み有効化
    def enable_irq(self):
        self.OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)
        self.timer.init(mode=Timer.PERIODIC, freq=FREQ, callback=self.update_speed)
        self.start_time = time.ticks_ms()

    # 割り込み無効化
    def disable_irq(self):
        self.timer.deinit()
        self.OUTA_1.irq(handler=None)
        self.OUTA_2.irq(handler=None)

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
        return False

def gps_guidance(goal_lat, goal_lon):
    motor.update_rpm(30, 30)
    motor.enable_irq()
    motor_state = 0
    while True:
        gps.read_nmea()
        if gps.update_data(goal_lat, goal_lon):
            if gps.distance <= 5:
                motor.stop()
                print("stoped")
                break
            
        bno.compute_heading()
        azimuth_error = ((gps.azimuth - bno.heading + 180) % 360) - 180
        if azimuth_error > 20:
            motor.turn_right

        elif azimuth_error < -20:
            motor.turn_left()
            
        else:
            motor.forward()
            
        print(gps.distance, azimuth_error)
        time.sleep(0.01)

if __name__ == '__main__':
    try:
        motor = Motor(
            ain1=18, ain2=17, pwma=16,
            bin1=19, bin2=22, pwmb=28,
            stby=9,
            outa1=3, outb1=2,
            outa2=7, outb2=6
        )
        uart0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
        gps = GPS(uart0)
        i2c_0 = I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))
        bno = BNO055Handler(i2c=i2c_0)
    
        gps_guidance(GOAL_LAT, GOAL_LON)
        
    except KeyboardInterrupt:
        motor.stop()
        motor.disable_irq()
        print("stopped!")
