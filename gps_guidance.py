from machine import Pin, PWM, I2C
from bno055 import *
from micropyGPS import MicropyGPS
import math
import time

# GPSの設定
uart0 = UART(0, baudrate=9600, tx=0, rx=1)
gps = MicropyGPS(9, 'dd')
updated_formats = set()
gps_updated = 0
lat = 0
lon = 0
GOAL_LAT, GOAL_LON = 35.7171709, 139.8232740

# BNO055の設定
i2c = I2C(0, sda = Pin(20), scl = Pin(21))
bno = BNO055(i2c)

# モーターの設定
AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
BIN1 = Pin(19, Pin.OUT)
BIN2 = Pin(22, Pin.OUT)
PWMB = PWM(Pin(28))
PWMB.freq(1000)
STBY = Pin(9, Pin.OUT, value=1)

# エンコーダの設定
OUTA_1 = Pin(3, Pin.IN)
OUTB_1 = Pin(2, Pin.IN)
OUTA_2 = Pin(7, Pin.IN)
OUTB_2 = Pin(6, Pin.IN)
PPR = 3  # PPR = CPR / 4
GEAR_RATIO = 297.92
pulse_count_A = 0
direction_A = 0
pulse_count_B = 0
direction_B = 0
KP_RPM = 1
TARGET_RPM = 30

def read_nmea():
    if uart0.any() > 0:
        sentence = uart0.read()
        if sentence:
            try:
                for char in sentence.decode('utf-8'):
                    gps_format = gps.update(char) 
                    if gps_format:
                        updated_formats.add(gps_format)
            except Exception as e:
                print(f"Error: {e}")

def get_lat_lon():
    global gps_updated, lat, lon
    if {'GNGGA', 'GNGSA'} <= updated_formats:
        updated_formats.clear()
        if gps.pdop < 3 and gps.satellites_in_use > 3:
            if lat != gps.latitude[0] or lon != gps.longitude[0]:
                gps_updated = 1
                lat = gps.latitude[0]
                lon = gps.longitude[0]
    else:
        gps_updated = 0

def compute_distance():
    EARTH_RADIUS = 6378137
    lat0, lon0, lat1, lon1 = map(math.radians, [lat, lon, GOAL_LAT, GOAL_LON])
    dlat = lat1 - lat0
    dlon = lon1 - lon0
    a = math.sin(dlat / 2)**2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

def compute_azimuth():
    lat0, lon0, lat1, lon1 = map(math.radians, [lat, lon, GOAL_LAT, GOAL_LON])
    dlon = lon1 - lon0
    x = math.sin(dlon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
    azimuth_rad = math.atan2(x, y)
    return (math.degrees(azimuth_rad) + 360) % 360

# エンコーダAのパルスカウント処理
def pulse_counter_A(pin):
    global pulse_count_A, direction_A
    if OUTA_1.value() == OUTB_1.value():
        direction_A = 1
    else:
        direction_A = -1
    pulse_count_A += direction_A

# エンコーダBのパルスカウント処理
def pulse_counter_B(pin):
    global pulse_count_B, direction_B
    if OUTA_2.value() == OUTB_2.value():
        direction_B = 1
    else:
        direction_B = -1
    pulse_count_B += direction_B

# 前進
def forward(rate_A, rate_B):
    rate_A = max(0, min(rate_A, 100))
    rate_B = max(0, min(rate_B, 100))
    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(int(65535 * rate_A / 100))
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(int(65535 * rate_B / 100))

# 停止
def stop():
    AIN1.off()
    AIN2.off()
    BIN1.off()
    BIN2.off()

# RPM計算
def compute_rpm(pulse_count, interval):
    if interval <= 0:
        return 0
    rpm = abs((pulse_count * 60) / (PPR * GEAR_RATIO * interval))
    return rpm

# P制御
def p_control(kp, target, actual):
    return kp * (target - actual)

if __name__ == '__main__':
    try:
        OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_A)
        OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_B)
        rate_A = rate_B = 10
        forward(rate_A, rate_B)
        start_time = time.ticks_ms()
        while True:
            time.sleep(0.1)
            now = time.ticks_ms()
            interval = (now - start_time) / 1000
            rpm_A = compute_rpm(pulse_count_A, interval)
            rpm_B = compute_rpm(pulse_count_B, interval)     
            rate_A += p_control(Kp, TARGET_RPM, rpm_A)
            rate_B += p_control(Kp, TARGET_RPM, rpm_B)
            forward(rate_A, rate_B)
            pulse_count_A = 0
            pulse_count_B = 0
            start_time = now
        
    except KeyboardInterrupt:
        stop()
        print("Stopped!")
