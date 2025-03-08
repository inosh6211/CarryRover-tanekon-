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



# モーターピン設定
AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
BIN1 = Pin(19, Pin.OUT)
BIN2 = Pin(22, Pin.OUT)
PWMB = PWM(Pin(28))
PWMB.freq(1000)
STBY = Pin(9, Pin.OUT, value = 1)

KP_YAW = 0.1

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
    rpm = 70 #現地で調査
    #t = distance / (135 * math.pi * (rpm /60))
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
            
            
            
    
if __name__ == '__main__':
    bno = BNO055Handler(I2C0)
    try:
        while True:
            forward(70, 70)
            time.sleep(2)
            
            backward(70, 70)
            time.sleep(2)
            
            turn_right(70)
            time.sleep(2)
            
            turn_left(70)
            time.sleep(2)
            
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
            time.sleep(2)
            
            soutai_turn(60, init_yaw)
            
            
            straight_ward("f", 5)
            
            
            straight_ward("b", 5)
            

    except KeyboardInterrupt:
        stop()
