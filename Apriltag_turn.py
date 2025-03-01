from machine import Pin, PWM, Timer
import time
import math
import utime
from bno055 import *
from ble_simple_peripheral import BLESimplePeripheral
#import bluetooth
import time
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))
"""ble = bluetooth.BLE()
p = BLESimplePeripheral(ble, name="RPpicoW")"""
time.sleep(1)

KP_YAW=0.3#ここで比例定数を決める


bno = BNO055(i2c)#bno,eulerの準備


PPR         = 3         # PPR = CPR / 4
GEAR_RATIO  = 297.92    # ギア比
FREQ        = 20        # タイマー割り込みの周波数 [Hz]
KP_RPM      = 0.1       # P制御の比例ゲイン

# モーターの状態
STOP       = 0
FORWARD    = 1
TURN_R = 2
TURN_L  = 3
BACKWARD   = 4

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


#UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

#キャリブレーション結果
TAG_SIZE = 30
FOCAL_LENGTH_X = 210.70  
FOCAL_LENGTH_Y = 210.70

CENTER_X = 100 # ROI_W / 2
CENTER_Y = 100 # ROI_H / 2
DIST_COEFFS = [-0.34155103, 0.25499062, -0.00517389, -0.00731524, -0.17990041]

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

        # エンコーダの割り込み設定（OUTA_A と OUTA_B に設定）
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
            interval = (now - self.start_time) / 1000
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

    def enable_irq(self):
        self.OUTA_A.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_B.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)
        self.timer.init(mode=Timer.PERIODIC, freq=FREQ, callback=self.update_speed)
        self.start_time = time.ticks_ms()

    def disable_irq(self):
        self.timer.deinit()
        self.OUTA_A.irq(handler=None)
        self.OUTA_B.irq(handler=None)



def euler():
    bno.iget(QUAT_DATA)
   

    w = (bno.w / 16384)
    x = (bno.x / 16384)
    y = (bno.y / 16384)
    z = (bno.z / 16384)

    ysqr = y * y

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.atan2(t0, t1);

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    if t2 > 1.0:
        t2 = 1.0
   
    elif t2 < -1.0:
        t2 = -1.0
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = math.atan2(t3, t4)

    roll *= 180/math.pi
    pitch *= 180/math.pi
    yaw *= 180/math.pi
    return roll, pitch, yaw

def soutai_turn_right_90():
        while True:
            motor.update_rpm(10,10)
            motor.run(TURN_R)
            roll, pitch, yaw = euler()
            current_yaw = yaw
            diff=-(current_yaw-init_yaw)
            if diff < 0:
                diff = diff + 360
            print(diff)
            if diff >= 89 or diff <= 91:#358to2とかで359とかでとまったときにdiff >= 90認定されないように
                motor.stop()
                print("stop")
                break
            time.sleep(0.01)

def soutai_turn_right_from358to2():
        while True:
            motor.update_rpm(10,10)
            motor.run(TURN_R)
            roll, pitch, yaw = euler()
            current_yaw = yaw
            diff=-(current_yaw-init_yaw)
            if diff < 0:
                diff = diff + 360
            print(diff)
            if 359 <= diff or diff <= 1:#もし358 <= diff or diffにおさまらなかったときようのを書いておくべき？
                motor.stop()
                print("stop")
                break
            time.sleep(0.01)


def soutai_turn_left_90():
        while True:
            motor.update_rpm(10,10)
            motor.run(TURN_L)
            roll, pitch, yaw = euler()
            current_yaw = yaw
            diff=(current_yaw-init_yaw)
            if diff < 0:
                diff = diff + 360
            print(diff)
            if diff >= 89 or diff <= 91:#358to2とかで359とかでとまったときにdiff >= 90認定されないように#
                motor.stop()
                print("stop")
                break
            time.sleep(0.01)
            
def soutai_turn_left_from358to2():
        while True:
            motor.update_rpm(10,10)
            motor.run(TURN_L)
            roll, pitch, yaw = euler()
            current_yaw = yaw
            diff=(current_yaw-init_yaw)
            if diff < 0:
                diff = diff + 360
            print(diff)
            if 359 <= diff or diff <= 1:
                motor.stop()
                print("stop")
                break
            time.sleep(0.01)
            
def soutai_turn_left_180():
        while True:
            motor.update_rpm(10,10)
            motor.run(TURN_L)
            roll, pitch, yaw = euler()
            current_yaw = yaw
            diff=(current_yaw-init_yaw)
            if diff < 0:
                diff = diff + 360
            print(diff)
            if diff >= 179 or diff <= 181:#358to2とかで359とかでとまったときにdiff >= 180認定されないように
                motor.stop()
                print("stop")
                break
            time.sleep(0.01)
            
            
           

# UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
uart = machine.UART(1, baudrate=115200, tx=4, rx=5)

# カメラキャリブレーション結果
FOCAL_LENGTH_X = 250.8939245
FOCAL_LENGTH_Y = 250.6935671
CENTER_X = 177.63441089
CENTER_Y = 94.85498261
DIST_COEFFS = [-0.34382066, -0.34788885, 0.01396397, -0.01068236, 0.71124219]



def straight_forward_t(t):
    start=time.time()
    roll, pitch, yaw = euler()
    init_yaw_s = yaw#init_yawは最初にとったののみにしたいので、変える
    while True:
        print(f"RESET")
        motor.update_rpm(30,30)
        motor.run(FORWARD)
        roll, pitch, yaw = euler()
        current_yaw = yaw
        diff=(current_yaw-init_yaw_s)
        print(diff)
        
        if abs(diff) >= 5:#それている#左が正
            print(f"soreteru")
            while True:
                roll, pitch, yaw = euler()
                current_yaw = yaw
                diff=(current_yaw-init_yaw_s)
                rate_a=-KP_YAW*diff+30
                rate_b=KP_YAW*diff+30
                motor.update_rpm(rate_a, rate_b)
                motor.run(FORWARD)
                print(f"L{diff}")
                time.sleep(0.01)
                if abs(diff) <= 5:
                    break
        
        elif time.time() - start >= t:
            break
                
        time.sleep(0.01)

#歪み補正
def undistort_point(x, y):
    norm_x = (x - CENTER_X) / FOCAL_LENGTH_X
    norm_y = (y - CENTER_Y) / FOCAL_LENGTH_Y
    r2 = norm_x**2 + norm_y**2
    radial_distortion = 1 + DIST_COEFFS[0] * r2 + DIST_COEFFS[1] * (r2**2) + DIST_COEFFS[4] * (r2**3)
    
    corrected_x = norm_x * radial_distortion * FOCAL_LENGTH_X + CENTER_X
    corrected_y = norm_y * radial_distortion * FOCAL_LENGTH_Y + CENTER_Y

    
    return corrected_x, corrected_y

def forward_to_Apriltag_mm(m):
    
    while True:
        if uart.any():  # 受信バッファにデータがあるか確認#tagを見つけられなければここでループ
            try:
                line = uart.readline()
                if line:
                    try:
                        line = line.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        print("UART decoding error")
                        #continue
                #else:
                    #continue
                data = line.split(',')

                # AprilTag
                if len(data) == 4 and data[0] != "NoTag":
                    tag_id = int(data[0])
                    cx, cy = float(data[1]), float(data[2])
                    distance = float(data[3])
                    
                    corrected_cx, corrected_cy = undistort_point(cx, cy)
                    corrected_distance = (FOCAL_LENGTH_X * TAG_SIZE) / corrected_cx
                    
                    pitch_rad = math.atan2(corrected_cy, corrected_distance)
                    pitch = math.degrees(pitch_rad)
                    yaw_rad = math.atan2(corrected_cx, corrected_distance)
                    yaw = math.degrees(yaw_rad)
                
                    print(f"AprilTag ID: {tag_id}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}, Distance: {corrected_distance:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

                # 色認識データの処理
                elif len(data) == 4 and data[0] == "Color" and data[1] != "None":
                    color_name = data[1]
                    cx, cy = float(data[2]), float(data[3])
                    corrected_cx, corrected_cy = undistort_point(cx, cy)
                    
                    print(f"Detected Color: {color_name}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}")
                    motor.run(FORWARD)
                    if corrected_distance <= m:
                        print(f"april")
                        motor.stop()
                        break
            except Exception as e:
                print(f"Error: {e}")
                


        utime.sleep(0.1)  # 100ms待機
   
motor = Motor()
motor.enable_irq()
print("e")
m=160#ここでapriltagの何ｍｍ前に来るかを決める

#if __name__ == '__main__':
try:
    tag_id = 0    # ID
    cx = 0      # X座標
    cy = 0      # Y座標
    distance = 0  # 距離
    pitch = 0     # ピッチ角
    yaw = 0       # ヨー角

       
    motor.enable_irq()
    motor.update_rpm(30, 30)
    print("e")
    while True:
        print("e")
        if uart.any():  # 受信バッファにデータがあるか確認
            break
        utime.sleep(0.1)  # 100ms待機
    try:
        print("go")
        # 1行のデータを受信
        line = uart.readline()
        if line:
            try:
                line = line.decode('utf-8').strip()
            except UnicodeDecodeError:
                print("UART decoding error")
                #continue
        #else:
            #continue
        data = line.split(',')

        #AprilTag認識
        if len(data) == 4 and data[0] != "Color":
            tag_id = int(data[0])
            cx, cy = float(data[1]), float(data[2])
            distance = float(data[3]) 
            
            corrected_cx, corrected_cy = undistort_point(cx, cy)
            corrected_distance = distance
            
            pitch_rad = math.atan2((corrected_cy - CENTER_Y) / FOCAL_LENGTH_Y, 1)
            pitch = math.degrees(pitch_rad)
            yaw_rad = math.atan2((corrected_cx - CENTER_X) / FOCAL_LENGTH_X, 1)
            yaw = math.degrees(yaw_rad)
        
            print(f"AprilTag ID: {tag_id}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}, Distance: {corrected_distance:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        #色認識
        elif len(data) == 4 and data[0] == "Color":
            color_name = data[1]
            cx, cy = float(data[2]), float(data[3])
            corrected_cx, corrected_cy = undistort_point(cx, cy)
            
            print(f"Detected Color: {color_name}, Corrected X: {corrected_cx:.2f}, Corrected Y: {corrected_cy:.2f}")

    except Exception as e:
        print(f"Error: {e}")


    if tag_id == 0:
        print(0)
        roll, pitch, yaw = euler()
        init_yaw = yaw
        forward_to_Apriltag_mm(m)
        print(f"complete")
       
    elif tag_id == 1:
        print(f"1")
        roll, pitch, yaw = euler()
        init_yaw = yaw
        soutai_turn_right_90()
        straight_forward_t(2)
        soutai_turn_left_from358to2()
        straight_forward_t(2)
        soutai_turn_left_90()
        print("ji")
        print(f"complete")
        
    elif tag_id == 2:#ここで特異点を超えてしまうやつが発生する危険性あり
        print(f"2")
        roll, pitch, yaw = euler()
        init_yaw = yaw
        soutai_turn_right_90()
        straight_forward_t(2)
        soutai_turn_left_from358to2()
        straight_forward_t(4)
        soutai_turn_left_90()
        straight_forward_t(2)
        soutai_turn_left_180()
        print(f"complete")
        
    elif tag_id == 3:
        print(f"3")
        roll, pitch, yaw = euler()
        init_yaw = yaw;
        soutai_turn_left_90()
        straight_forward_t(2)
        soutai_turn_right_from358to2()
        straight_forward_t(2)
        soutai_turn_right_90()
        print(f"complete")
       
except KeyboardInterrupt:#stopを押したときにモーターを止めるための
                motor.stop()
                motor.disable_irq()
                print("stopped!")
