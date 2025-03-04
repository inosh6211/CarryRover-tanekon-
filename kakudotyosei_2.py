from machine import UART, Pin, PWM, Timer
import time
import math
import utime
from bno055 import *
from ble_simple_peripheral import BLESimplePeripheral
import bluetooth
import time
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))

ble = bluetooth.BLE()
p = BLESimplePeripheral(ble, name="RPpicoW")
time.sleep(1)

bno = BNO055(i2c)#bno,eulerの準備

UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))


PPR         = 3         # PPR = CPR / 4
GEAR_RATIO  = 297.92    # ギア比
FREQ        = 20        # タイマー割り込みの周波数 [Hz]
KP_RPM      = 0.5       # P制御の比例ゲイン
KP_YAW=0.3#ここで比例定数を決める

# モーターの状態
STOP       = 0
FORWARD    = 1
TURN_R     = 2
TURN_L     = 3
BACKWARD   = 4

# 画像誘導
RED    = 0
BLUE   = 1
PURPLE = 2

KP_CAMERA = 0.05

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
        self.start_time = time.ticks_ms()

    def disable_irq(self):
        self.timer.deinit()
        self.OUTA_A.irq(handler=None)
        self.OUTA_B.irq(handler=None)

UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

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
                print("Unitv connected")
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
            if len(data) > 1:
                if (len(data) - 1) % 6 == 0:
                    for i in range((len(data) - 1) / 6):
                        tag_id = int(data[i * 6 + 1])
                        self.tag_detected[tag_id] = 1
                        self.tag_cx[tag_id] = int(data[i * 6 + 2])
                        self.tag_cy[tag_id] = int(data[i * 6 + 3])
                        self.tag_distance[tag_id] = float(data[i * 6 + 4])
                        self.tag_roll[tag_id] = float(data[i * 6 + 5])
                        self.tag_pitch[tag_id] = float(data[i * 6 + 6])




def april_tag_alignment():
    motor=Motor()
    motor.enable_irq()
    motor.update_rpm(30, 30)
    
    while True:
        # カメラからタグ情報を取得（read_tags(0)でtag_distance, tag_pitch等が更新される）
        camera.read_tags(0)
        # 距離補正式
        corrected_distance = 848.23 + (2.5032 * camera.tag_distance[2] - 1.1803)
        # タグのピッチ角（°）を取得
        ka = camera.tag_pitch[2]
        sinx = math.sin(math.radians(ka))
        print("Corrected distance:", corrected_distance, "Pitch:", ka, "sin:", sinx)
        
        if 10 <= ka <= 180:
            # ●【ブランチ1】：kaが10～180°の場合（例：右側に傾いている場合）
            # ①後退：2秒バック走行してロボットの位置を調整
            motor.update_rpm(30, 30)
            motor.run(BACKWARD)
            time.sleep(2)
            motor.stop()
            
            # ②右回転：BNO055からyawを取得し、初期角度から「(90 - ka)」分（概ね）右に旋回
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            while True:
                motor.update_rpm(10, 10)
                motor.run(TURN_R)
                bno.compute_euler()
                current_yaw = (-bno.yaw + 360) % 360
                # 差分（正負±180°の範囲に正規化）
                diff = ((current_yaw - init_yaw + 180) % 360) - 180
                if diff >= (90 - ka):
                    motor.stop()
                    break
                time.sleep(0.01)
            
            # ③前進：前進すべき距離は　go = corrected_distance * |sin(ka)|　とし、
            #     移動時間は　t = go / 424.115 　（※係数は現地調整の換算値）
            go = corrected_distance * abs(sinx)
            t_time = go / 424.115  # 単位：秒
            start_time = time.ticks_ms()
            motor.update_rpm(30, 30)
            while time.ticks_diff(time.ticks_ms(), start_time) < t_time * 1000:
                motor.run(FORWARD)
                time.sleep(0.01)
            motor.stop()
            
            # ④最終旋回：正対状態にするため左回り90°旋回
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            while True:
                motor.update_rpm(10, 10)
                motor.run(TURN_L)
                bno.compute_euler()
                current_yaw = (-bno.yaw + 360) % 360
                diff = ((current_yaw - init_yaw + 180) % 360) - 180
                if diff >= 90:
                    motor.stop()
                    break
                time.sleep(0.01)
        
        elif 180 <= ka <= 350:
            # ●【ブランチ2】：kaが180～350°の場合（例：左側に傾いている場合）
            # ①後退：2秒バック走行してロボットの位置を調整
            motor.update_rpm(30, 30)
            motor.run(BACKWARD)
            time.sleep(2)
            motor.stop()
            
            # ②左回転：yawの差分が「(450 - ka)」に達するまで左回転
            #     （※実質、kaを360未満に直した場合と対称的な操作）
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            while True:
                motor.update_rpm(10, 10)
                motor.run(TURN_L)
                bno.compute_euler()
                current_yaw = (-bno.yaw + 360) % 360
                diff = ((current_yaw - init_yaw + 180) % 360) - 180
                if diff >= (450 - ka):
                    motor.stop()
                    break
                time.sleep(0.01)
            
            # ③前進：前進すべき距離は同様に　go = corrected_distance * |sin(ka)|、
            #     移動時間 t = go / 424.115
            go = corrected_distance * abs(sinx)
            t_time = go / 424.115
            start_time = time.ticks_ms()
            motor.update_rpm(30, 30)
            while time.ticks_diff(time.ticks_ms(), start_time) < t_time * 1000:
                motor.run(FORWARD)
                time.sleep(0.01)
            motor.stop()
            
            # ④最終旋回：正対状態にするため右回り90°旋回
            bno.compute_euler()
            init_yaw = (-bno.yaw + 360) % 360
            while True:
                motor.update_rpm(10, 10)
                motor.run(TURN_R)
                bno.compute_euler()
                current_yaw = (-bno.yaw + 360) % 360
                diff = ((current_yaw - init_yaw + 180) % 360) - 180
                if diff >= 90:
                    motor.stop()
                    break
                time.sleep(0.01)
        
        # ループ内で待機
        time.sleep(0.01)
