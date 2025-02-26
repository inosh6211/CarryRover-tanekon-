from machine import UART,Pin, PWM, Timer
import time
import math

PPR         = 3         # PPR = CPR / 4
GEAR_RATIO  = 297.92    # ギア比
FREQ        = 20        # タイマー割り込みの周波数 [Hz]
KP_RPM      = 0.5       # P制御の比例ゲイン

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
STOP       = 0
FORWARD    = 1
TURN_RIGHT = 2
TURN_LEFT  = 3
BACKWARD   = 4

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
            
        elif self.state == TURN_RIGHT:
            self.AIN1.on()
            self.AIN2.off()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.on()
            self.BIN2.off()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
            
        elif self.state == TURN_LEFT:
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

uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
        
def read_camera():
    uart1.write("1\n")

    while uart1.any() == 0:
        pass

    message = ""

    while uart1.any() > 0:
        message += uart1.read().decode('utf-8').strip()

    if message.startswith("$"):
        data = message[1:].split(",")
        
        if data[0] == "0":
            return None

        objects_count = int(data[0])
        data = list(map(int, data[1:]))

        max_pixels = 0
        target_cx = 0
        target_cy = 0

        for i in range(objects_count):
            pixels = data[i * 3]
            cx = data[i * 3 + 1]
            cy = data[i * 3 + 2]

            if pixels > max_pixels:
                max_pixels = pixels
                target_cx = cx
                target_cy = cy

        return max_pixels, target_cx, target_cy
    
try:
    motor = Motor()
    motor.enable_irq()
    
    while True:
        camera_data = read_camera()
        if camera_data is None:
            pixels, cx, cy = 0, 0, 0  # デフォルト値
        else:
            pixels, cx, cy = camera_data

        print(f"ピクセル数: {pixels}, 中心座標: ({cx}, {cy})")
        time.sleep(0.1)

        if 0 < pixels <= 10000:
            if 0 <= cx <= 120:
                motor.update_rpm(30,30)
                motor.run(TURN_RIGHT)
            elif 120 < cx <= 240:
                motor.update_rpm(30,30)
                motor.run(TURN_LEFT)
            
        elif 10000 < pixels:#パラシュートが近いとき
            motor.stop()
            time.sleep(5)
            
        else: 
            motor.update_rpm(30,30)
            motor.run(FORWARD)
finally:
        motor.stop()
        motor.disable_irq()
        print("stopped!")
