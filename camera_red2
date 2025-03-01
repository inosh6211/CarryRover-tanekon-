from machine import UART, Pin, PWM, Timer
import time
import math

PPR         = 3         # PPR = CPR / 4
GEAR_RATIO  = 297.92    # ギア比
FREQ        = 20        # タイマー割り込みの周波数 [Hz]
KP_RPM      = 0.5       # P制御の比例ゲイン

# モーターの状態
STOP       = 0
FORWARD    = 1
TURN_R     = 2
TURN_L     = 3
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

UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart

    def read_camera(self):
        message = ""
        while not self.uart.any():
            time.sleep(0.01)
                
        while self.uart.any() > 0: 
            message += self.uart.read().decode('utf-8').strip()
            
        data = message.split(',')
        
        return data

    def read_color(self):
        self.uart.write("c\n")
        data = self.read_camera()
        
        if data[0] == "c":
            self.color_pixels = []
            self.color_cx = []
            self.color_cy = []
            
            for color in range(3):
                self.color_pixels.append(float(data[color*3 + 1]))
                self.color_cx.append(float(data[color*3 + 2]))
                self.color_cy.append(float(data[color*3 + 3]))

    def read_tag(self):
        self.uart.write("t\n")    
        data = self.read_camera()
        
        if data[0] == "t":
            self.tag_detected = []
            self.tag_cx = []
            self.tag_cy = []
            self.tag_distance = []
            self.tag_pitch = []
            
            for tag_id in range(10):
                self.tag_detected.append(int(data[tag_id*5 + 1]))
                self.tag_cx.append(float(data[tag_id*5 + 2]))
                self.tag_cy.append(float(data[tag_id*5 + 3]))
                self.tag_distance.append(float(data[tag_id*5 + 4]))
                self.tag_pitch.append(float(data[tag_id*5 + 5]))


if __name__ == "__main__":
    camera = CameraReceiver(UART1)
    time.sleep(2)

def constrain(value, min_val, max_val):
        return max(min_val, min(value, max_val))
    
try:
    motor = Motor()
    motor.enable_irq()    
    while True:
        camera.read_color()
        for i in range(3):
            print(i, camera.color_pixels[i], camera.color_cx[i], camera.color_cy[i])
            
        camera.read_tag()    
        for j in range(10):
            print(j, camera.tag_cx[j], camera.tag_cy[j], camera.tag_distance[j], camera.tag_pitch[j])

        time.sleep(0.1)
        
        Kp_camera = 0.5
        pix = 120
        
        if camera.color_pixels[0] == 0:
            motor.update_rpm(30,30)
            motor.run(TURN_R)#旋回
            
        else:
            cx = camera.color_cx[0]
            p_pwma = constrain(-Kp_camera * (cx - pix) + 30 , 0, 30)
            p_pwmb = constrain(Kp_camera * (cx - pix) + 30 , 0, 30)
            print(f"p_pwma: {p_pwma}, p_pwmb: {p_pwmb}, cx: {cx}, pix: {pix}")
            motor.update_rpm(p_pwma, p_pwmb)
            motor.run(FORWARD)
                       
        if camera.color_pixels[0] > 4000:
            motor.update_rpm(30,30)
            motor.run(FORWARD)
            time.sleep(0.5)
            motor.stop()
            print("Goal!")
            break
finally:
        motor.stop()
        motor.disable_irq()
        print("stopped!")
