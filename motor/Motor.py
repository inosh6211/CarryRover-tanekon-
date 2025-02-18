from machine import Pin, PWM, Timer
import time
import math

STATE_STOP       = 0
STATE_FORWARD    = 1
STATE_TURN_RIGHT = 2
STATE_TURN_LEFT  = 3
STATE_BACKWARD   = 4

PPR         = 3            # PPR = CPR / 4
GEAR_RATIO  = 297.92       # ギア比
INTERVAL = 0.05            # タイマー割り込み間隔[s]
KP_RPM      = 0.5          # P制御の比例ゲイン

class Motor:
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
        self.state = STATE_STOP

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
        if self.state != STATE_FORWARD:
            self.stop()
        
        self.AIN1.off()
        self.AIN2.on()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.on()
        self.BIN2.off()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = STATE_FORWARD

    def turn_right(self):
        if self.state != STATE_TURN_RIGHT:
            self.stop()
            
        self.AIN1.on()
        self.AIN2.off()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.on()
        self.BIN2.off()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = STATE_TURN_RIGHT

    def turn_left(self):
        if self.state != STATE_TURN_LEFT:
            self.stop()
            
        self.AIN1.off()
        self.AIN2.on()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.off()
        self.BIN2.on()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = STATE_TURN_LEFT

    def backward(self):
        if self.state != STATE_BACKWARD:
            self.stop()

        self.AIN1.on()
        self.AIN2.off()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.off()
        self.BIN2.on()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        self.state = STATE_BACKWARD

    def stop(self):
        self.AIN1.off()
        self.AIN2.off()
        self.BIN1.off()
        self.BIN2.off()
        self.rate_a = 10
        self.rate_b = 10
        self.state = STATE_STOP

    def compute_rpm(self, pulse_count):
        if INTERVAL <= 0:
            return 0
        rpm = abs((pulse_count * 60) / (PPR * GEAR_RATIO * INTERVAL))
        return rpm

    def update_speed(self, timer):
        rpm_a = self.compute_rpm(self.pulse_count_a)
        rpm_b = self.compute_rpm(self.pulse_count_b)
        self.rate_a += KP_RPM * (self.target_rpm_a - rpm_a)
        self.rate_b += KP_RPM * (self.target_rpm_b - rpm_b)
        self.rate_a = min(max(self.rate_a, 0), 100)
        self.rate_b = min(max(self.rate_b, 0), 100)
        self.pulse_count_a = 0
        self.pulse_count_b = 0
    
    def update_rpm(self, target_rpm_a, target_rpm_b):
        self.target_rpm_a = target_rpm_a
        self.target_rpm_b = target_rpm_b

    # 割り込み有効化
    def enable_irq(self):
        self.OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)
        self.timer.init(mode=Timer.PERIODIC, freq=1/INTERVAL, callback=self.update_speed)

    # 割り込み無効化
    def disable_irq(self):
        self.timer.deinit()
        self.OUTA_1.irq(handler=None)
        self.OUTA_2.irq(handler=None)

if __name__ == '__main__':
    try:
        motor = Motor(
            ain1=18, ain2=17, pwma=16,
            bin1=19, bin2=22, pwmb=28,
            stby=9,
            outa1=3, outb1=2,
            outa2=7, outb2=6
        )

        motor.enable_irq()
        motor.update_rpm(30, 30)
        
        start_time = time.time()
        while time.time() - start_time < 3:
            motor.forward()
            time.sleep(0.01)

        start_time = time.time()
        while time.time() - start_time < 3:
            motor.turn_right()
            time.sleep(0.01)
        
        start_time = time.time()
        while time.time() - start_time < 3:
            motor.turn_left()
            time.sleep(0.01)
        
        start_time = time.time()
        while time.time() - start_time < 3:
            motor.backward()
            time.sleep(0.01)
        
        motor.stop()
        motor.disable_irq()
        print("stopped!")
        
    except KeyboardInterrupt:
        motor.stop()
        motor.disable_irq()
        print("stopped!")
