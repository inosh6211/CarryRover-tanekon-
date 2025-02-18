from machine import Pin, PWM, Timer
import time
import math

PPR         = 3            # PPR = CPR / 4
GEAR_RATIO  = 297.92       # ギア比
FREQ        = 20           # タイマー割り込みの周波数[hz]
KP_RPM      = 0.1          # P制御の比例ゲイン

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
        
        while True:
            time.sleep(3)
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
        
    except KeyboardInterrupt:
        motor.stop()
        motor.disable_irq()
        print("stopped!")
