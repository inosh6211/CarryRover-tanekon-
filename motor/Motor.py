from machine import Pin, PWM
import time

class Motor:
    PPR = 3  # PPR = CPR / 4
    GEAR_RATIO = 297.92  # ギア比
    KP_RPM = 0.5  # P制御の比例ゲイン

    def __init__(self, ain1, ain2, pwma, bin1, bin2, pwmb, stby, outa1, outb1, outa2, outb2):
        # モーター用ピンの設定(Aが右, Bが左)
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
        self.rate_a = 10
        self.rate_b = 10
        self.pulse_count_a = 0
        self.pulse_count_b = 0
        self.direction_a = 0
        self.direction_b = 0
        self.start_time = time.ticks_ms()

        # エンコーダの割り込み設定
        self.OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)

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
        self.AIN1.off()
        self.AIN2.on()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.on()
        self.BIN2.off()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        
    def turn_right(self):
        self.AIN1.on()
        self.AIN2.off()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.on()
        self.BIN2.off()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
        
    def turn_left(self):
        self.AIN1.off()
        self.AIN2.on()
        self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
        self.BIN1.off()
        self.BIN2.on()
        self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
    
    def backward(self):
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
        self.rate_a = 10
        self.rate_b = 10

    def compute_rpm(self, pulse_count, interval):
        if interval <= 0:
            return 0
        rpm = abs((pulse_count * 60) / (self.PPR * self.GEAR_RATIO * interval))
        return rpm

    def update_motor_speed(self, target_rpm_a, target_rpm_b):
        now = time.ticks_ms()
        interval = (now - self.start_time) / 1000
        rpm_a = self.compute_rpm(self.pulse_count_a, interval)
        rpm_b = self.compute_rpm(self.pulse_count_b, interval)
        self.rate_a += self.KP_RPM * (target_rpm_a - rpm_a)
        self.rate_b += self.KP_RPM * (target_rpm_b - rpm_b)
        self.start_time = now
        self.pulse_count_a = 0
        self.pulse_count_b = 0

if __name__ == '__main__':
    try:
        motor = Motor(
            ain1=18, ain2=17, pwma=16,
            bin1=19, bin2=22, pwmb=28,
            stby=9,
            outa1=3, outb1=2,
            outa2=7, outb2=6
        )

        TARGET_RPM = 30
        while True:
            motor.update_motor_speed(TARGET_RPM, TARGET_RPM)
            motor.forward()
            time.sleep(0.05)
    except KeyboardInterrupt:
        motor.stop()
        print("stopped!")
