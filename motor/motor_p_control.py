from machine import Pin, PWM
import time

# モーターピン設定
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
OUTA_1 = Pin(3, Pin.IN)
OUTB_1 = Pin(2, Pin.IN)
OUTA_2 = Pin(7, Pin.IN)
OUTB_2 = Pin(6, Pin.IN)

# エンコーダの設定
PPR = 3  # PPR = CPR / 4
GEAR_RATIO = 297.92

pulse_count_a = 0
direction_a = 0
pulse_count_b = 0
direction_b = 0

# P制御
KP_RPM = 1
TARGET_RPM = 30

# エンコーダAのパルスカウント処理
def pulse_counter_a(pin):
    global pulse_count_a, direction_a
    if OUTA_1.value() == OUTB_1.value():
        direction_a = 1
    else:
        direction_a = -1
    pulse_count_a += direction_a

# エンコーダBのパルスカウント処理
def pulse_counter_b(pin):
    global pulse_count_b, direction_b
    if OUTA_2.value() == OUTB_2.value():
        direction_b = 1
    else:
        direction_b = -1
    pulse_count_b += direction_b

# 前進
def forward(rate_a, rate_b):
    rate_a = max(0, min(rate_a, 100))
    rate_b = max(0, min(rate_b, 100))

    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(int(65535 * rate_a / 100))
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(int(65535 * rate_b / 100))

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

def p_control(kp, target, actual):
    return kp * (target - actual)
    
if __name__ == '__main__':
    try:
        OUTA_1.irq(trigger = Pin.IRQ_RISING, handler = pulse_counter_a)
        OUTA_2.irq(trigger = Pin.IRQ_RISING, handler = pulse_counter_b)
        rate_a = rate_b = 10
        forward(rate_a, rate_b)
        start_time = time.ticks_ms()
        while True:
            time.sleep(0.1)
            now = time.ticks_ms()
            interval = (now - start_time) / 1000
            rpm_a = compute_rpm(pulse_count_a, interval)
            rpm_b = compute_rpm(pulse_count_b, interval)     
            rate_a += p_control(KP_RPM, TARGET_RPM, rpm_a)
            rate_b += p_control(KP_RPM, TARGET_RPM, rpm_b)
            forward(rate_a, rate_b)
            pulse_count_a = 0
            pulse_count_b = 0
            start_time = now
        
    except KeyboardInterrupt:
        stop()
        print("Stopped!")
