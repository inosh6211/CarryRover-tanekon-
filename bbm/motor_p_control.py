from machine import Pin, PWM
import time

# モーターA設定
AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
OUTA_1 = Pin(2, Pin.IN)
OUTB_1 = Pin(3, Pin.IN)

# モーターB設定
BIN1 = Pin(19, Pin.OUT)
BIN2 = Pin(22, Pin.OUT)
PWMB = PWM(Pin(28))
PWMB.freq(1000)
OUTA_2 = Pin(6, Pin.IN)
OUTB_2 = Pin(7, Pin.IN)

# その他設定
STBY = Pin(9, Pin.OUT)

PPR = 3  # PPR = CPR / 4
GEAR_RATIO = 297.92
INTERVAL = 0.1

pulse_count_A = 0
pulse_count_B = 0
direction_A = 0
direction_B = 0

Kp = 0.2
target_rpm_A = 60
target_rpm_B = 60

# モーターAのエンコーダ処理
def pulseCounterA(pin):
    global pulse_count_A, direction_A
    if OUTB_1.value() == 1:
        direction_A = 1
    else:
        direction_A = -1
    pulse_count_A += direction_A

# モーターBのエンコーダ処理
def pulseCounterB(pin):
    global pulse_count_B, direction_B
    if OUTB_2.value() == 1:
        direction_B = 1
    else:
        direction_B = -1
    pulse_count_B += direction_B

# モーターAの前進
def forwardA(rate):
    OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=pulseCounterA)
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA.duty_u16(2**16 // 100 * rate)

# モーターBの前進
def forwardB(rate):
    OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=pulseCounterB)
    STBY.on()
    BIN1.off()
    BIN2.on()
    PWMB.duty_u16(2**16 // 100 * rate)

# 停止
def stop():
    OUTA_1.irq(handler=None)
    OUTA_2.irq(handler=None)
    AIN1.off()
    AIN2.off()
    BIN1.off()
    BIN2.off()

# RPM計算
def calculateRPM(pulse_count, interval):
    rpm = (pulse_count * 60) / (PPR * GEAR_RATIO * interval)
    return rpm

# 制御ループ
def moterPcontral():
    global pulse_count_A, pulse_count_B, target_rpm_A, target_rpm_B
    rate_A = 0
    rate_B = 0
    while True:
        forwardA(int(rate_A))
        forwardB(int(rate_B))
        time.sleep(0.1)
        
        current_rpm_A = abs(calculateRPM(pulse_count_A, 0.1))
        current_rpm_B = abs(calculateRPM(pulse_count_B, 0.1))
        
        pulse_count_A = 0
        pulse_count_B = 0
        
        diff_rpm_A = target_rpm_A - current_rpm_A
        diff_rpm_B = target_rpm_B - current_rpm_B
        
        rate_A += Kp * diff_rpm_A
        rate_B += Kp * diff_rpm_B
        
        print(f"Motor A RPM: {current_rpm_A}, Motor B RPM: {current_rpm_B}")

# 実行
try:
    moterPcontral()
except KeyboardInterrupt:
    stop()
    print("Stop!!")


