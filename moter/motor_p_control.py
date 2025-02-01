from machine import Pin, PWM
import time

# モーターピン設定
AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
OUTA_1 = Pin(3, Pin.IN)
OUTB_1 = Pin(2, Pin.IN)
BIN1 = Pin(19, Pin.OUT)
BIN2 = Pin(22, Pin.OUT)
PWMB = PWM(Pin(28))
PWMB.freq(1000)
OUTA_2 = Pin(7, Pin.IN)
OUTB_2 = Pin(6, Pin.IN)
STBY = Pin(9, Pin.OUT)

# エンコーダーの設定
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
def pulse_counter_A(pin):
    global pulse_count_A, direction_A
    if OUTB_1.value() == 1:
        direction_A = 1
    else:
        direction_A = -1
    pulse_count_A += direction_A

# モーターBのエンコーダ処理
def pulse_counter_B(pin):
    global pulse_count_B, direction_B
    if OUTB_2.value() == 1:
        direction_B = 1
    else:
        direction_B = -1
    pulse_count_B += direction_B

# 前進
def forward(rate_A, rate_B):
    OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_A)
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA.duty_u16(2**16 // 100 * rate_A)
    
    OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_B)
    STBY.on()
    BIN1.off()
    BIN2.on()
    PWMB.duty_u16(2**16 // 100 * rate_B)

# 停止
def stop():
    OUTA_1.irq(handler=None)
    AIN1.off()
    AIN2.off()
    
    OUTA_2.irq(handler=None)
    BIN1.off()
    BIN2.off()

# RPM計算
def calculateRPM(pulse_count, interval):
    rpm = (pulse_count * 60) / (PPR * GEAR_RATIO * interval)
    return rpm

# 制御ループ
def moter_p_contral():
    global pulse_count_A, pulse_count_B, target_rpm_A, target_rpm_B
    rate_A = 0
    rate_B = 0
    while True:
        forward(int(rate_A), int(rate_B))
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

if __name__ == '__main__':
    try:
        moter_p_contral()
    except KeyboardInterrupt:
        stop()
        print("Stop!!")
