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

pulse_count_A = 0
direction_A = 0
pulse_count_B = 0
direction_B = 0

# PID制御
Kp = 0.1
Ki = 0.05
Kd = 0.05
TARGET_RPM = 30

integral_A = 0
prev_error_A = 0
integral_B = 0
prev_error_B = 0

# エンコーダAのパルスカウント処理
def pulse_counter_A(pin):
    global pulse_count_A, direction_A
    if OUTA_1.value() == OUTB_1.value():
        direction_A = 1
    else:
        direction_A = -1
    pulse_count_A += direction_A

# エンコーダBのパルスカウント処理
def pulse_counter_B(pin):
    global pulse_count_B, direction_B
    if OUTA_2.value() == OUTB_2.value():
        direction_B = 1
    else:
        direction_B = -1
    pulse_count_B += direction_B

# PWM制御（前進）
def forward(rate_A, rate_B):
    rate_A = max(0, min(rate_A, 100))
    rate_B = max(0, min(rate_B, 100))

    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(int(65535 * rate_A / 100))
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(int(65535 * rate_B / 100))

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
    
if __name__ == '__main__':
    try:
        OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_A)
        OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_B)
        rate_A = rate_B = 10
        forward(rate_A, rate_B)
        start_time = time.ticks_ms()
        integral_A = 0
        integral_B = 0
        pre_error_A = 0
        pre_error_B = 0
        while True:
            time.sleep(0.1)
            now = time.ticks_ms()
            interval = (now - start_time) / 1000
            rpm_A = calculate_rpm(pulse_count_A, interval)
            rpm_B = calculate_rpm(pulse_count_B, interval)
            error_A = TARGET_RPM - rpm_A
            error_B = TARGET_RPM - rpm_B
            integral_A += error_A * interval 
            integral_B += error_B * interval
            if interval == 0:
                derivative_A = 0
                derivative_B = 0
            else:
                derivative_A = (error_A - pre_error_A) / interval
                derivative_B = (error_B - pre_error_B) / interval
                
            rate_A += Kp * error_A + Ki * integral_A + Kd * derivative_A
            rate_B += Kp * error_B + Ki * integral_B + Kd * derivative_B
            forward(rate_A, rate_B)
            pulse_count_A = 0
            pulse_count_B = 0
            start_time = now
        
    except KeyboardInterrupt:
        stop()
        print("Stopped!")
