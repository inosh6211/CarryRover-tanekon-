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
INTERVAL = 0.1

target_rpm_A = 50  # 目標回転数
target_rpm_B = 50

pulse_count_A = 0
direction_A = 0
pulse_count_B = 0
direction_B = 0

# PIDパラメータ
kp = 1.0
ki = 0.1
kd = 0.05
integral_A = 0
prev_error_A = 0
integral_B = 0
prev_error_B = 0

def pid_compute(target, actual, integral, prev_error, dt):
    error = target - actual
    integral += error * dt
    derivative = (error - prev_error) / dt
    output = (kp * error) + (ki * integral) + (kd * derivative)
    prev_error = error
    return max(0, min(output, 100)), integral, prev_error

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

# RPM計算
def calculate_rpm(pulse_count, interval):
    if interval > 0:
        return (pulse_count * 60) / (PPR * GEAR_RATIO * interval)
    else:
        return 0

# PWM更新
def update_motor_speed():
    global pulse_count_A, pulse_count_B, integral_A, prev_error_A, integral_B, prev_error_B
    rpm_A = calculate_rpm(pulse_count_A, INTERVAL)
    rpm_B = calculate_rpm(pulse_count_B, INTERVAL)
    
    duty_A, integral_A, prev_error_A = pid_compute(target_rpm_A, rpm_A, integral_A, prev_error_A, INTERVAL)
    duty_B, integral_B, prev_error_B = pid_compute(target_rpm_B, rpm_B, integral_B, prev_error_B, INTERVAL)
    
    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(int(65535 * duty_A / 100))
    
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(int(65535 * duty_B / 100))
    
    pulse_count_A = 0
    pulse_count_B = 0

if __name__ == "__main__":
    try:
        OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_A)
        OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_B)
        start_time = time.ticks_ms()

        while True:
            time.sleep(INTERVAL)
            update_motor_speed()
            elapsed_time = (time.ticks_ms() - start_time) / 1000
            print(f"Elapsed Time: {elapsed_time:.2f}s, RPM A: {calculate_rpm(pulse_count_A, INTERVAL):.2f}, RPM B: {calculate_rpm(pulse_count_B, INTERVAL):.2f}")
    except KeyboardInterrupt:
        AIN1.off()
        AIN2.off()
        BIN1.off()
        BIN2.off()
        print("Stop!!")
