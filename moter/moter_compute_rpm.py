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
STBY = Pin(9, Pin.OUT, value = 1)

# エンコーダピン設定
OUTA_1 = Pin(3, Pin.IN)
OUTB_1 = Pin(2, Pin.IN)
OUTA_2 = Pin(7, Pin.IN)
OUTB_2 = Pin(6, Pin.IN)

# エンコーダの設定
PPR = 3  # PPR = CPR / 4
GEAR_RATIO = 297.92
INTERVAL = 0.1

pulse_count_A = 0
direction_A = 0
pulse_count_B = 0
direction_B = 0

# エンコーダAのパルスカウント処理
def pulse_counter_A(pin):
    global pulse_count_A, direction_A

    if OUTA_1.value() == OUTB_1.value():
        direction_A = 1  # 正回転
    else:
        direction_A = -1  # 逆回転

    pulse_count_A += direction_A

# エンコーダBのパルスカウント処理
def pulse_counter_B(pin):
    global pulse_count_B, direction_B

    if OUTA_2.value() == OUTB_2.value():
        direction_B = 1  # 正回転
    else:
        direction_B = -1  # 逆回転

    pulse_count_B += direction_B

# 前進
def forward(rate_A, rate_B):  # rate: 0～100
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
    if interval == 0:
        return 0
    return (pulse_count * 60) / (PPR * GEAR_RATIO * interval)

if __name__ == '__main__':
    try:
        OUTA_1.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_A)
        OUTA_2.irq(trigger=Pin.IRQ_RISING, handler=pulse_counter_B)
        forward(40, 40)
        start_time = time.ticks_ms()

        while True:
            time.sleep(INTERVAL)

            elapsed_time = (time.ticks_ms() - start_time) / 1000
            rpm_A = compute_rpm(pulse_count_A, INTERVAL)
            rpm_B = compute_rpm(pulse_count_B, INTERVAL)
            
            print(f"Elapsed Time: {elapsed_time:.2f}s, RPM A: {rpm_A:.2f}, Pulse Count A: {pulse_count_A}")
            print(f"Elapsed Time: {elapsed_time:.2f}s, RPM B: {rpm_B:.2f}, Pulse Count B: {pulse_count_B}")

            pulse_count_A = 0
            pulse_count_B = 0

    except KeyboardInterrupt:
        stop()
        print("Stop!!")
