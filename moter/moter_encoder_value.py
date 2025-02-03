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
pulse_count_A = 0
direction_A = 0
pulse_count_B = 0
direction_B = 0

def forward(rate_a, rate_b):
    rate_a = max(0, min(rate_a, 100))
    rate_b = max(0, min(rate_b, 100))
    AIN1.off()
    AIN2.on()
    PWMA.duty_u16(65535 * rate_a // 100)
    BIN1.on()
    BIN2.off()
    PWMB.duty_u16(65535 * rate_b // 100)

def stop():
        AIN1.off()
        AIN2.off()
        BIN1.off()
        BIN2.off()

if __name__ == '__main__':
    try:
        forward(30, 30)
        while True:
            state_A1 = OUTA_1.value()
            state_B1 = OUTB_1.value()
            state_A2 = OUTA_2.value()
            state_B2 = OUTB_2.value()

            print(f"OUTA_1: {state_A1}, OUTB_1: {state_B1}, OUTA_2: {state_A2}, OUTB_2: {state_B2}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        stop()
