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

def forward(rate_a, rate_b):
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
        while True:
            forward(30, 30)
            time.sleep(1)

    except KeyboardInterrupt:
        stop()
