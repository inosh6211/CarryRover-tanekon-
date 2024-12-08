from machine import Pin, PWM
import time

AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
STBY = Pin(9, Pin.OUT)
OUTA_1 = Pin(2, Pin.IN)
OUTB_1 = Pin(3, Pin.IN)

PPR = 3 #PPR = CPR / 4
GEAR_RATIO = 297.92
INTERVAL = 0.3
pulse_count = 0
direction = 0

def pulseCounter(pin):
    global pulse_count, direction
    if OUTB_1.value() == 1:
        direction = 1
    else:
        direction = -1
    pulse_count += direction

def forward(rate): #rate:0～100
    OUTA_1.irq(trigger = Pin.IRQ_RISING, handler = pulseCounter)
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA.duty_u16(2**16 // 100 * rate)

def stop():
    OUTA_1.irq(handler = None)
    AIN1.off()
    AIN2.off()
    STBY.off()

def calculate_rpm(pulse_count, interval):
    return ((pulse_count * 60) / (PPR * GEAR_RATIO * interval))

try:
    forward(50)
    start_time = time.ticks_ms()
    
    while True:
        time.sleep(INTERVAL)
        elapsed_time = (time.ticks_ms() - start_time) / 1000
        rpm = calculate_rpm(pulse_count, INTERVAL)
        print(f"Elapsed Time: {elapsed_time:.2f}s, RPM: {rpm:.2f}")
        pulse_count = 0

except KeyboardInterrupt:
    stop()
    print("Stop!!")
