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
INTERVAL = 0.1
pulse_count = 0
direction = 0

Kp = 0.2
target_rpm = 60

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

def calculateRPM(pulse_count, interval):
    rpm = (pulse_count * 60) / (PPR * GEAR_RATIO * interval)
    return rpm

def moterPcontral():
    global pulse_count, target_rpm
    rate = 0
    currrent_rpm = 0
    while True:
        forward(int(rate))
        time.sleep(0.1)
        current_rpm = calculateRPM(pulse_count, 0.1)
        pulse_count = 0
        diff_rpm = target_rpm - current_rpm
        rate += Kp * diff_rpm
        print(current_rpm)
    
try:
    moterPcontral()
    
except KeyboardInterrupt:
    stop()
    print("Stop!!")
