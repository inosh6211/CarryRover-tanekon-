from machine import Pin
import time

fusing_gpio = Pin(8, Pin.OUT, value = 0)

def fusing():
    fusing_gpio.on()
    time.sleep(0.3)
    fusing_gpio.off()
    
if __name__ == '__main__':
    try:
        fusing()
    except KeyboardInterrupt:
        print("Stopped!")
