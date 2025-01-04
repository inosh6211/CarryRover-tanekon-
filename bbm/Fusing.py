from machine import Pin
import time

fusing = Pin(8, Pin.OUT)
fusing.off()

def Fusing():
    fusing.on()
    time.sleep_ms(100)
    fusing.off()
        

if __name__ == '__main__':
    Fusing()
    print('Fusing')
