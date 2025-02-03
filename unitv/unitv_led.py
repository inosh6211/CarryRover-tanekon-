import time
from modules import ws2812

led = ws2812(8, 1)

while True:
    led.set_led(0, (0, 255, 0))  
    led.display()
    time.sleep(0.5)

    led.set_led(0, (0, 0, 0)) 
    led.display()
    time.sleep(0.5)
