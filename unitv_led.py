import time
from modules import ws2812

leds = ws2812(8, 1)

while True:
    leds.set_led(0, (0, 255, 0))  # 赤色
    leds.display()
    time.sleep(0.5)

    leds.set_led(0, (0, 0, 0))  # 消灯
    leds.display()
    time.sleep(0.5)
