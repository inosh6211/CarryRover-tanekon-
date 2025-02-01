import machine
import time
from bno055 import *
i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))  


bno = BNO055(i2c)

while True:
    bno.iget(QUAT_DATA)
    print(f"w:{bno.w}")
    print(f"x:{bno.x}")
    print(f"y:{bno.y}")
    print(f"z:{bno.z}")
    time.sleep(0.1)
