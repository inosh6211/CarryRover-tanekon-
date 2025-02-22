from machine import I2C, Pin
from servo import Servos
import utime

I2C1 = I2C(1, scl=Pin(27), sda=Pin(26))

# アームの初期位置
ARM_INIT_ANGLE0 = 90
ARM_INIT_ANGLE1 = 180
ARM_INIT_ANGLE2 = 0
ARM_INIT_ANGLE3 = 0
ARM_INIT_ANGLE4 = 10


class ControlArm:
    def __init__(self, i2c):
        self.servos = Servos(i2c)
        self.angles = [
            ARM_INIT_ANGLE0,
            ARM_INIT_ANGLE1,
            ARM_INIT_ANGLE2,
            ARM_INIT_ANGLE3,
            ARM_INIT_ANGLE4
        ]

    def control_servo(self, index, angle):
        self.servos.position(index, degrees=angle)
        self.angles[index] = angle

if __name__ == "__main__":
    arm = ControlArm(I2C1)
