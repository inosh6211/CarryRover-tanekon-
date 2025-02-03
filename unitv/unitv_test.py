import sensor
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

while True:
    img = sensor.snapshot()
    img.replace(vflip = False, hmirror = True, transpose = True)
    time.sleep(0.1)
