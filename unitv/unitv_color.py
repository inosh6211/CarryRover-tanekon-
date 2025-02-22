import sensor
import image
import time
import machine
from modules import ws2812
import lcd
import math
from fpioa_manager import fm
from Maix import GPIO
from machine import UART

# LCDの初期化
lcd.init()
lcd.rotation(2)

# LEDの設定
led = ws2812(8, 1)
led.set_led(0, (0, 255, 0))  # 緑色に点灯
led.display()

# UARTの設定
fm.register(34, fm.fpioa.UART1_TX, force=True)
fm.register(35, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, baudrate=115200, read_buf_len=1024)

# カメラの初期
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

# カラー閾値（LAB）
red_threshold = (20, 130, 40, 80, 40, 80)
orange_threshold = (30, 79, 102, 13, -12, 78)
pink_threshold = (75, 97, 28, 7, -9, 8)

# 起動確認用めっさげ
print("UnitV Ready. Waiting for objects...")  

# カメラパラメータ
FOCAL_LENGTH = 250
IMG_W, IMG_H = sensor.width(), sensor.height()
CX, CY = IMG_W // 2, IMG_H // 2

# フレームカウントの初期化
frame_count = 0
log_interval = 10  # 何フレームごとにログを出力するか

while True:
  img = sensor.snapshot()
  img.replace(vflip=False, hmirror=True, transpose=True)

  color_dict = {
  "red": (red_threshold, (255, 0, 0)),
  "orange": (orange_threshold, (255, 165, 0)),
  "pink": (pink_threshold, (255, 192, 203))
  }

   detected_objects = []
   for color_name, (threshold, draw_color) in color_dict.items():
                    blobs = img.find_blobs([threshold], pixels_threshold=200, area_threshold=200, merge=True)
     if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())
        x, y, w, h = largest_blob.rect()
        x_center = x + w // 2
        y_center = y + h // 2

        img.draw_rectangle(largest_blob.rect(), color=draw_color)
        img.draw_cross(x_center, y_center, color=(0, 255, 0))
        img.draw_string(x_center + 5, y_center, color_name, color=(255, 255, 255))
        detected_objects.append(f"{color_name}:{x_center},{y_center}")
       
   if detected_objects:
       message = ",".join(detected_objects) + "\n"
       uart.write(message)
     
       # 一定フレームごとにログを表示
       if frame_count % log_interval == 0:
            print("Blob detected: {}".format(detected_objects))
            print("Sending data: {}".format(message))
   else:
      uart.write("$0\n")
      print("No object detected.")
     
 lcd.display(img)

frame_count += 1  # フレームカウンターを更新
time.sleep(0.1)
