from machine import Pin, UART
import time

UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart

    def read_camera(self):
        message = ""
        while not self.uart.any():
            time.sleep(0.01)
                
        while self.uart.any() > 0: 
            message += self.uart.read().decode('utf-8').strip()
            
        return message.split(',')

    def read_color(self, command):
        self.uart.write(command + "\n")
        data = self.read_camera()

        if data[0] == command:
            return int(data[1]), int(data[2]), int(data[3])  # pixels, cx, cy
        return 0, 0, 0  # 検出なしの場合

    def read_tags(self):
        self.uart.write("T\n")
        data = self.read_camera()

        tag_results = []
        if data[0] == "T":
            for i in range(1, len(data), 6):
                tag_id = int(data[i])
                cx = int(data[i + 1])
                cy = int(data[i + 2])
                distance = float(data[i + 3])
                yaw = float(data[i + 4])
                pitch = float(data[i + 5])
                tag_results.append((tag_id, cx, cy, distance, yaw, pitch))

        return tag_results

if __name__ == "__main__":
    camera = CameraReceiver(UART1)
    time.sleep(2)
    
    while True:
        for color in ["P", "R", "B"]:
            pixels, cx, cy = camera.read_color(color)
            print("{}: pixels={}, cx={}, cy={}".format(color, pixels, cx, cy))#本番のコードでは最初の:やpixels=などはなくして大丈夫です

        tags = camera.read_tags()
        for tag in tags:
            print("Tag {}: cx={}, cy={}, distance={}, yaw={}, pitch={}".format(*tag))#こちらも同様

        time.sleep(0.1)

