from machine import Pin, UART
import time

UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

class CameraReceiver:
    def __init__(self, uart):
        self.uart = uart
        
        time.sleep(2)
    
        # UnitVとの接続確認
        while True:
            self.uart.write("1\n")
            
            while not self.uart.any():
                print("Waiting for Unitv signal...")
                time.sleep(0.5)
                
            message = self.uart.readline().decode('utf-8').strip()
            
            if message == "1":
                print("Unitv connected")
                break
            
            time.sleep(0.1)
            

    def read_camera(self):
        message = ""
        
        while True:
            if self.uart.any():
                char = self.uart.read(1).decode('utf-8')
                if char == "\n":
                    break
                message += char
                
                time.sleep(0.01)
        
        print(message)
        
        return message.split(',')
    
    def detect_para(self):
        # 0: Red, 1: Blue
        self.para_pixels = 0
        self.para_cx = 0
        self.para_cy = 0
        
        self.uart.write("P\n")
        data = self.read_camera()
        
        if len(data) == 4:
            if data[0] == "P":
                self.para_pixels = int(data[1])
                self.para_cx = int(data[2])
                self.para_cy = int(data[3])

    def read_color(self):
        # 0: Red, 1: Blue
        self.color_pixels = [0] * 2
        self.color_cx = [0] * 2
        self.color_cy = [0] * 2
        self.aspect_ratio = [0] * 2
        self.color_rotation = [0] * 2
        
        self.uart.write("C\n")
        data = self.read_camera()
        
        if data[0] == "C":
            if (len(data) - 1) % 6 == 0:
                if len(data) > 1:
                    for i in range((len(data) - 1) // 6):
                        color = int(data[i * 6 + 1])
                        self.color_pixels[color] = int(data[i * 6 + 2])
                        self.color_cx[color] = int(data[i * 6 + 3])
                        self.color_cy[color] = int(data[i * 6 + 4])
                        self.aspect_ratio[color] = float(data[i * 6 + 5])
                        self.color_rotation[color] = float(data[i * 6 + 6])
    
    def read_tags(self, mode):
        self.tag_detected = [0] * 10
        self.tag_cx = [0] * 10
        self.tag_cy = [0] * 10
        self.tag_distance = [0] * 10
        self.tag_roll = [0] * 10
        self.tag_pitch = [0] * 10
        
        self.uart.write(f"T{mode}\n")
        data = self.read_camera()

        if data[0] == "T":
            if len(data) > 1:
                if (len(data) - 1) % 6 == 0:
                    for i in range((len(data) - 1) // 6):
                        tag_id = int(data[i * 6 + 1])
                        self.tag_detected[tag_id] = 1
                        self.tag_cx[tag_id] = int(data[i * 6 + 2])
                        self.tag_cy[tag_id] = int(data[i * 6 + 3])
                        self.tag_distance[tag_id] = float(data[i * 6 + 4])
                        self.tag_roll[tag_id] = float(data[i * 6 + 5])
                        self.tag_pitch[tag_id] = float(data[i * 6 + 6])


if __name__ == "__main__":
    cam = CameraReceiver(UART1)
    
    while True:
        cam.read_color()
        for i in range(2):
            print(cam.color_pixels[i], cam.color_cx[i], cam.color_cy[i], cam.aspect_ratio[i], cam.color_rotation[i])
        
