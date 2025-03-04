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
    try:
        message = self.uart.readline().decode('utf-8').strip()  # **修正: 1行全体を受信**
        if not message:
            return []  # 空行なら空リストを返す
        return message.split(',')
    except Exception as e:
        print("UART Read Error:", e)
        return []  # エラー発生時も空リストを返す


    def read_color(self):
    self.color_pixels = [0] * 3
    self.color_cx = [0] * 3
    self.color_cy = [0] * 3

    self.uart.write("C\n")
    data = self.read_camera()

    if len(data) < 2 or data[0] != "C":
        print("Invalid color data:", data)
        return

    num_colors = (len(data) - 1) // 4
    num_colors = min(num_colors, 3)  # 最大3色

    for i in range(num_colors):
        idx = i * 4 + 1
        if idx + 3 >= len(data):
            continue  # **修正: インデックス範囲外の参照を防ぐ**
        try:
            color = int(data[idx])
            self.color_pixels[color] = int(data[idx + 1])
            self.color_cx[color] = int(data[idx + 2])
            self.color_cy[color] = int(data[idx + 3])
        except (IndexError, ValueError):
            print("Malformed color data:", data)

            
    def read_tags(self, mode):
    self.tag_detected = [0] * 10
    self.tag_cx = [0] * 10
    self.tag_cy = [0] * 10
    self.tag_distance = [0] * 10
    self.tag_roll = [0] * 10
    self.tag_pitch = [0] * 10

    self.uart.write(f"T{mode}\n")
    data = self.read_camera()

    if len(data) < 2 or data[0] != "T":
        print("Invalid tag data:", data)
        return

    num_tags = (len(data) - 1) // 6
    num_tags = min(num_tags, 10)  # 最大10タグ

    for i in range(num_tags):
        idx = i * 6 + 1
        if idx + 5 >= len(data):
            continue  # **修正: インデックス範囲外の参照を防ぐ**
        try:
            tag_id = int(data[idx])
            self.tag_detected[tag_id] = 1
            self.tag_cx[tag_id] = int(data[idx + 1])
            self.tag_cy[tag_id] = int(data[idx + 2])
            self.tag_distance[tag_id] = float(data[idx + 3])
            self.tag_roll[tag_id] = float(data[idx + 4])
            self.tag_pitch[tag_id] = float(data[idx + 5])
        except (IndexError, ValueError):
            print("Malformed tag data:", data)
            
if __name__ == "__main__":
    cam = CameraReceiver(UART1)
    
    while True:
        cam.read_tags(0)
        print(cam.tag_detected[6])
        time.sleep(0.1)
    
    """
    while True:
        cam.read_color()
        print(f"Blue: {cam.color_pixels[1]}, {cam.color_cx[1]}, {cam.color_cy[1]}")
        time.sleep(0.1)
    """
