from machine import Pin, UART
import time
import math

#UARTの初期設定（Pico W側、GPIO4=TX, GPIO5=RX）
UART1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

#キャリブレーション結果
TAG_SIZE = 30
FOCAL_LENGTH_X = 210.70  
FOCAL_LENGTH_Y = 210.70

CENTER_X = 100 # ROI_W / 2
CENTER_Y = 100 # ROI_H / 2
DIST_COEFFS = [-0.34155103, 0.25499062, -0.00517389, -0.00731524, -0.17990041]

FOCAL_LENGTH_PX = 210.70
TAG_SIZE = 30

# 色
BLUE = 0
RED = 1
YELLOW = 2


class Camera:
    def __init__(self,uart):
        self.uart = uart
        self.color_cx = []
        self.color_cy = []
        self.color_pixels = []
        self.tag_cx = []
        self.tag_cy = []
        self.tag_pitch = []
        self.tag_yaw = []
        self.tag_distance = []      
            
    #歪み補正
    def undistort_point(self, x, y):
        norm_x = (x - CENTER_X) / FOCAL_LENGTH_X
        norm_y = (y - CENTER_Y) / FOCAL_LENGTH_Y
        r2 = norm_x**2 + norm_y**2
        radial_distortion = 1 + DIST_COEFFS[0] * r2 + DIST_COEFFS[1] * (r2**2) + DIST_COEFFS[4] * (r2**3)
        
        corrected_x = norm_x * radial_distortion * FOCAL_LENGTH_X + CENTER_X
        corrected_y = norm_y * radial_distortion * FOCAL_LENGTH_Y + CENTER_Y
      
        return corrected_x, corrected_y

    def rad_camera(self):
        while not self.uart.any():
            time.sleep(0.01)
                
        while uart1.any() > 0:
            message += self.uart.read().decode('utf-8').strip()
            
        data = message.split(',')
        
        return data

    def compute_tag_data(self, c0, c1, c2, c3):
        tag_width = math.sqrt((c0 - c1)**2 + (c0 - c1)**2)
        tag_height = math.sqrt((c0 - c3)**2 + (c0 - c3)**2)
        tag_size_avg = (tag_width + tag_height) / 2
        if tag_size_avg > 0:
            tag_distance = (FOCAL_LENGTH_PX * TAG_SIZE) / tag_size_avg  # mm単位
        else:
            tag_distance = -1
        
        return tag_pitch, tag_yaw, tag_distance

    """
    0: Red, 1: Blue, 2:Yellow
    """
    def read_color(self, color):
        self.uart.write(f"0, {color}\n")
        data = self.read_camera()
        if len(data) == 5 and data[0] == "Color":
            cx, cy = float(data[color + 1]), (data[color + 2])
            self.color_cx, self.color_cy = undistort_point(cx, cy)
            self.color_pixels = float(data[color + 3])
            
            return True
        
        return False

    def read_tag(self, tag_id):
        self.uart.write("1, {tag_id}\n")    
        data = read_camera()
        if len(data) == 7 and data[0] == "Tag":
            cx, cy = float(data[tag_id + 1]), float(data[tag_id + 2])
            self.tag_cx, self.tag_cy = undistort_point(cx, cy)
            c0, c1, c2, c3 = float(data[tag_id + 3]), float(data[tag_id + 4]), float(data[tag_id + 5]), float(data[tag_id + 6])
            self.tag_pitch, self.tag_yaw, self.tag_distance = compute_tag_info(c0, c1, c2, c3)
            
            return True
        
        return False
        

if __name__ == "__main__":
    camera = Camera(UART1)
    if read_color(BLUE):
        print(f"{camera.color_cx[BLUE]}, {camera.color_cy[BLUE]}")

