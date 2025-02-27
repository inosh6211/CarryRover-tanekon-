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


class Camera:
    def __init__(self,uart):
        self.uart = uart
        self.color_cx = []
        self.color_cy = []
        self.color_pixels = []
        self.tag_cx = []
        self.tag_cy = []
        self.tag_distance = []
        self.tag_pitch = []
            
    #歪み補正
    def undistort_point(self, x, y):
        norm_x = (x - CENTER_X) / FOCAL_LENGTH_X
        norm_y = (y - CENTER_Y) / FOCAL_LENGTH_Y
        r2 = norm_x**2 + norm_y**2
        radial_distortion = 1 + DIST_COEFFS[0] * r2 + DIST_COEFFS[1] * (r2**2) + DIST_COEFFS[4] * (r2**3)
        
        corrected_x = norm_x * radial_distortion * FOCAL_LENGTH_X + CENTER_X
        corrected_y = norm_y * radial_distortion * FOCAL_LENGTH_Y + CENTER_Y
      
        return corrected_x, corrected_y

    def read_camera(self):
        while not self.uart.any():
            time.sleep(0.01)
                
        while uart1.any() > 0:
            message += self.uart.read().decode('utf-8').strip()
            
        data = message.split(',')
        
        return data

    def read_color(self):
        self.uart.write("0\n")
        data = self.read_camera()
        if data[0] == "c":
            for color in range(3):
            cx, cy = float(data[color + 1]), (data[color + 2])
            self.color_cx, self.color_cy = undistort_point(cx, cy)
            self.color_pixels = float(data[color + 3])

    def read_tag(self):
        self.uart.write("1\n")    
        data = read_camera()
        if data[0] == "t":
            for tag_id in range(10):
            cx, cy = float(data[tag_id + 1]), float(data[tag_id + 2])
            self.tag_cx, self.tag_cy = undistort_point(cx, cy)
            self.tag_distance = float(data[tag_id + 3])
            self.tag_pitch = float(data[tag_id + 4])
        

if __name__ == "__main__":
    camera = Camera(UART1)
    
    while True:
        camera.read_color()
        camra.read_tag()
        for i in range(3):
            print(f"camera.color_cx = [i], self.color_cy = [i], self.color_pixels = []")
            
        for j in range(10):
            print(camera.tag_cx = [j], camera.tag_cy = [j], camera.tag_distance = [j], camera.tag_pitch = [j])
        
        time.sleep(0.1)
