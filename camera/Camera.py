from machine import Pin, UART
import time
import math

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
            
        data = message.split(',')
        
        return data

    def read_color(self):
        self.uart.write("c\n")
        data = self.read_camera()
        
        if data[0] == "c":
            self.color_pixels = []
            self.color_cx = []
            self.color_cy = []
            
            for color in range(3):
                self.color_pixels.append(float(data[color*3 + 1]))
                self.color_cx.append(float(data[color*3 + 2]))
                self.color_cy.append(float(data[color*3 + 3]))

    def read_tag(self):
        self.uart.write("t\n")    
        data = self.read_camera()
        
        if data[0] == "t":
            self.tag_detected = []
            self.tag_cx = []
            self.tag_cy = []
            self.tag_distance = []
            self.tag_pitch = []
            
            for tag_id in range(10):
                self.tag_detected.append(int(data[tag_id*5 + 1]))
                self.tag_cx.append(float(data[tag_id*5 + 2]))
                self.tag_cy.append(float(data[tag_id*5 + 3]))
                self.tag_distance.append(float(data[tag_id*5 + 4]))
                self.tag_pitch.append(float(data[tag_id*5 + 5]))


if __name__ == "__main__":
    camera = CameraReceiver(UART1)
    time.sleep(2)
    
    while True:
        camera.read_color()
        for i in range(3):
            print(i, camera.color_pixels[i], camera.color_cx[i], camera.color_cy[i])
        
        camera.read_tag()    
        for j in range(10):
            print(j, camera.tag_cx[j], camera.tag_cy[j], camera.tag_distance[j], camera.tag_pitch[j])
        

        time.sleep(0.1)

