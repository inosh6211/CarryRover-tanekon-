from machine import Pin, UART
from micropyGPS import MicropyGPS
import math
import time

class GPS:
    EARTH_RADIUS = 6378137  # 地球の半径(m)

    def __init__(self, uart, tx, rx, baudrate, goal_lat, goal_lon):
        self.uart = UART(uart, baudrate=baudrate, tx=Pin(tx), rx=Pin(rx))
        self.micropygps = MicropyGPS(9, 'dd')
        self.updated_formats = set()
        self.goal_lat = goal_lat
        self.goal_lon = goal_lon
        self.lat = 0
        self.lon = 0
        self.distance = 0
        self.azimuth = 0

    def read_nmea(self):
        if self.uart.any() > 0:
            sentence = self.uart.read()
            if sentence:
                try:
                    for char in sentence.decode('utf-8'):
                        gps_format = self.micropygps.update(char)  # ✅ `micropygps` に変更
                        if gps_format:
                            self.updated_formats.add(gps_format)
                except Exception as e:
                    print(f"Error: {e}")

    def compute_distance(self):
        lat0, lon0, lat1, lon1 = map(math.radians, [self.lat, self.lon, self.goal_lat, self.goal_lon])
        dlat = lat1 - lat0
        dlon = lon1 - lon0
        a = math.sin(dlat / 2)**2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        self.distance = self.EARTH_RADIUS * c

    def compute_azimuth(self):
        lat0, lon0, lat1, lon1 = map(math.radians, [self.lat, self.lon, self.goal_lat, self.goal_lon])
        dlon = lon1 - lon0
        x = math.sin(dlon) * math.cos(lat1)
        y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
        azimuth_rad = math.atan2(x, y)
        self.azimuth = (math.degrees(azimuth_rad) + 360) % 360
    
    def update_data(self):
        if {'GNGGA', 'GNGSA'} <= self.updated_formats:
            self.updated_formats.clear()
            if self.micropygps.pdop < 3 and self.micropygps.satellites_in_use > 3:  # ✅ `micropygps` に変更
                if self.lat != self.micropygps.latitude[0] or self.lon != self.micropygps.longitude[0]:
                    self.lat = self.micropygps.latitude[0]
                    self.lon = self.micropygps.longitude[0]    
                    self.compute_distance()
                    self.compute_azimuth()
                    return True
        return False

if __name__ == '__main__':
    gps = GPS(uart=0, tx=0, rx=1, baudrate=9600, goal_lat=35.7171709, goal_lon=139.8232740)
    
    while True:
        gps.read_nmea()
        if gps.update_data():
            print(f"距離: {gps.distance:.2f} m, 方位角: {gps.azimuth:.2f}°")
    
        time.sleep(0.1)
