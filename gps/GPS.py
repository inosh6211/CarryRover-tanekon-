from machine import Pin, UART
from micropyGPS import MicropyGPS
import math
import time

UART0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

EARTH_RADIUS = 6378137  # 地球の半径(m)
GOAL_LAT,GOAL_LON = 35.9186300, 139.9081696  # 7号館

class GPS:
    def __init__(self, uart):
        self.uart = uart
        self.micropygps = MicropyGPS(9, 'dd')
        self.updated_formats = set()
        self.lat = 0
        self.lon = 0
        self.distance = 0
        self.azimuth = 0

    def read_nmea(self):
        if self.uart.any() > 0:
            sentence = self.uart.read()
            if sentence is None:
                return
            try:
                for char in sentence.decode('utf-8'):
                    gps_format = self.micropygps.update(char)
                    if gps_format:
                        self.updated_formats.add(gps_format)
            except Exception as e:
                print(f"Error: {e}")

    def compute_distance(self, goal_lat, goal_lon):
        lat0, lon0, lat1, lon1 = map(math.radians, [self.lat, self.lon, goal_lat, goal_lon])
        dlat = lat1 - lat0
        dlon = lon1 - lon0
        a = math.sin(dlat / 2)**2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        self.distance = EARTH_RADIUS * c

    def compute_azimuth(self, goal_lat, goal_lon):
        lat0, lon0, lat1, lon1 = map(math.radians, [self.lat, self.lon, goal_lat, goal_lon])
        dlon = lon1 - lon0
        x = math.sin(dlon) * math.cos(lat1)
        y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
        azimuth_rad = math.atan2(x, y)
        self.azimuth = (math.degrees(azimuth_rad) + 360) % 360

    def update_data(self, goal_lat, goal_lon):
        if {'GNGGA', 'GNGSA'} <= self.updated_formats:
            self.updated_formats.clear()
            if self.micropygps.pdop < 3 and self.micropygps.satellites_in_use > 3:
                try:
                    lat = self.micropygps.latitude[0]
                    lon = self.micropygps.longitude[0]
                    if lat is not None and lon is not None:
                        if self.lat != lat or self.lon != lon:
                            self.lat = lat
                            self.lon = lon
                            self.compute_distance(goal_lat, goal_lon)
                            self.compute_azimuth(goal_lat, goal_lon)
                            return True
                except IndexError as e:
                    print(f"Error: {e}")
        return False

if __name__ == '__main__':
    gps = GPS(UART0)
    
    while True:
        gps.read_nmea()
        if gps.update_data(GOAL_LAT, GOAL_LON):
            print(f"距離: {gps.distance:.7f} m, 方位角: {gps.azimuth:.7f}°")
            
        time.sleep(0.5)
