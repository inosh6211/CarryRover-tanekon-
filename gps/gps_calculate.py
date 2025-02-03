from machine import UART
from micropyGPS import MicropyGPS
import math
import time

uart0 = UART(0, baudrate=9600, tx=0, rx=1)
gps = MicropyGPS(9, 'dd')
updated_formats = set()
gps_updated = 0
lat = 0
lon = 0
GOAL_LAT, GOAL_LON = 35.7171709, 139.8232740

def read_nmea():
    if uart0.any() > 0:
        sentence = uart0.read()
        if sentence:
            try:
                for char in sentence.decode('utf-8'):
                    gps_format = gps.update(char) 
                    if gps_format:
                        updated_formats.add(gps_format)
            except Exception as e:
                print(f"Error: {e}")

def get_lat_lon():
    global gps_updated, lat, lon
    if {'GNGGA', 'GNGSA'} <= updated_formats:
        updated_formats.clear()
        if gps.pdop < 3 and gps.satellites_in_use > 3:
            if lat != gps.latitude[0] or lon != gps.longitude[0]:
                gps_updated = 1
                lat = gps.latitude[0]
                lon = gps.longitude[0]
    else:
        gps_updated = 0

def calculate_distance():
    EARTH_RADIUS = 6378137
    
    lat0, lon0, lat1, lon1 = map(math.radians, [lat, lon, GOAL_LAT, GOAL_LON])
    dlat = lat1 - lat0
    dlon = lon1 - lon0
    a = math.sin(dlat / 2)**2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

def calculate_azimuth():
    lat0, lon0, lat1, lon1 = map(math.radians, [lat, lon, GOAL_LAT, GOAL_LON])
    dlon = lon1 - lon0
    x = math.sin(dlon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
    azimuth_rad = math.atan2(x, y)
    return (math.degrees(azimuth_rad) + 360) % 360

if __name__ == '__main__':
    while True:
        read_nmea()
        get_lat_lon()
        if gps_updated == 1:
            print(f"緯度: {lat:.7f}, 経度: {lon:.7f}")
            print(f"距離: {calculate_distance():.2f} m, 方位角: {calculate_azimuth():.2f}°")
        
        time.sleep(0.1)
