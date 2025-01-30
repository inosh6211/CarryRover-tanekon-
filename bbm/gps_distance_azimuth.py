from machine import UART
from micropyGPS import MicropyGPS
import math
import time

uart0 = UART(0, baudrate=9600, tx=0, rx=1)
gps = MicropyGPS(9, 'dd')
current_lat = 0
current_lon = 0
updated_formats = set()
GOAL_LAT, GOAL_LON = 35.7171796, 139.8232646

def read_gps():
    if uart0.any() > 0:
        sentence = uart0.read()
        if sentence:
            try:
                for char in sentence.decode('utf-8'):
                    gps_format = gps.update(char) 
                    if gps_format:
                        updated_formats.add(gps_format)
            except Exception:
                print("Error")

def distance(lat0, lon0, lat1, lon1): #0:現在地 1:目的地
    EARTH_RADIUS = 6378137  #地球の半径[m]
    
    lat0 = math.radians(lat0)
    lon0 = math.radians(lon0)
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    
    dlat = lat1 - lat0
    dlon = lon1 - lon0
    a = math.sin(dlat / 2)**2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_m = EARTH_RADIUS * c
    return distance_m

def azimuth(lat0, lon0, lat1, lon1): #0:現在地 1:目的地
    lat0 = math.radians(lat0)
    lon0 = math.radians(lon0)
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    
    dlon = lon1 - lon0
    x = math.sin(dlon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
    azimuth_rad = math.atan2(x, y)
    azimuth_deg = math.degrees(azimuth_rad)
    return (azimuth_deg + 360) % 360

if __name__ == '__main__':
    while True:
        read_gps()
        if {'GNGGA', 'GNGSA'} <= updated_formats:
            updated_formats.clear()
            if gps.pdop < 3 and gps.satellites_in_use > 3:
                if current_lat != gps.latitude[0] or current_lon != gps.longitude[0]:
                    current_lat = gps.latitude[0]
                    current_lon = gps.longitude[0]
                    print(f"距離:{distance(current_lat, current_lon, GOAL_LAT, GOAL_LON)}m, 方位角:{azimuth(current_lat, current_lon, GOAL_LAT, GOAL_LON)}°")

        time.sleep(0.1)
