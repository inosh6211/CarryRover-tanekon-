from machine import UART
from micropyGPS import MicropyGPS
import math
import utime

uart0 = UART(0, baudrate=9600, tx=0, rx=1)
gps = MicropyGPS(9, 'dd')

GOAL_LAT, GOAL_LONG = 35.9202211, 139.8996160

def readGPS():
    while True:
        if uart0.any() > 0:
            data = uart0.read(1)
            if data == b'$':
                sentence = '$'
                while True:
                    if uart0.any() > 0:
                        data = uart0.read(1)
                        sentence += data.decode('utf-8')
                        if data == b'\n':
                            if 'GNGGA' in sentence:
                                for x in sentence:
                                    gps.update(x)
                                lat = gps.latitude[0]
                                long = gps.longitude[0]
                                return lat, long
                            break

def distance(lat1, long1, lat2, long2):
    EARTH_RADIUS = 6378137
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)
    
    dlat = lat2 - lat1
    dlong = long2 - long1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlong / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_km = EARTH_RADIUS * c
    return distance_km

def azimuth(lat1, long1, lat2, long2):
    lat1 = math.radians(lat1)
    long1 = math.radians(long1)
    lat2 = math.radians(lat2)
    long2 = math.radians(long2)
    
    dlong = long2 - long1
    x = math.sin(dlong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlong)
    azimuth_rad = math.atan2(x, y)
    azimuth_deg = math.degrees(azimuth_rad)
        
    return azimuth_deg

if __name__ == '__main__':
    while True:
        current_lat, current_long = readGPS()
        print(f"現在地 - 緯度: {current_lat:.8f}, 経度: {current_long:.8f}")

        dist = distance(current_lat, current_long, GOAL_LAT, GOAL_LONG)
        azi = azimuth(current_lat, current_long, GOAL_LAT, GOAL_LONG)
        print(f"目的地までの距離: {dist:.10f} m")
        print(f"目的地までの方位角: {azi:.10f} 度")
