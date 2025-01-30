from machine import UART
from micropyGPS import MicropyGPS
import time

uart0 = UART(0, baudrate=9600, tx=0, rx=1)
gps = MicropyGPS(9, 'dd')

def read_gps():
    if uart0.any() > 0:
        sentence = uart0.read()
        if sentence:
            try:
                for char in sentence.decode('utf-8'):
                    gps.update(char)
            except Exception as e:
                print("error", e)
                
if __name__ == '__main__':
    lat = 0
    lon = 0
    
    while True:
        read_gps()
        
        #データ取得状況
        if gps.fix_stat > 0:
            if lat != gps.latitude[0] or lon != gps.longitude[0]:
                print("GPS信号を受信")
                lat = gps.latitude[0]
                lon = gps.longitude[0]
                
                #経度、緯度
                print(f"Latitude: {lat:.7f}°, Longitude: {lon:.7f}°")

                #PDOP
                print(f"PDOP: {gps.pdop}")

                #測位可能衛星数
                print(f"Satellites in view: {gps.satellites_in_view}")

        else:
            print("GPS信号を取得できていません")
            time.sleep(1)
