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
                print("Error decoding sentence:", e)

while True:
    read_gps()
    
    #データ取得状況
    if gps.fix_stat > 0:
        print("GPS信号を受信")
        
        #経度、緯度
        print(f"Latitude: {gps.latitude[0]}°, Longitude: {gps.longitude[0]}°")

        #PDOP
        print(f"PDOP: {gps.pdop}")

        #測位可能衛星数
        print(f"Satellites in view: {gps.satellites_in_view}")

        #信号強度
        print("Signal Strengths (SNR):")
        for satellite_id, data in gps.satellite_data.items():
            elevation, azimuth, snr = data
            print(f"  Satellite ID: {satellite_id}, SNR: {snr}")

    else:
        print("GPS信号を取得できていません")
        time.sleep(1)
