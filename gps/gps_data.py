from machine import Pin, UART
from micropyGPS import MicropyGPS
import time

uart0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
gps = MicropyGPS(9, 'dd')
current_lat = 0
current_lon = 0
updated_formats = set()

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

if __name__ == '__main__':    
    while True:
        read_gps()
        if {'GNGGA', 'GNGSA'} <= updated_formats:
            updated_formats.clear()
            if gps.pdop < 3 and gps.satellites_in_use > 3:
                if current_lat != gps.latitude[0] or current_lon != gps.longitude[0]:
                    current_lat = gps.latitude[0]
                    current_lon = gps.longitude[0]
                    print(f"現在地 - 緯度: {gps.latitude[0]:.7f}, 経度: {gps.longitude[0]:.7f}")
                    print(f"PDOP: {gps.pdop}")
                    print(f"使用衛星数: {gps.satellites_in_use}")

        time.sleep(0.1)
