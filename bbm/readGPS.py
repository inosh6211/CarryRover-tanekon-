from machine import UART
from micropyGPS import MicropyGPS
import utime

uart0 = UART(0, baudrate=9600, tx=0, rx=1)
gps = MicropyGPS(9, 'dd')

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

if __name__ == '__main__':
        lat, long = readGPS()
        print(f"緯度: {lat:.8f}, 経度: {long:.8f}")
