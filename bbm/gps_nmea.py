from machine import UART
import time

uart0 = UART(0, baudrate=9600, tx=0, rx=1)

if __name__ == '__main__':
    while True:
        if uart0.any():
            sentence = uart0.read()
            if sentence:
                try:
                    print(sentence.decode('utf-8'))
                except Exception:
                    print("Error:")

        time.sleep(0.1)
