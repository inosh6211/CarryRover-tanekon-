from machine import UART
import time

uart0 = UART(0, baudrate=9600, tx=0, rx=1)

while True:
    if uart0.any():
        sentence = uart0.read()
        if sentence:
            try:
                print(sentence.decode('utf-8'))
            except Exception as e:
                print("error", e)
