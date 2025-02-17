import machine
from bme280 import BME280

i2c = machine.I2C(0, sda=machine.Pin(20), scl=machine.Pin(21))

bme = BME280(i2c=i2c)

print(f"温度: {bme.temperature():.2f} ℃")
print(f"気圧: {bme.pressure():.2f} hPa")
print(f"湿度: {bme.humidity():.2f} %")
