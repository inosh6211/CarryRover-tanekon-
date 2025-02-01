from machine import Pin, I2C
from bme280_float import BME280



# I2Cの設定
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)  
bme = BME280(i2c=i2c)

# データを格納する配列
result = [0, 0, 0]

# 補正データを読み取る
bme.read_compensated_data(result)

# 結果を表示
temperature = result[0]
pressure = result[1]
humidity = result[2]

print("Temperature: {:.2f} °C".format(temperature))
print("Pressure: {:.2f} hPa".format(pressure / 100))  # hPaに変換
print("Humidity: {:.2f} %".format(humidity))



