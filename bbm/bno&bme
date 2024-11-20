import utime
import math
from machine import Pin,I2C
from bme280 import BME280


#I2C設定（BNO055）
i2c = I2C(0,scl=Pin(21), sda=Pin(20),freq=100000)
#I2C設定（BME280）
i2c = I2C(0,scl=Pin(21), sda=Pin(20),freq=100000)
bme280 = BME280(i2c=i2c)

#BNO055のI2Cアドレス
BNO055_ADDRESS = 0x28

ACC = 0x08
MAG = 0x09
GYRO = 0x0A
MODE_REGISTER = 0x3D
NDOF_MODE = 0x0C

# センサーをNDOFモードに設定(<---これやらないと取得した角度が0になることがある)
i2c.writeto_mem(BNO055_ADDRESS, MODE_REGISTER, bytes([NDOF_MODE]))
utime.sleep(0.1)  # 設定反映のための待機

#加速度、ジャイロ、地磁気を取得する関数
def read_data(register):
    #レジスタからデータを読み取る
    data = i2c.readfrom_mem(BNO055_ADDRESS, register, 6)
    return data

#16ビット符号付整数に変換関数
def to_signed_16bit(value):
    if value > 32767:
        value -= 65536
    return value

#オイラー角を計算
def calculate_euler_angles(acc_data, mag_data):
    #加速度
    acc_x = to_signed_16bit((acc_data[1] << 8) | acc_data[0])
    acc_y = to_signed_16bit((acc_data[3] << 8) | acc_data[2])
    acc_z = to_signed_16bit((acc_data[5] << 8) | acc_data[4])
    
    #地磁気
    mag_x = to_signed_16bit((mag_data[1] << 8) | mag_data[0])
    mag_y = to_signed_16bit((mag_data[3] << 8) | mag_data[2])
    mag_z = to_signed_16bit((mag_data[5] << 8) | mag_data[4])

    #Euler角（ピッチ、ロール、ヨー）
    pitch = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)) * 180.0 / math.pi
    roll = math.atan2(-acc_x, acc_z) * 180.0 / math.pi
    
    heading = math.atan2(mag_y, mag_x) * 180.0 / math.pi
    
    #結果を返す
    return pitch,roll,heading

while True:
    #加速度と地磁気の読み取り
    acc_data = read_data(ACC)
    mag_data = read_data(MAG)
    
    #気圧を読み取る
    temperature, pressure, humidity = bme280.read_compensated_data()
    
    #オイラー角を返す
    pitch, roll, heading = calculate_euler_angles(acc_data, mag_data)
    
    #出力を表示
    print("Pitch =",pitch, "Roll =",roll, "Yaw =",heading)
    print("------------------")
    print("Pressure: {:.2f}hPa".format(pressure/100)) #hpaに変換
    
    #データの取得間隔を定義
    utime.sleep(0.1)
                                

