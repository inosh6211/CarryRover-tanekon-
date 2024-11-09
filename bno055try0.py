import utime
from machine import I2C, Pin
import math

# I2C設定
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)

# BNO055のI2Cアドレス
BNO055_ADDRESS = 0x28

# アドレス
ACC = 0x08
MAG = 0x09
GYRO = 0x0A
MODE_REGISTER = 0x3D
NDOF_MODE = 0x0C

# センサーをNDOFモードに設定(<---これやらないと取得した角度が0になることがある)
i2c.writeto_mem(BNO055_ADDRESS, MODE_REGISTER, bytes([NDOF_MODE]))
utime.sleep(0.1)  # 設定反映のための待機

# 加速度、ジャイロ、地磁気データを取得する関数
def read_data(register):
    # レジスタからデータを読み取る（6バイト）
    data = i2c.readfrom_mem(BNO055_ADDRESS, register, 6)
    return data

# 16ビットの符号付き整数に変換する関数
def to_signed_16bit(value):
    if value > 32767:
        value -= 65536
    return value

# オイラー角を計算する関数
def calculate_euler_angles(acc_data, mag_data):
    # 加速度データ (X, Y, Z)
    acc_x = to_signed_16bit((acc_data[1] << 8) | acc_data[0])
    acc_y = to_signed_16bit((acc_data[3] << 8) | acc_data[2])
    acc_z = to_signed_16bit((acc_data[5] << 8) | acc_data[4])
    
    # 地磁気データ (X, Y, Z)
    mag_x = to_signed_16bit((mag_data[1] << 8) | mag_data[0])
    mag_y = to_signed_16bit((mag_data[3] << 8) | mag_data[2])
    mag_z = to_signed_16bit((mag_data[5] << 8) | mag_data[4])

    # ピッチとロールの計算
    pitch = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)) * 180.0 / math.pi
    roll = math.atan2(-acc_x, acc_z) * 180.0 / math.pi

    # ヨー（地磁気を使って計算）
    heading = math.atan2(mag_y, mag_x) * 180.0 / math.pi

    # 結果を返す
    return pitch, roll, heading

while True:
    # 加速度データと地磁気データの読み取り
    acc_data = read_data(ACC)
    mag_data = read_data(MAG)

    # データの表示（デバッグ用）
    #print("Raw acceleration data:", acc_data)
    #print("Raw magnetic data:", mag_data)

    # オイラー角（ピッチ、ロール、ヨー）を計算
    pitch, roll, heading = calculate_euler_angles(acc_data, mag_data)

    # オイラー角を表示
    print("Pitch =", pitch, "Roll =", roll, "Heading (Yaw) =", heading)
    print("----------")

    # 0.1秒間隔でデータを取得
    utime.sleep(0.1)

