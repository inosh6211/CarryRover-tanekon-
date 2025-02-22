from machine import Pin, SPI, PWM, Timer
from ble_simple_peripheral import BLESimplePeripheral
import os
import time
import math
import bluetooth
import sdcard
import _thread

DEVICE_NAME = "RPpicoW"
FILE_NAME = "CarryRover"
SPI1 = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
SPI1_CS = Pin(13, Pin.OUT)

PPR         = 3         # PPR = CPR / 4
GEAR_RATIO  = 297.92    # ギア比
FREQ        = 20        # タイマー割り込みの周波数 [Hz]
KP_RPM      = 0.5       # P制御の比例ゲイン

# モーターの状態
STOP       = 0
FORWARD    = 1
TURN_R     = 2
TURN_L     = 3
BACKWARD   = 4

# モータードライバピン設定
AIN1 = Pin(18, Pin.OUT)
AIN2 = Pin(17, Pin.OUT)
PWMA = PWM(Pin(16))
PWMA.freq(1000)
BIN1 = Pin(19, Pin.OUT)
BIN2 = Pin(22, Pin.OUT)
PWMB = PWM(Pin(28))
PWMB.freq(1000)
STBY = Pin(9, Pin.OUT, value=1)

# エンコーダピン設定
OUTA_A = Pin(3, Pin.IN)
OUTB_A = Pin(2, Pin.IN)
OUTA_B = Pin(7, Pin.IN)
OUTB_B = Pin(6, Pin.IN)


class Logger:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.sd = None
        self.file_name = None
        self.sd_state = False
        self.ble = BLESimplePeripheral(bluetooth.BLE(), name=DEVICE_NAME)
        
        time.sleep(1)
        while not self.ble.is_connected():
            print("Waiting for Bluetooth connection...")
            time.sleep(1)
        print("Bluetooth connected")
        time.sleep(1)
        
        try:
            self.sd = sdcard.SDCard(self.spi, self.cs)
            os.mount(self.sd, "/sd")
            self.create_file()
            self.sd_state = True
        except Exception as e:
            print(f"SD card not detected: {e}")
            if self.ble.is_connected():
                self.ble.send(f"SD card not detected: {e}")
        time.sleep(1)
        
        self.log_queue = []
        self.log_lock = _thread.allocate_lock()
        
        # 停止フラグを初期化
        self.running = True
        
        # 別スレッドでログ処理開始
        _thread.start_new_thread(self.log_thread, ())

    def create_file(self):
        counter = 0
        while True:
            file_name = f"/sd/{FILE_NAME}{counter}.txt"
            try:
                with open(file_name, "x") as f:
                    print(f"File created: {FILE_NAME}{counter}.txt")
                    if self.ble.is_connected():
                        self.ble.send(f"File created: {FILE_NAME}{counter}.txt")
                    self.file_name = file_name
                    return file_name
            except OSError:
                counter += 1

    def log_thread(self):
        while self.running:
            self.log_lock.acquire()
            if self.log_queue:
                message = self.log_queue.pop(0)
                self.log_lock.release()
                self.write_log(message)
            else:
                self.log_lock.release()
                time.sleep(0.1)
        print("Logger thread stopped.")

    def write_log(self, message):
        print(message)
        self.ble.send(message)
        if self.sd_state:
            try:
                with open(self.file_name, "a") as f:
                    f.write(message + "\n")
            except OSError as e:
                self.sd_state = False
                print(f"SD card write error: {e}")

    def message(self, message):
        self.log_lock.acquire()
        self.log_queue.append(message)
        self.log_lock.release()


class Motor:
    def __init__(self):
        self.AIN1 = AIN1
        self.AIN2 = AIN2
        self.PWMA = PWMA
        self.BIN1 = BIN1
        self.BIN2 = BIN2
        self.PWMB = PWMB
        self.STBY = STBY
        self.OUTA_A = OUTA_A
        self.OUTB_A = OUTB_A
        self.OUTA_B = OUTA_B
        self.OUTB_B = OUTB_B

        # モーター制御用変数の設定
        self.rate_a = 0
        self.rate_b = 0
        self.target_rpm_a = 0
        self.target_rpm_b = 0
        self.pulse_count_a = 0
        self.pulse_count_b = 0
        self.direction_a = 0
        self.direction_b = 0
        self.start_time = time.ticks_ms()
        self.state = 0

        # エンコーダの割り込み設定（OUTA_A と OUTA_B に設定）
        self.OUTA_A.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_B.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)

        # タイマーオブジェクトを生成
        self.timer = Timer()

    def pulse_counter_a(self, pin):
        if self.OUTA_A.value() == self.OUTB_A.value():
            self.direction_a = 1
        else:
            self.direction_a = -1
        self.pulse_count_a += self.direction_a

    def pulse_counter_b(self, pin):
        if self.OUTA_B.value() == self.OUTB_B.value():
            self.direction_b = 1
        else:
            self.direction_b = -1
        self.pulse_count_b += self.direction_b
        
    def run(self, state):
        if self.state != state:
            self.stop()
            self.state = state

        if self.state == FORWARD:
            self.AIN1.off()
            self.AIN2.on()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.on()
            self.BIN2.off()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
            
        elif self.state == TURN_R:
            self.AIN1.on()
            self.AIN2.off()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.on()
            self.BIN2.off()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
            
        elif self.state == TURN_L:
            self.AIN1.off()
            self.AIN2.on()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.off()
            self.BIN2.on()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))
            
        elif self.state == BACKWARD:
            self.AIN1.on()
            self.AIN2.off()
            self.PWMA.duty_u16(int(65535 * self.rate_a / 100))
            self.BIN1.off()
            self.BIN2.on()
            self.PWMB.duty_u16(int(65535 * self.rate_b / 100))

    def stop(self):
        self.AIN1.off()
        self.AIN2.off()
        self.BIN1.off()
        self.BIN2.off()
        self.rate_a = 20
        self.rate_b = 20
        self.state = 0

    def compute_rpm(self, pulse_count, interval):
        if interval <= 0:
            return 0
        rpm = abs((pulse_count * 60) / (PPR * GEAR_RATIO * interval))
        return rpm

    def update_speed(self, timer):
        if self.state == 0:
            self.start_time = time.ticks_ms()
            self.pulse_count_a = 0
            self.pulse_count_b = 0
        else:
            now = time.ticks_ms()
            interval = (now - self.start_time) / 1000
            rpm_a = self.compute_rpm(self.pulse_count_a, interval)
            rpm_b = self.compute_rpm(self.pulse_count_b, interval)
            self.rate_a += KP_RPM * (self.target_rpm_a - rpm_a)
            self.rate_b += KP_RPM * (self.target_rpm_b - rpm_b)
            self.rate_a = min(max(self.rate_a, 0), 100)
            self.rate_b = min(max(self.rate_b, 0), 100)
            
            print(rpm_b, rpm_a)
            
            #self.run(self.state)
            
            self.start_time = now
            self.pulse_count_a = 0
            self.pulse_count_b = 0

    def update_rpm(self, target_rpm_a, target_rpm_b):
        self.target_rpm_a = target_rpm_a
        self.target_rpm_b = target_rpm_b

    def enable_irq(self):
        self.OUTA_A.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_a)
        self.OUTA_B.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_counter_b)
        self.timer.init(mode=Timer.PERIODIC, freq=FREQ, callback=self.update_speed)
        self.start_time = time.ticks_ms()

    def disable_irq(self):
        self.timer.deinit()
        self.OUTA_A.irq(handler=None)
        self.OUTA_B.irq(handler=None) 


if __name__ == '__main__':
    motor = Motor()
    motor.enable_irq()
    motor.update_rpm(30, 30)
    log = Logger(SPI1, SPI1_CS)
    
    try:
        AIN1.off()
        AIN2.on()
        PWMA.duty_u16(int(65535 * 30 / 100))
        BIN1.on()
        BIN2.off()
        PWMB.duty_u16(int(65535 * 30 / 100))
        motor.state = 1
        
        while True:
            log.message("Hello World")
            time.sleep(0.1)

    except KeyboardInterrupt:
        motor.stop()
        motor.disable_irq()
        log.running = False
        log.ble.active(False)
        print("stopped!")
        time.sleep(1)
