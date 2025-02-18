import math
import time
import random

# パラメータ
L1 = 59.717
L2 = 113.25
L3 = 86
h = 98
x_target = 150
y_target = 1

def F(theta1, theta2, theta3):
    theta1_rad, theta2_rad, theta3_rad = map(math.radians, [theta1, theta2, theta3])
    
    f1 = (L1 * math.cos(theta1_rad) + L2 * math.cos(theta2_rad) + L3 * math.cos(theta3_rad) - x_target)
    f2 = (L1 * math.sin(theta1_rad) + L2 * math.sin(theta2_rad) + L3 * math.sin(theta3_rad) + h - y_target)
    return [f1, f2]

def theta_solver():
    max_attempts = 180**3
    start_total = time.ticks_ms()
    
    for _ in range(max_attempts):
        start = time.ticks_ms()
        theta1 = random.randint(0, 180)
        theta2 = random.randint(theta1 - 180, theta1)

        min_val = max(theta2 - 180, -60)
        max_val = min(theta2, 0)
        if min_val > max_val:
            continue

        theta3 = random.randint(min_val, max_val)
        error_x, error_y = F(theta1, theta2, theta3)
        if abs(error_x) <= 10 and abs(error_y) <= 10:
            print(f"解: θ1 = {theta1}, θ2 = {theta2}, θ3 = {theta3}")
            print(f"  誤差: Δx = {error_x:.6f}, Δy = {error_y:.6f}")
            print("-" * 40)
            break
        
        ela_time = (time.ticks_ms() - start) / 1000
        print(f"試行 {_:d} - 経過時間: {ela_time:.4f} 秒")

theta_solver()
