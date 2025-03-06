def straight_ward(self, ward, distance, rpm):
    start=time.ticks_ms()
    t = distance / (135 * math.pi * (rpm /60))
    bno.compute_euler()
    init_yaw = (-bno.yaw + 360) % 360
            
    if ward == f
        while True:
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360
            diff = ((current_yaw - init_yaw + 540) % 360) - 180
            rate_a=-KP_YAW*diff+rpm
            rate_b=KP_YAW*diff+rpm
            self.update_rpm(rate_a, rate_b)
            self.run(FORWARD)
            print(f"L{diff}")
            now = time.ticks_ms()
            if (time.ticks_ms() - start) / 1000 >= t:
                motor.stop()
                break
            time.sleep(0.01)
            
    elif ward == b
        while True:
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360
            diff = ((current_yaw - init_yaw + 540) % 360) - 180
            rate_a=KP_YAW*diff+rpm
            rate_b=-KP_YAW*diff+rpm
            self.update_rpm(rate_a, rate_b)
            self.run(BACKWARD)
            print(f"L{diff}")
            now = time.ticks_ms()
            if (time.ticks_ms() - start) / 1000 >= t:
                motor.stop()
                break
            time.sleep(0.01)

