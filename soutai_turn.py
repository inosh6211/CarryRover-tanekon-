    def soutai_turn(self, angle, init_yaw):#diffは右回り正の,init_yawからの角度の差を示し、angleはその中のdiffの角度をさし、そこに向かって回転する
        while True:                    #angleは右回り正で０から360
            self.update_rpm(10,10)
            bno.reset()
            bno.compute_euler()
            current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
            diff = ((current_yaw - init_yaw + 360) % 360)#((x - y + 360) % 360)はx,yが右回り正、0から360の時ｙをきじゅんとしてｘと角度差の角度差を0から360に変換する
            if ((angle - diff + 360) % 360) <= 180:#angleはたどり着きたい角度のinit_yawから右回り正のやつ
                while True:
                    print(diff)
                    bno.computer_euler()
                    current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
                    diff = ((current_yaw - init_yaw + 360) % 360)
                    motor.run(TURN_R)
                    if angle-diff < -1:
                        motor.run(TURN_R)
                    if abs(angle-diff) <= 1:
                        self.stop()
                        break
                    time.sleep(0.01)
            elif ((angle - diff + 360) % 360) > 180:
                while True:
                    print(diff)
                    bno.compute_euler()
                    current_yaw = (-bno.yaw + 360) % 360#右回り正にしたいなら(bno.yaw + 360)
                    diff = ((current_yaw - init_yaw + 360) % 360)
                    motor.run(TURN_L)
                    if angle-diff > 1:
                        motor.run(TURN_L)
                    if abs(angle-diff) <= 1:
                        self.stop()
                        break
                    time.sleep(0.01)
                    
            print(diff)
            if abs(angle-diff) <= 1:#358to2とかで359とかでとまったときにdiff >= 90認定されないように
                print("stop")
                break
            time.sleep(0.01)
