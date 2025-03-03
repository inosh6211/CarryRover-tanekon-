def apriltag_guidance(index):
     """エイプリルタグでの誘導"""
    #ここから90度回転して正面に来るプログラム
    KP_YAW=0.3#ここで比例定数を決める
    #motor = Motor()
    Motor.enable_irq() 

    m=160#ここでapriltagの何ｍｍ前に来るかを決める


    if __name__ == "__main__":
        #camera = CameraReceiver(UART1)
        time.sleep(3)
        
        
        tag_id = 0    # ID
        cx = 0      # X座標
        cy = 0      # Y座標
        distance = 0  # 距離
        pitch = 0     # ピッチ角
        yaw = 0       # ヨー角
    
           
        Motor.enable_irq()
        Motor.update_rpm(30, 30)
        
        
        
        print("go")
        
    try:
        #motor = Motor()
        motor.enable_irq()    
        #cam = CameraReceiver(UART1)
        
        while True:
            CameraReceiver(UART1).read_tags(0)
            print(cam.tag_detected[6])
            time.sleep(0.1)
            
            
            
            
            if cam.tag_detected[0]:
                print("0")
                roll, pitch, yaw = compute_euler()
                init_yaw = yaw
                #forward_to_Apriltag_mm(m)
                print(f"complete")
               
            elif cam.tag_detected[1]:
                print(f"1")
                roll, pitch, yaw = compute_euler()
                init_yaw = yaw
                soutai_turn_right_90()
                straight_forward_t(2)
                soutai_turn_left_from358to2()
                straight_forward_t(2)
                soutai_turn_left_90()
                print("ji")
                print(f"complete")
                 
            elif cam.tag_detected[2]:#ここで特異点を超えてしまうやつが発生する危険性あり
                print(f"2")
                roll, pitch, yaw = compute_euler()
                init_yaw = yaw
                soutai_turn_right_90()
                straight_forward_t(2)
                soutai_turn_left_from358to2()
                straight_forward_t(4)
                soutai_turn_left_90()
                straight_forward_t(2)
                soutai_turn_left_180()
                print(f"complete")
                
            elif cam.tag_detected[3]:
                print(f"3")
                roll, pitch, yaw = compute_euler()
                init_yaw = yaw;
                soutai_turn_left_90()
                straight_forward_t(2)
                soutai_turn_right_from358to2()
                straight_forward_t(2)
                soutai_turn_right_90()
                print(f"complete")
        #ここまで90度回転して正面に来るプログラム
