if __name__ == "__main__":
    log = Logger(SPI1, SPI1_CS)
    bno = BNO055Handler(I2C0)
    bme = BME280(I2C0)
    gps = GPS(UART0)
    cam = CameraReceiver(UART1)
    arm = ArmController(I2C1)
    
    
    log.sd_write("Setup completed")
    
    time.sleep(2)
    
    try:
        start()
        released()
        landing()
        fusing()
        gps_guidance(0)
        color_guidance(0)
        apriltag_alignment(0)
        apriltag_guidance(0)
        collect_material(0)
        gps_guidance(1)
        color_guidance(1)
        apriltag_alignment(1)
        apriltag_guidance(1)
        arm.place_object()
        color_guidance(0)
        gps_guidance(0)
        color_guidance(0)
        apriltag_alignment(0)
        apriltag_guidance(0)
        arm.place_object()
        
        
        log.sd_write("Mission completed!")
        
    finally:
        stop()
        time.sleep(1)
