# 
std_msgs/Header header
int32 error_type	# Definition of error type.
                    # 1000~: Common Error type. 
                    # 1000: Clear
                    # 1001: Serial Open Error, 1002: Serial Read Error, 1003: Serial Data Loss
                    # 1101: Ethernet Open Error, 1102: Ethernet Read Error, 1103: Ethernet Data Loss
                    # 1201: CAN Open Error, 1202: CAN Read Error, 1203: CAN Write Error
                    
                    # 0~999: Each Node has speical error type. ->
                    ## 0~49:IMU 
                    ## 50~99:LiDAR 
                    ## 100~149:BAT 
                    # 100: Battery Need to Charge, 101: Battery Over Voltage, 102: Battery Charge Over Current
                    # 103: Battery Charge Over Temperature
                    # 110: Battery BMS Version Incorrect
                    ## 150~199:Motor
                    # 150: Emg Stop, 151: Emg Release, 152: Safety Stop
                    # 160: Motor fault Flag 
                    # 170: RunTime Status fault flag(Right)
                    # 180: RunTime Status fault flag(Left)
                    # 190: Status Flag
                    ## 200~249:GPIO(EMG) 
                    ## 250~299:STM32 
                    ## 300~349:STATE
                    ## 350~399:VISION
                    # 360:VISION_CLIFF ???
                    # 365:VISION_CLIFF_OK
                    # 366:VISION_CLIFF_NG
int32 error_code	# Error code line indication number.
                    # 0~49:IMU / 50~99:LiDAR / 100~149:BAT / 150~199:Motor / 200~249:GPIO(EMG) / 250~299:STM32 / 300~349:STATE / 350~499:VISION
                    # 0: IMU Open OK
                    # 365:VISION_CLIFF_OK
                    # 366:VISION_CLIFF_NG
string error_msg	# "Error message description"
string source_node	# IMU, LIDAR, BAT, MOTOR, BAT, GPIO(EMG), STM32, VISION_CLIFF, VISION_CLIFF_1DLIDAR
string level		# WARN, ERROR, INFO