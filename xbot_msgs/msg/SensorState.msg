# Xbot Sensor Data Messages
#
# For more direct simple interactions (buttons, leds, gyro, motor velocity
# etc) use the other topics. This provides detailed information about the
# entire state package that is transmitted at 50Hz from the robot.
#





# Over current states
uint8 OVER_CURRENT_1_WHEEL  = 1
uint8 OVER_CURRENT_2_WHEEL = 2
uint8 OVER_CURRENT_3_WHEEL  = 3
uint8 OVER_CURRENT_4_WHEEL  = 4
uint8 OVER_CURRENT_BOTH_WHEELS = 5



###### MESSAGE ######

Header header

###################
# Core Packet
###################
uint32 time_stamp      # milliseconds starting when turning on Xbot (max. 65536, then starts from 0 again)
#uint16 left_encoder    # accumulated ticks left wheel starting with turning on Xbot (max. 65535)
#uint16 right_encoder   # accumulated ticks right wheel starting with turning on Xbot (max. 65535)
uint16 encoder1
uint16 encoder2
uint16 encoder3
uint16 encoder4
uint8  charger         # see charger states
uint8  battery         # battery voltage in 0.1V (ex. 16.1V -> 161)


###################
# Current Packet
###################
uint8[] current        # motor current for the left and right motor in 10mA (ex. 12 -> 120mA)
uint8   over_current   # see over current states

