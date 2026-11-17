# Status of one HTD-35H bus servo, published by servo_driver_node at ~2 Hz.
std_msgs/Header header
uint8 id              # servo ID on the bus (1-6)
float32 voltage       # supply voltage, V
float32 temperature   # internal temperature, degrees C
float32 load          # load reported by the servo (see servo_driver_node for scaling)
bool responding       # false if the servo missed its last read
