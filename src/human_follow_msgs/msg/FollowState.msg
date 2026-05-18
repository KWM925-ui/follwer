uint8 IDLE=0
uint8 TARGET_ACQUIRED=1
uint8 FOLLOW=2
uint8 SEARCH=3
uint8 LOST=4

std_msgs/Header header
uint8 state
string state_name
string detail
