Header header

# the trajectory id, starts from "1".
uint32 trajectory_id

# the action command for trajectory server.
uint32 ACTION_ADD           	   =   1
uint32 ACTION_ABORT         	   =   2
uint32 ACTION_WARN_START           =   3
uint32 ACTION_WARN_FINAL           =   4
uint32 ACTION_WARN_IMPOSSIBLE      =   5
uint32 action

# the order of trajectory.
uint32 num_order
uint32 num_segment

# the polynomial coecfficients of the trajectory.
float64 	start_yaw
float64 	final_yaw
float64[] 	coef_x
float64[] 	coef_y
float64[] 	time
float64   	mag_coeff
uint32[]  	order
float64 time_sum

string debug_info
uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8  ADDJUST_ROBOT_STATUS = 4
uint8  ADDJUST_ROBOT_STATUS_COMPLETED = 5
