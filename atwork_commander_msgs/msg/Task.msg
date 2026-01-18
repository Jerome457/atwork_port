# Task.msg

# Standard header
std_msgs/Header header

# Robots that execute this task
atwork_commander_msgs/RobotHeader[] execute_on

# Arena states
atwork_commander_msgs/Workstation[] arena_start_state
atwork_commander_msgs/Workstation[] arena_target_state

# Task timing
builtin_interfaces/Duration prep_time
builtin_interfaces/Duration exec_time

# Task identification
string type
uint64 id
string config
uint64 commit
