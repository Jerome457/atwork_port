# RobotState.msg

# The robot sending this state
atwork_commander_msgs/RobotHeader sender

# Robot pose
geometry_msgs/Pose2D pose

# Robot path as an array of stamped poses
geometry_msgs/PoseStamped[] path

# Objects on the robot (fixed-size array of 3 elements)
atwork_commander_msgs/Object[3] objects_on_robot

# Current arena state
atwork_commander_msgs/Workstation[] current_arena_state

# Future extensions
# TODO: datatype for barriertape
# TODO: datatype for obstacles
