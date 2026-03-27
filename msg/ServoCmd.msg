# ServoCmd.msg
# Angles in degrees for each of the 4 arm servos
# Channel mapping:
#   0 = base rotation
#   1 = shoulder
#   2 = elbow
#   3 = gripper
float32[4] angles        # target angle in degrees (0.0 - 180.0)
float32[4] speeds        # move speed 0.0-1.0 (1.0 = full speed); ignored if 0.0
