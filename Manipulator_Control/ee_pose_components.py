
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# This script makes the end-effector go to a specific pose by defining the pose components
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python ee_pose_components.py  # python3 bartender.py if using ROS Noetic'

def main():
    bot = InterbotixManipulatorXS("px150", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.0, roll=0.2, pitch=0.2)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
