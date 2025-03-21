
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

# This script commands currents [mA] to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx250'
# Then change to this directory and type 'python joint_current_control.py  # python3 bartender.py if using ROS Noetic'

def main():
    bot = InterbotixManipulatorXS(robot_model="px150", group_name="arm", gripper_name="gripper")

    joint_pwms = [0, 100 , 100, 25, 0]
    bot = InterbotixManipulatorXS("px150", "arm", "gripper")
    bot.dxl.robot_set_operating_modes("group", "arm", "pwm")
    bot.dxl.robot_write_commands("arm", joint_pwms)

if __name__=='__main__':
    main()
