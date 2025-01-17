
import time
import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.5.3'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot

mc = MyCobot('/dev/ttyACM0', 115200)

angle_max = []
angle_min = []

for i in range(1,7):
    joint_max = mc.get_joint_max_angle(i)
    angle_max.append(joint_max)
    time.sleep(0.05)
    joint_min = mc.get_joint_min_angle(i)
    angle_min.append(joint_min)
    
print('最大关节限位:',angle_max)
print('最大关节限位:',angle_min)