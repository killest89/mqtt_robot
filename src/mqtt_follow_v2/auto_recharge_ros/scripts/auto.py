#!/usr/bin/env python 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作
#2:Python.源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码
import roslaunch
import rospy
import time

rospy.init_node('launch_looper', anonymous=True)



#回充
#roslaunch turn_on_wheeltec_robot navigation.launch （小车端执行）
#roslaunch auto_recharge_ros auto_recharger_node_v1.launch（小车端执行）
navigation_launch = roslaunch.parent.ROSLaunchParent(
    roslaunch.rlutil.get_or_generate_uuid(None, False),
    ["/home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/launch/navigation.launch"]
)
# 启动
navigation_launch.start()
time.sleep(10)  

auto_recharger_node_launch = roslaunch.parent.ROSLaunchParent(
    roslaunch.rlutil.get_or_generate_uuid(None, False),
    ["/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/launch/auto_recharger_node_v1.launch"]
)    

# 启动
auto_recharger_node_launch.start()

# 运行持续时间（按需调整）
time.sleep(4000)  

# 安全关闭
navigation_launch.shutdown()
print ("test")
auto_recharger_node_launch.shutdown()
# 间隔时间
time.sleep(2)
print ("test1")

    


