#!/usr/bin/env python 
# coding=utf-8
# V2版本：专注于话题触发，移除键盘控制和低电量自动回充

import rospy
from std_msgs.msg import Bool, Int8, UInt8, Float32, String
from turtlesim.srv import Spawn
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalID
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
import tf, tf2_ros
import time
import json
import math

# 存放充电桩位置的文件位置
json_file='/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/Charger_Position.json'

# 打印带颜色的信息
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

PI=3.1415926535897


class AutoRecharger():
	def __init__(self):
		# 创建节点
		rospy.init_node("auto_recharger_v2") 
		rospy.loginfo(GREEN+'[V2] 自动回充节点启动 - 话题触发 + 低电量自动回充')

		# 机器人状态变量
		self.robot = {
			'Type':'Plus', 
			'BatteryCapacity':5000, 
			'Voltage':25, 
			'Charging':0, 
			'Charging_current':0, 
			'RED':0, 
			'Rotation_Z':0,
			'car_mode':'mini_mec'
		}

		# 状态变量
		self.nav_end_z=0
		self.start_turn = 0
		self.find_redsignal = 0
		self.red_count=0
		self.chargeflag=0
		self.last_time=rospy.Time.now()
		self.lost_red_flag=rospy.Time.now()
		self.charge_complete=0
		self.last_charge_complete=0
		self.json_data=0
		self.nav_end_rc_flag=0

		# 偏移参数
		self.diff_point = 1.2
		self.diff_angle = -15

		# 读取json文件内保存的充电桩位置信息
		try:
			with open(json_file,'r')as fp:
				self.json_data = json.load(fp)
			rospy.loginfo('[V2] 充电桩位置数据加载成功')
		except Exception as e:
			rospy.logerr('[V2] 充电桩位置数据加载失败: %s', str(e))

		# 创建发布者
		self.Charger_Position_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
		self.NavGoal_Cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=5)
		self.Charger_marker_pub = rospy.Publisher('path_point', MarkerArray, queue_size = 100) 
		self.Recharger_Flag_pub = rospy.Publisher("robot_recharge_flag", Int8, queue_size=5)
		self.Cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
		self.Over_point_nav = rospy.Publisher("nav_over", Bool, queue_size=1)

		# 创建订阅者
		self.Voltage_sub = rospy.Subscriber("PowerVoltage", Float32, self.Voltage_callback)
		self.Charging_Flag_sub = rospy.Subscriber("robot_charging_flag", Bool, self.Charging_Flag_callback)
		self.Charging_Current_sub = rospy.Subscriber("robot_charging_current", Float32, self.Charging_Current_callback)
		self.RED_Flag_sub = rospy.Subscriber("robot_red_flag", UInt8, self.RED_Flag_callback)
		self.Charger_Position_Update_sub = rospy.Subscriber("charger_position_update", PoseStamped, self.Position_Update_callback)
		self.Goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.Nav_Result_callback)
		self.Odom_sub = rospy.Subscriber('/odom', Odometry, self.Odom_callback)
		
		# 【新增】订阅自动回充触发话题（来自line_follow节点）
		self.Auto_Recharge_Trigger_sub = rospy.Subscriber('/auto_recharge/trigger', String, self.Auto_Recharge_Trigger_callback)

		# 创建服务调用者
		self.set_charge = rospy.ServiceProxy('/set_charge', Spawn)

		# V2版本说明（仅话题触发）
		rospy.loginfo("[V2] 触发方式: 话题 /auto_recharge/trigger")

	def check_topic(self, topic_name):
		'''检查是否存在对应的话题'''
		topics = rospy.get_published_topics()
		for i in topics:
			if i[0] == topic_name:
				return True
		return False

	def clear_allNav(self):
		'''清除多点导航'''
		topic = Bool()
		topic.data=True
		for i in range(10):
			self.Over_point_nav.publish(topic)

	def set_charge_mode(self, value, max_callcount=10):
		'''设置自动回充的状态'''
		try:
			# 增加超时时间到5秒
			rospy.wait_for_service('/set_charge', timeout=5)
		except rospy.ROSException as e:
			rospy.logerr('[V2] 开启自动回充功能失败: 等待服务超时')
			rospy.logerr('[V2] 提示: 请检查底盘节点是否正常运行')
			return

		state = None
		call_time = 0
		while True:
			try:
				response = self.set_charge(x=value)
				state = response.name
			except rospy.ServiceException as e:
				state = "false"
				rospy.logwarn('[V2] 底盘服务调用失败: %s', str(e))

			if state=="true":
				rospy.loginfo('[V2] 底盘通信成功，回充模式已设置')
				break
			else:
				call_time = call_time + 1
				if call_time > max_callcount:
					rospy.logerr('[V2] 尝试与底盘通信多次失败')
					rospy.logerr('[V2] 提示: 降级使用红外对接模式')
					break
			time.sleep(0.2)  # 增加重试间隔

	def Pub_Charger_Position(self):
		'''使用最新充电桩位置发布导航目标点话题'''
		self.clear_allNav()
		self.nav_end_rc_flag=1

		nav_goal=PoseStamped()
		nav_goal.header.frame_id = 'map'
		nav_goal.header.stamp = rospy.Time.now()
		nav_goal.pose.position.x = self.json_data['p_x']
		nav_goal.pose.position.y = self.json_data['p_y']
		nav_goal.pose.orientation.z = self.json_data['orien_z']
		nav_goal.pose.orientation.w = self.json_data['orien_w']

		self.Charger_Position_pub.publish(nav_goal)
		self.Pub_Charger_marker(
			self.json_data['p_x'], 
			self.json_data['p_y'], 
			self.json_data['orien_z'], 
			self.json_data['orien_w'])

	def Pub_NavGoal_Cancel(self):
		'''取消导航'''
		topic=GoalID()
		self.NavGoal_Cancel_pub.publish(topic)

	def Pub_Charger_marker(self, p_x, p_y, o_z, o_w):
		'''发布目标点可视化话题'''
		# 角度偏移
		tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
		tmp_angle = math.radians(-self.diff_angle)
		new_yaw = (tmp_yaw+tmp_angle)/2
		o_z = math.sin(new_yaw)
		o_w = math.cos(new_yaw)

		tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
		diff_x = math.cos(tmp_yaw)
		diff_y = math.sin(tmp_yaw)
		p_x = p_x - diff_x*self.diff_point
		p_y = p_y - diff_y*self.diff_point

		markerArray = MarkerArray()

		marker_shape = Marker()
		marker_shape.id = 0
		marker_shape.header.frame_id = 'map'
		marker_shape.type = marker_shape.ARROW
		marker_shape.action = marker_shape.ADD
		marker_shape.scale.x = 0.5
		marker_shape.scale.y = 0.05
		marker_shape.scale.z = 0.05
		marker_shape.color.a = 1
		marker_shape.color.r = 1
		marker_shape.color.g = 0
		marker_shape.color.b = 0
		marker_shape.pose.position.x = p_x
		marker_shape.pose.position.y = p_y
		marker_shape.pose.position.z = 0.1
		marker_shape.pose.orientation.z = o_z
		marker_shape.pose.orientation.w = o_w
		markerArray.markers.append(marker_shape)

		marker_string = Marker()
		marker_string.id = 1
		marker_string.header.frame_id = 'map'
		marker_string.type = marker_string.TEXT_VIEW_FACING
		marker_string.action = marker_string.ADD
		marker_string.scale.x = 0.5
		marker_string.scale.y = 0.5
		marker_string.scale.z = 0.5
		marker_string.color.a = 1
		marker_string.color.r = 1
		marker_string.color.g = 0
		marker_string.color.b = 0
		marker_string.pose.position.x = p_x
		marker_string.pose.position.y = p_y
		marker_string.pose.position.z = 0.1
		marker_string.pose.orientation.z = o_z
		marker_string.pose.orientation.w = o_w
		marker_string.text = 'Charger'
		markerArray.markers.append(marker_string)
		self.Charger_marker_pub.publish(markerArray)

	def Pub_Recharger_Flag(self, set_velflag=0):
		'''发布自动回充任务是否开启标志位话题'''
		self.set_charge_mode(self.chargeflag)

		if set_velflag==1:
			topic=Int8()
			topic.data=self.chargeflag
			for i in range(10):
				self.Recharger_Flag_pub.publish(topic)

	def Voltage_callback(self, topic):
		'''更新机器人电池电量'''
		self.robot['Voltage']=topic.data

	def Charging_Flag_callback(self, topic):
		'''更新机器人充电状态'''
		if(self.robot['Charging']==0 and topic.data==1):
			rospy.loginfo("[V2] 充电已开始")
		if(self.robot['Charging']==1 and topic.data==0):
			rospy.logwarn("[V2] 充电已断开")
		self.robot['Charging']=topic.data

	def Charging_Current_callback(self, topic):
		'''更新机器人充电电流数据'''
		self.robot['Charging_current']=topic.data
		
	def RED_Flag_callback(self, topic):
		'''更新是否寻找到红外信号(充电桩)状态'''
		self.red_count = topic.data
		
		if self.robot['Charging']==0:
			if topic.data==0 and self.robot['RED']==1:
				if(rospy.Time.now()-self.lost_red_flag).secs>=2:
					rospy.logwarn("[V2] 红外信号丢失")
				self.lost_red_flag = rospy.Time.now()
	
			if topic.data==1 and self.robot['RED']==0:
				rospy.loginfo("[V2] 发现红外信号")

		if topic.data>0:
			self.robot['RED']=1
		else:
			self.robot['RED']=0

		# 自转寻找红外时处理逻辑
		if self.start_turn==1:
			if self.robot['RED']==1:
				self.find_redsignal = self.find_redsignal + 1 
				if self.find_redsignal>=60:
					self.find_redsignal = 0
					self.start_turn=0
					rospy.loginfo('[V2] 通过自转发现红外信号，开始对接充电')
					vel_topic=Twist()
					self.Cmd_vel_pub.publish(vel_topic)
					self.chargeflag=1
					self.Pub_Recharger_Flag()
					self.nav_end_z = self.robot['Rotation_Z']
			else:
				self.find_redsignal = 0

	def Position_Update_callback(self, topic):
		'''更新json文件中的充电桩位置'''
		position_dic={'p_x':0, 'p_y':0, 'orien_z':0, 'orien_w':0 }
		position_dic['p_x']=topic.pose.position.x
		position_dic['p_y']=topic.pose.position.y
		position_dic['orien_z']=topic.pose.orientation.z
		position_dic['orien_w']=topic.pose.orientation.w

		# 位置偏移
		tmp_yaw = math.atan2(2*(position_dic['orien_w']*position_dic['orien_z']),1-2*(position_dic['orien_z']**2))
		diff_x = math.cos(tmp_yaw)
		diff_y = math.sin(tmp_yaw)
		position_dic['p_x'] = position_dic['p_x'] + diff_x*self.diff_point
		position_dic['p_y'] = position_dic['p_y'] + diff_y*self.diff_point

		# 角度偏移
		tmp_angle = math.radians(self.diff_angle)
		new_yaw = (tmp_yaw+tmp_angle)/2
		position_dic['orien_z'] = math.sin(new_yaw)
		position_dic['orien_w'] = math.cos(new_yaw)

		# 保存到json文件
		with open(json_file, 'w') as fp:
			json.dump(position_dic, fp, ensure_ascii=False)
			rospy.loginfo("[V2] 充电桩位置已更新")
		
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)
		
		self.Pub_Charger_marker(position_dic['p_x'], position_dic['p_y'], position_dic['orien_z'], position_dic['orien_w'])

	def Nav_Result_callback(self, topic):
		'''导航结果订阅函数'''
		if 'canceled' in topic.status.text:
			return

		if self.nav_end_rc_flag == 1:
			self.nav_end_rc_flag = 0
			
			if 'Failed' in topic.status.text:
				rospy.logerr('[V2] 无法导航到充电桩位置')
				return
				
			if 'Aborting' in topic.status.text:
				rospy.logerr('[V2] 充电桩位置数据异常')
				return
				
			if 'oscillating' in topic.status.text:
				rospy.logwarn('[V2] 警告：请清除充电桩附近障碍物')
				return

			# 导航到达充电桩处
			if self.robot['RED']==1:
				rospy.loginfo('[V2] 已到达充电桩位置，开始对接充电')
				self.chargeflag=1
				self.Pub_Recharger_Flag()
			else:
				self.lost_power_once=1
				if 'akm' in self.robot['car_mode']:
					rospy.logwarn('[V2] 未找到充电桩')	
				else:
					rospy.logwarn('[V2] 未找到充电桩，开始自转寻找红外信号')
					self.nav_end_z = self.robot['Rotation_Z']
					time.sleep(1)
					topic=Twist()
					topic.angular.z = 0.2
					self.Cmd_vel_pub.publish(topic) 
					self.start_turn = 1

	def Odom_callback(self, topic):
		'''更新的机器人实时位姿'''
		self.robot['Rotation_Z']=topic.pose.pose.position.z	 

	def Stop_Charge(self):
		'''停止回充'''
		self.Pub_NavGoal_Cancel() 
		self.nav_end_rc_flag=0
		self.lost_power_once=1
		
		self.chargeflag=0
		self.Pub_Recharger_Flag()
		
		topic=Twist()
		self.Cmd_vel_pub.publish(topic)
		
		if self.robot['Charging']==1:
			topic=Twist()
			topic.linear.x = 0.1
			self.Cmd_vel_pub.publish(topic)

			if 'mini' in self.robot['car_mode'] or 'akm' in self.robot['car_mode']: 
				time.sleep(3)
			else:
				time.sleep(1)
			topic.linear.x = 0.0
			self.Cmd_vel_pub.publish(topic)

	def Auto_Recharge_Trigger_callback(self, msg):
		'''话题触发回调 - 接收来自line_follow的触发指令'''
		command = msg.data.lower()
		
		if command == "start" or command == "green_detected":
			rospy.loginfo('[V2] 触发回充: '+command)
			self.trigger_recharge()
		elif command == "stop":
			rospy.loginfo('[V2] 停止回充')
			self.Stop_Charge()

	def trigger_recharge(self):
		'''触发回充流程'''
		# 存在3路以上的红外信号，直接对接
		if self.red_count>=2:
			self.Pub_NavGoal_Cancel()
			self.clear_allNav()
			self.chargeflag=1
			self.Pub_Recharger_Flag()
			rospy.loginfo('[V2] 检测到高强度红外信号，直接对接')
		else:
			# 检测是否存在导航节点
			if self.check_topic('/move_base/goal')==True:
				self.Pub_NavGoal_Cancel()
				self.Pub_Charger_Position() 
				rospy.loginfo('[V2] 开始导航到充电桩位置')
			else:
				if self.robot['RED']==1:
					self.chargeflag=1
					self.Pub_Recharger_Flag()
					rospy.loginfo('[V2] 未检测到导航节点，使用红外信号对接')
				else:
					rospy.logerr('[V2] 未检测到导航节点和红外信号，无法启动回充')

	def autoRecharger(self):
		'''自动回充逻辑（仅话题触发，移除低电量自动触发）'''
		
		# 检测充电完成
		if self.robot['Charging']==1:
			if (self.robot['Type']=='Plus' and self.robot['Voltage']>25) or (self.robot['Type']=='Mini' and self.robot['Voltage']>12.5):
				self.charge_complete=self.charge_complete+1
			else:
				self.charge_complete=0	

		# 1Hz循环任务
		if (rospy.Time.now()-self.last_time).secs>=1:
			self.Pub_Charger_marker(
				self.json_data['p_x'], 
				self.json_data['p_y'], 
				self.json_data['orien_z'], 
				self.json_data['orien_w'])
			self.last_time=rospy.Time.now()


			# 自转寻找充电桩
			if self.start_turn == 1:
				if abs(self.robot['Rotation_Z']-self.nav_end_z)>2*PI:
					self.start_turn=0
					self.Stop_Charge()
					rospy.logerr('[V2] 自转完成，无法找到充电桩')

		# 充电完成判断
		if self.charge_complete>10:
			self.charge_complete=0
			if self.last_charge_complete!=0:
				self.last_charge_complete=0
				self.Stop_Charge()			
			rospy.loginfo('[V2] 充电已完成')
		self.last_charge_complete=self.charge_complete

if __name__ == '__main__':
	try:
		AutoRecharger=AutoRecharger()

		# 从参数服务器获取参数
		AutoRecharger.robot['BatteryCapacity'] = rospy.get_param("/robot_BatteryCapacity", default="5000")
		AutoRecharger.robot['car_mode'] = rospy.get_param("/wheeltec_robot/car_mode", default='mini_mec')
		AutoRecharger.diff_point = rospy.get_param("~diff_point", default='1.2')
		AutoRecharger.diff_angle = rospy.get_param("~diff_angle", default='-15')

		if AutoRecharger.robot['car_mode'][0:4]!='mini':
			AutoRecharger.robot['Type'] = 'Plus'
		else:
			AutoRecharger.robot['Type'] = 'Mini'
		
		time.sleep(2)
		rospy.loginfo('[V2] 节点初始化完成，等待触发指令...')
		rospy.loginfo('[V2] 触发方式: 话题(/auto_recharge/trigger) + 低电量自动')
		
		# 主循环 - 使用ROS Rate控制频率（移除键盘控制）
		rate = rospy.Rate(10)  # 10Hz
		while not rospy.is_shutdown():
			AutoRecharger.autoRecharger()
			rate.sleep()
			
	except rospy.ROSInterruptException:
		rospy.loginfo('[V2] 节点正常退出')
	except Exception as e:
		rospy.logerr('[V2] 节点异常: %s', str(e))
	finally:
		topic=Twist()
		AutoRecharger.Cmd_vel_pub.publish(topic)
		AutoRecharger.chargeflag=0
		AutoRecharger.Pub_Recharger_Flag(1)
		rospy.loginfo('[V2] 节点已停止')
