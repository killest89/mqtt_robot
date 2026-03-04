#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy
import time
from geometry_msgs.msg import Twist
import serial
import threading

# 定义所有继电器控制指令
RELAY_COMMANDS = {
    # 开启指令
    "1_ON": b'\xA0\x01\x01\xA2',
    "1_OFF": b'\xA0\x01\x00\xA1',
    "2_ON": b'\xA0\x02\x01\xA3',
    "2_OFF": b'\xA0\x02\x00\xA2',
    "3_ON": b'\xA0\x03\x01\xA4',
    "3_OFF": b'\xA0\x03\x00\xA3',
    "4_ON": b'\xA0\x04\x01\xA5',
    "4_OFF": b'\xA0\x04\x00\xA4',
    "8_ON": b'\xA0\x08\x01\xA9',
    "8_OFF": b'\xA0\x08\x00\xA8'
}

# 全局串口对象和锁
serial_port = None
serial_lock = threading.Lock()

def nothing(s):
    pass

# 颜色阈值定义
col_black = (0, 0, 0, 180, 255, 46)  # black
col_red = (0, 100, 80, 10, 255, 255)  # red
col_blue = (90, 90, 90, 110, 255, 255)  # blue
col_green = (65, 70, 70, 85, 255, 255)  # green
col_yellow = (26, 43, 46, 34, 255, 255)  # yellow

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'

def init_serial_port():
    """初始化串口连接"""
    global serial_port
    try:
        serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)
        rospy.loginfo("[SERIAL] Serial port initialized successfully")
    except serial.SerialException as e:
        rospy.logerr("[SERIAL ERROR] Could not open serial port: %s" % e)
        serial_port = None

def send_relay_command(command):
    """发送单个继电器控制指令"""
    global serial_port, serial_lock

    if serial_port is None:
        rospy.logwarn("[SERIAL] Serial port not initialized, skipping command")
        return

    try:
        with serial_lock:
            if not serial_port.isOpen():
                rospy.logwarn("[SERIAL] Port closed, trying to reopen")
                serial_port.open()
                
            if serial_port.isOpen():
                serial_port.write(command)
                rospy.loginfo("[SERIAL] Command sent: %s" % command)
    except Exception as e:
        rospy.logerr("[SERIAL ERROR] Failed to send command: %s" % e)

def execute_relay_sequence():
    """执行复杂的继电器控制序列"""
    rospy.loginfo("[RELAY] Starting relay sequence")
    
    # 第一路开启
    send_relay_command(RELAY_COMMANDS["1_ON"])
    time.sleep(10)
    
    # 第一路关闭及第三路开启
    send_relay_command(RELAY_COMMANDS["1_OFF"])
    time.sleep(1)
    send_relay_command(RELAY_COMMANDS["3_ON"])
    time.sleep(3)
    
    # 第三路关闭
    send_relay_command(RELAY_COMMANDS["3_OFF"])
    time.sleep(1)
    
    # 第三路开启
    send_relay_command(RELAY_COMMANDS["3_ON"])
    time.sleep(1)
    
    # 第三路关闭
    send_relay_command(RELAY_COMMANDS["3_OFF"])
    time.sleep(2)
    
    # 第四路开启
    send_relay_command(RELAY_COMMANDS["4_ON"])
    time.sleep(1)
    
    # 第四路关闭再开启
    send_relay_command(RELAY_COMMANDS["4_OFF"])
    time.sleep(1)
    send_relay_command(RELAY_COMMANDS["4_ON"])
    time.sleep(1)
    
    # 第四路关闭再开启
    send_relay_command(RELAY_COMMANDS["4_OFF"])
    time.sleep(1)
    send_relay_command(RELAY_COMMANDS["4_ON"])
    time.sleep(1)
    
    # 第四路关闭再开启
    send_relay_command(RELAY_COMMANDS["4_OFF"])
    time.sleep(1)
    send_relay_command(RELAY_COMMANDS["4_ON"])
    time.sleep(1)
    
    # 第四路关闭及第三路开启
    send_relay_command(RELAY_COMMANDS["4_OFF"])
    time.sleep(1)
    send_relay_command(RELAY_COMMANDS["3_ON"])
    time.sleep(2)
    
    # 第三路关闭再开启
    send_relay_command(RELAY_COMMANDS["3_OFF"])
    time.sleep(2)
    send_relay_command(RELAY_COMMANDS["3_ON"])
    time.sleep(2)

    # 第三路关闭后再开启后关闭
    send_relay_command(RELAY_COMMANDS["3_OFF"])
    time.sleep(1)
    
    # 第二路开启
    send_relay_command(RELAY_COMMANDS["2_ON"])
    time.sleep(10)
    
    # 第二路关闭
    send_relay_command(RELAY_COMMANDS["2_OFF"])
    rospy.loginfo("[RELAY] Relay sequence completed")

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.use_usb_cam = rospy.get_param("/use_usb_cam", 'no')
        if self.use_usb_cam == 'yes':
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        else:
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_ori", Twist, queue_size=1)
        self.twist = Twist()
        self.temp = 0

        # 状态机初始化
        self.state = "INIT"  # INIT, FOLLOWING, STOPPED, AVOIDING
        self.stop_start_time = 0
        self.avoid_start_time = 0
        self.follow_start_time = time.time()
        self.avoid_duration = 2.0
        self.red_detected = False
        self.last_erro = 0
        self.sequence_thread = None

        # 初始发送第八路开启指令
        send_relay_command(RELAY_COMMANDS["8_ON"])
        self.state = "FOLLOWING"

    def detect_red_segment(self, hsv_img):
        """检测红色线段"""
        red_mask = cv2.inRange(hsv_img, (col_red[0], col_red[1], col_red[2]),
                               (col_red[3], col_red[4], col_red[5]))
        h, w = red_mask.shape
        roi = red_mask[3 * h // 4:h, :]
        red_pixels = cv2.countNonZero(roi)
        total_pixels = roi.size
        ratio = float(red_pixels) / total_pixels

        if ratio > 0.05 and not self.red_detected:
            self.red_detected = True
            return True
        elif ratio < 0.01:
            self.red_detected = False
        return False

    def avoid_red_segment(self):
        """绕过红色线段的策略"""
        elapsed = time.time() - self.avoid_start_time

        if elapsed < self.avoid_duration * 0.3:
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.5
            return False
        elif elapsed < self.avoid_duration * 0.7:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            return False
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.3
            return elapsed >= self.avoid_duration

    def image_callback(self, msg):
        mask = numpy.zeros((240, 320), dtype=numpy.uint8)

        if self.temp == 0:
            cv2.namedWindow('Adjust_hsv', cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch, 'Adjust_hsv', 4, 4, nothing)
            self.temp = 1

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ================== 状态机处理 ==================
        if self.state == "FOLLOWING":
            if self.detect_red_segment(hsv):
                self.state = "STOPPED"
                self.stop_start_time = time.time()
                rospy.loginfo("[STATE] Switching to STOPPED state")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                
                # 启动继电器控制序列线程
                if self.sequence_thread is None or not self.sequence_thread.is_alive():
                    self.sequence_thread = threading.Thread(target=execute_relay_sequence)
                    self.sequence_thread.start()

        elif self.state == "STOPPED":
            if time.time() - self.stop_start_time > 10.0 and (self.sequence_thread is None or not self.sequence_thread.is_alive()):
                self.state = "AVOIDING"
                self.avoid_start_time = time.time()
                self.last_erro = 0
                rospy.loginfo("[STATE] Switching to AVOIDING state")

        elif self.state == "AVOIDING":
            if self.avoid_red_segment():
                self.state = "FOLLOWING"
                self.red_detected = False
                self.follow_start_time = time.time()
                rospy.loginfo("[STATE] Switching back to FOLLOWING state")

        # ================== 黑线跟踪 ==================
        if self.state == "FOLLOWING":
            kernel = numpy.ones((5, 5), numpy.uint8)
            hsv_erode = cv2.erode(hsv, kernel, iterations=1)
            hsv_dilate = cv2.dilate(hsv_erode, kernel, iterations=1)
            mask = cv2.inRange(hsv_dilate, (col_black[0], col_black[1], col_black[2]),
                               (col_black[3], col_black[4], col_black[5]))
            h, w, d = image.shape
            search_top = h - 20
            search_bot = h
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
                erro = cx - w / 2 - 15
                d_erro = erro - self.last_erro
                self.twist.linear.x = 0.15
                if erro != 0:
                    self.twist.angular.z = -float(erro) * 0.005 - float(d_erro) * 0.000
                else:
                    self.twist.angular.z = 0
                self.last_erro = erro
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0.2

        # ================== 发布速度命令 ==================
        self.cmd_vel_pub.publish(self.twist)

        # ================== 可视化增强 ==================
        state_colors = {
            "INIT": (200, 200, 200),
            "FOLLOWING": (0, 255, 0),
            "STOPPED": (0, 0, 255),
            "AVOIDING": (0, 255, 255)
        }
        color = state_colors.get(self.state, (255, 255, 255))
        cv2.rectangle(image, (5, 5), (250, 60), color, -1)
        state_duration = time.time() - {
            "FOLLOWING": self.follow_start_time,
            "STOPPED": self.stop_start_time,
            "AVOIDING": self.avoid_start_time
        }.get(self.state, 0)

        cv2.putText(image, "State: {}".format(self.state), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "RedDetected: {}".format(self.red_detected), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "Duration: {:.1f}s".format(state_duration), (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # ================== 显示窗口 ==================
        cv2.imshow("window", image)
        cv2.imshow("Adjust_hsv", mask)
        cv2.waitKey(3)

rospy.init_node("opencv")
init_serial_port()
follower = Follower()
rospy.spin()

# 程序退出时关闭所有继电器
if serial_port is not None and serial_port.isOpen():
    rospy.loginfo("[SERIAL] Closing serial port and turning off all relays")
    send_relay_command(RELAY_COMMANDS["8_OFF"])
    time.sleep(0.5)
    serial_port.close()
