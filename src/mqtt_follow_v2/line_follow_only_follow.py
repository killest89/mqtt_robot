#!/usr/bin/env python
# coding=utf-8
import threading

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

current_temperature = "N/A"


def nothing(s):
    pass


# 颜色阈值定义
col_black = (0, 0, 0, 180, 255, 46)  # black
col_red = (0, 100, 80, 10, 255, 255)  # red
col_blue = (90, 90, 90, 110, 255, 255)  # blue
col_green = (65, 70, 70, 85, 255, 255)  # green
col_yellow = (26, 43, 46, 34, 255, 255)  # yellow

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


def get_current_temperature():
    """获取当前温度值"""
    global current_temperature
    return current_temperature


def execute_relay_sequence():
    # ========== 在这里添加温度获取和发送 ==========
    """执行复杂的继电器控制序列"""
    rospy.loginfo("[RELAY] Starting relay sequence")
    # 触发继电器序列
    trigger_msg = String()
    trigger_msg.data = "trigger"
    rospy.loginfo("[Line Follower] 准备发布继电器触发消息: %s", trigger_msg.data)
    # self.relay_trigger_pub.publish(trigger_msg)

    # 重置继电器序列完成标志
    rospy.loginfo("[RELAY] Relay sequence completed")

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
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
        self.yellow_detected = False
        self.last_erro = 0
        self.sequence_thread = None

        # 控制命令订阅
        self.control_sub = rospy.Subscriber('/line_follower/control', String, self.control_callback)

        # 继电器序列触发发布
        self.relay_trigger_pub = rospy.Publisher('/line_follower/relay_trigger', String, queue_size=10)

        self.state = "WAITING"

    def control_callback(self, msg):
        """处理控制命令"""
        command = msg.data.lower()
        rospy.loginfo("[Line Follower] 收到控制命令: %s", command)

        if command == "start" and (self.state == "WAITING" or self.state == "RECHARGING"):
            # 如果正在回充，通知回充节点停止
            # if self.state == "RECHARGING" and self.recharge_triggered:
            #     self.stop_recharge()

            self.state = "FOLLOWING"
            self.follow_start_time = time.time()
            # 重置检测标志
            self.red_detected = False
            rospy.loginfo("[Line Follower] 开始寻线任务")

        elif command == "stop":
            # 如果正在回充，通知回充节点停止
            # if self.state == "RECHARGING" and self.recharge_triggered:
            #     self.stop_recharge()

            self.state = "WAITING"
            self.waiting_start_time = time.time()

            # 停止机器人运动
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

            # 显示静态的WAITING状态窗口
            # self.show_waiting_status()
            rospy.loginfo("[Line Follower] 停止寻线任务")

        elif command == "relay_sequence_finish":
            rospy.loginfo("[Line Follower] 收到继电器序列完成通知，准备切换到AVOIDING状态")
            # self.relay_sequence_completed = True

    def detect_red_segment(self, hsv_img):
        """检测红色线段（保持原功能不变）"""
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

    def detect_yellow_segment(self, hsv_img):
        """检测黄色线段（用于停机触发）"""
        yellow_mask = cv2.inRange(hsv_img, (col_yellow[0], col_yellow[1], col_yellow[2]),
                                  (col_yellow[3], col_yellow[4], col_yellow[5]))
        h, w = yellow_mask.shape
        roi = yellow_mask[3 * h // 4:h, :]
        yellow_pixels = cv2.countNonZero(roi)
        total_pixels = roi.size
        ratio = float(yellow_pixels) / total_pixels

        if ratio > 0.05 and not self.yellow_detected:
            self.yellow_detected = True
            return True
        elif ratio < 0.01:
            self.yellow_detected = False
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

        if self.state == "WAITING" or self.state == "RECHARGING":
            return

        mask = np.zeros((240, 320), dtype=np.uint8)

        if self.temp == 0:
            cv2.namedWindow('Adjust_hsv', cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch, 'Adjust_hsv', 4, 4, nothing)
            self.temp = 1

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ================== 状态机处理 ==================
        if self.state == "FOLLOWING":
            if self.detect_yellow_segment(hsv):
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
            kernel = np.ones((5, 5), np.uint8)
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
        cv2.rectangle(image, (5, 5), (250, 100), color, -1)  # 增加矩形高度以容纳更多信息
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
        # 添加温度显示
        cv2.putText(image, "Temp: {}".format(get_current_temperature()), (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # ================== 显示窗口 ==================
        cv2.imshow("window", image)
        cv2.imshow("Adjust_hsv", mask)
        cv2.waitKey(3)

    def cleanup(self):
        """清理资源"""
        try:
            rospy.loginfo("[Line Follower] 开始清理资源...")
            rospy.loginfo("[Line Follower] 资源清理完成")
        except Exception as e:
            rospy.logerr("[Line Follower] 资源清理错误: %s", str(e))


if __name__ == '__main__':
    rospy.init_node("line_follower")
    time.sleep(0.5)
    follower = LineFollower()

    # 注册关闭钩子
    rospy.on_shutdown(follower.cleanup)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[Line Follower] 节点被用户中断")
    finally:
        follower.cleanup()
