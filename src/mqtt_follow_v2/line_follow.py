#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

# 颜色阈值定义
col_black = (0, 0, 0, 180, 255, 46)  # black
col_red = (0, 100, 80, 10, 255, 255)  # red
col_blue = (90, 90, 90, 110, 255, 255)  # blue
col_green = (65, 70, 70, 85, 255, 255)  # green
col_yellow = (26, 43, 46, 34, 255, 255)  # yellow


class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()

        # 显示模式参数 - 控制是否显示CV2窗口
        self.show_debug_window = rospy.get_param("~show_debug_window", True)
        rospy.loginfo("[Line Follower] 调试窗口显示模式: %s", "开启" if self.show_debug_window else "关闭")

        # 图像订阅
        self.use_usb_cam = rospy.get_param("/use_usb_cam", 'no')
        if self.use_usb_cam == 'yes':
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        else:
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        # 速度发布
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_ori", Twist, queue_size=1)
        self.twist = Twist()

        # 控制命令订阅
        self.control_sub = rospy.Subscriber('/line_follower/control', String, self.control_callback)

        # 继电器序列触发发布
        self.relay_trigger_pub = rospy.Publisher('/line_follower/relay_trigger', String, queue_size=10)

        # 状态机初始化
        self.state = "WAITING"  # WAITING, FOLLOWING, STOPPED, AVOIDING, RECHARGING
        self.stop_start_time = 0
        self.avoid_start_time = 0
        self.follow_start_time = time.time()
        self.waiting_start_time = time.time()
        self.recharge_start_time = 0
        self.avoid_duration = 2.0
        self.red_detected = False
        self.green_detected = False
        self.yellow_detected = False
        self.relay_sequence_completed = False

        self.last_error = 0

        # 显示相关
        self.display_initialized = False
        self.last_display_time = 0
        self.display_interval = 0.1  # 100ms显示间隔

        rospy.loginfo("[Line Follower] 寻线节点初始化完成，等待启动指令...")

        # 初始化时显示WAITING状态
        self.show_waiting_status()

    def init_display(self):
        """初始化显示窗口"""
        if not self.show_debug_window:
            self.display_initialized = False
            return

        try:
            cv2.destroyAllWindows()
            cv2.namedWindow('Line Follower', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Line Follower', 640, 480)
            self.display_initialized = True
            rospy.loginfo("[Line Follower] 显示窗口初始化成功")
        except Exception as e:
            rospy.logerr("[Line Follower] 显示窗口初始化失败: %s", str(e))
            self.display_initialized = False

    def control_callback(self, msg):
        """处理控制命令"""
        command = msg.data.lower()
        rospy.loginfo("[Line Follower] 收到控制命令: %s", command)

        if command == "start" and (self.state == "WAITING" or self.state == "RECHARGING"):
            self.state = "FOLLOWING"
            self.follow_start_time = time.time()
            # 重置检测标志
            self.red_detected = False
            self.green_detected = False
            self.yellow_detected = False
            rospy.loginfo("[Line Follower] 开始寻线任务")

        elif command == "stop":
            self.state = "WAITING"
            self.waiting_start_time = time.time()

            # 停止机器人运动
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

            # 显示静态的WAITING状态窗口
            self.show_waiting_status()
            rospy.loginfo("[Line Follower] 停止寻线任务")

        elif command == "relay_sequence_finish":
            rospy.loginfo("[Line Follower] 收到继电器序列完成通知，准备切换到AVOIDING状态")
            self.relay_sequence_completed = True

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

    def detect_green_segment(self, hsv_img):
        """检测绿色线段（统一形态学+ROI+去抖）"""
        lower = np.array([col_green[0], col_green[1], col_green[2]], dtype=np.uint8)
        upper = np.array([col_green[3], col_green[4], col_green[5]], dtype=np.uint8)
        mask = cv2.inRange(hsv_img, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        h, w = mask.shape
        roi_top = max(h - 40, 0)
        roi = mask[roi_top:h, :]
        if not hasattr(self, 'green_consec'):
            self.green_consec = 0
        ratio = float(cv2.countNonZero(roi)) / max(roi.size, 1)
        if ratio > 0.10:
            self.green_consec += 1
        elif ratio < 0.02:
            self.green_consec = 0
        if self.green_consec >= 3 and not self.green_detected:
            self.green_detected = True
            self.green_consec = 0
            return True
        return False

    def detect_yellow_segment(self, hsv_img):
        """检测黄色线段（统一形态学+ROI+去抖）"""
        lower = np.array([col_yellow[0], col_yellow[1], col_yellow[2]], dtype=np.uint8)
        upper = np.array([col_yellow[3], col_yellow[4], col_yellow[5]], dtype=np.uint8)
        mask = cv2.inRange(hsv_img, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        h, w = mask.shape
        roi_top = max(h - 40, 0)
        roi = mask[roi_top:h, :]
        if not hasattr(self, 'yellow_consec'):
            self.yellow_consec = 0
        ratio = float(cv2.countNonZero(roi)) / max(roi.size, 1)
        if ratio > 0.10:
            self.yellow_consec += 1
        elif ratio < 0.02:
            self.yellow_consec = 0
        if self.yellow_consec >= 3 and not self.yellow_detected:
            self.yellow_detected = True
            self.yellow_consec = 0
            return True
        return False

    def avoid_yellow_segment(self):
        """绕过黄色线段的策略"""
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

    def show_waiting_status(self):
        """显示静态的WAITING状态窗口"""
        if not self.show_debug_window:
            return

        try:
            if not self.display_initialized:
                self.init_display()

            if not self.display_initialized:
                return

            status_image = np.zeros((240, 320, 3), dtype=np.uint8)
            color = (200, 200, 200)  # WAITING状态的颜色
            cv2.rectangle(status_image, (5, 5), (250, 100), color, -1)

            cv2.putText(status_image, "State: WAITING", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.putText(status_image, "Robot Stopped", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(status_image, "Send 'start' to begin", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

            cv2.imshow("Line Follower", status_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("[Line Follower] 显示WAITING状态错误: %s", str(e))

    def image_callback(self, msg):
        """图像处理回调函数"""
        # WAITING和RECHARGING状态下不处理图像
        if self.state == "WAITING" or self.state == "RECHARGING":
            return

        try:
            # 转换图像消息
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if image is None or image.size == 0:
                rospy.logwarn("[Line Follower] 收到空图像或无效图像")
                return

            # 调整图像尺寸
            image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # 状态机处理
            if self.state == "FOLLOWING":
                # 红强势时抑制黄/绿误触发（red_ratio 在 follow_line 内更新）
                red_strong = hasattr(self, 'red_ratio') and self.red_ratio is not None and self.red_ratio >= 0.15
                # 检测绿线 - 优先级高于红线
                if not red_strong and self.detect_green_segment(hsv):
                    rospy.loginfo("[Line Follower] 检测到绿色线段，切换到RECHARGING状态")
                    self.state = "RECHARGING"
                    self.recharge_start_time = time.time()

                    # 停止运动
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    self.yellow_detected = False

                    # 巡线结束，发布 follow_end 消息给 mqtt_bridge
                    # mqtt_bridge 会负责关闭灯光并启动回充流程
                    end_msg = String()
                    end_msg.data = "follow_end"
                    self.relay_trigger_pub.publish(end_msg)
                    rospy.loginfo("[Line Follower] 已发送 follow_end 消息，等待 mqtt_bridge 处理回充")

                elif not red_strong and self.detect_yellow_segment(hsv):
                    self.state = "STOPPED"
                    self.stop_start_time = time.time()
                    rospy.loginfo("[Line Follower] 检测到黄色线段，切换到STOPPED状态")

                    # 停止运动
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    self.red_detected = False

                    # 触发继电器序列
                    trigger_msg = String()
                    trigger_msg.data = "trigger"
                    rospy.loginfo("[Line Follower] 准备发布继电器触发消息: %s", trigger_msg.data)
                    self.relay_trigger_pub.publish(trigger_msg)

                    # 重置继电器序列完成标志
                    self.relay_sequence_completed = False

            elif self.state == "STOPPED":
                self.yellow_detected = False
                if time.time() - self.stop_start_time > 10.0 and self.relay_sequence_completed:
                    self.state = "AVOIDING"
                    self.avoid_start_time = time.time()
                    self.last_erro = 0
                    self.relay_sequence_completed = False
                    rospy.loginfo("[Line Follower] 切换到AVOIDING状态")

            elif self.state == "AVOIDING":
                self.red_detected = False
                if self.avoid_yellow_segment():
                    self.state = "FOLLOWING"
                    self.yellow_detected = False
                    self.follow_start_time = time.time()
                    rospy.loginfo("[Line Follower] 切换回FOLLOWING状态")

            # 红线跟踪逻辑
            if self.state == "FOLLOWING":
                self.follow_line(image, hsv)
            elif self.state == "RECHARGING":
                # 回充状态下不进行运动控制，确保黄线检测标志为False
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.yellow_detected = False

            # 发布速度命令
            self.cmd_vel_pub.publish(self.twist)

            # 显示图像
            if self.show_debug_window:
                self.display_image(image)

        except Exception as e:
            rospy.logerr("[Line Follower] 图像处理错误: %s", str(e))

    def follow_line(self, image, hsv):
        """红线跟踪：双区间红掩码 + 开闭形态学 + 自适应ROI"""
        # 1) 双区间红色掩码（放宽S/V以适应暗部/反光）
        lower_red1 = np.array([0, 90, 60], dtype=np.uint8)
        upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_red2 = np.array([160, 90, 60], dtype=np.uint8)
        upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 2) 形态学：开运算后闭运算（3x3, 1次）
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        h, w, d = image.shape

        # 3) 自适应ROI：默认40px；若连续丢失则临时扩到80px
        if not hasattr(self, 'red_miss_consec'):
            self.red_miss_consec = 0
        roi_height = 80 if self.red_miss_consec >= 3 else 40
        search_top = max(h - roi_height, 0)
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)

        if M['m00'] > 0:
            self.red_detected = True
            self.red_miss_consec = 0

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)

            # 记录红线占比与质心用于互斥抑制
            red_pixels = cv2.countNonZero(mask[search_top:search_bot, :])
            total_pixels = (search_bot - search_top) * w
            self.red_ratio = float(red_pixels) / max(total_pixels, 1)
            self.red_cx = cx

            error = cx - w / 2 - 15
            d_error = error - self.last_error

            self.twist.linear.x = 0.15
            if error != 0:
                self.twist.angular.z = -float(error) * 0.005 - float(d_error) * 0.000
            else:
                self.twist.angular.z = 0

            self.last_error = error

        else:
            self.red_detected = False
            self.red_miss_consec += 1
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2  # 小幅度旋转寻找红线

        if self.show_debug_window:
            cv2.rectangle(image, (0, search_top), (w, search_bot), (0, 255, 255), 2)

    def display_image(self, image):
        """显示图像"""
        if not self.show_debug_window:
            return

        current_time = time.time()

        if current_time - self.last_display_time < self.display_interval:
            return

        self.last_display_time = current_time

        try:
            if not self.display_initialized:
                self.init_display()

            if not self.display_initialized:
                return

            state_colors = {
                "WAITING": (200, 200, 200),
                "FOLLOWING": (0, 255, 0),
                "STOPPED": (0, 0, 255),
                "AVOIDING": (0, 255, 255),
                "RECHARGING": (255, 0, 255)
            }

            color = state_colors.get(self.state, (255, 255, 255))
            cv2.rectangle(image, (5, 5), (250, 140), color, -1)

            state_duration = time.time() - {
                "WAITING": self.waiting_start_time,
                "FOLLOWING": self.follow_start_time,
                "STOPPED": self.stop_start_time,
                "AVOIDING": self.avoid_start_time,
                "RECHARGING": self.recharge_start_time
            }.get(self.state, 0)

            cv2.putText(image, "State: {}".format(self.state), (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(image, "YellowDetected: {}".format(self.yellow_detected), (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(image, "RedDetected: {}".format(self.red_detected), (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(image, "GreenDetected: {}".format(self.green_detected), (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(image, "Duration: {:.1f}s".format(state_duration), (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            cv2.imshow("Line Follower", image)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC键退出
                rospy.signal_shutdown("用户请求退出")

        except Exception as e:
            rospy.logerr("[Line Follower] 显示图像错误: %s", str(e))
            self.display_initialized = False

    def cleanup(self):
        """清理资源"""
        try:
            rospy.loginfo("[Line Follower] 开始清理资源...")

            # 销毁窗口
            if self.show_debug_window:
                cv2.destroyAllWindows()

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
