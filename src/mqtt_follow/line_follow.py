#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy
import time
from geometry_msgs.msg import Twist
import serial
import threading
import subprocess
import json
from datetime import datetime
import signal
import os

# 颜色阈值定义
col_black = (0, 0, 0, 180, 255, 46)  # black
col_red = (0, 100, 80, 10, 255, 255)  # red
col_blue = (90, 90, 90, 110, 255, 255)  # blue
col_green = (65, 70, 70, 85, 255, 255)  # green
col_yellow = (26, 43, 46, 34, 255, 255)  # yellow

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'

# 定义所有继电器控制指令
RELAY_COMMANDS = {
    "1_ON": b'\xA0\x01\x01\xA2', "1_OFF": b'\xA0\x01\x00\xA1',
    "2_ON": b'\xA0\x02\x01\xA3', "2_OFF": b'\xA0\x02\x00\xA2',
    "3_ON": b'\xA0\x03\x01\xA4', "3_OFF": b'\xA0\x03\x00\xA3',
    "4_ON": b'\xA0\x04\x01\xA5', "4_OFF": b'\xA0\x04\x00\xA4',
    "8_ON": b'\xA0\x08\x01\xA9', "8_OFF": b'\xA0\x08\x00\xA8'
}

# 全局变量
serial_port = None
serial_lock = threading.Lock()
current_temperature = "N/A"
temperature_lock = threading.Lock()
main_thread_id = threading.current_thread().ident


def nothing(s):
    pass


def is_main_thread():
    """检查当前是否为主线程"""
    return threading.current_thread().ident == main_thread_id


def init_serial_port():
    """初始化串口连接"""
    global serial_port
    try:
        serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)
        rospy.loginfo("[SERIAL] Serial port initialized successfully")
        return True
    except serial.SerialException as e:
        rospy.logerr("[SERIAL ERROR] Could not open serial port: %s" % e)
        serial_port = None
        return False


def send_relay_command(command):
    """发送单个继电器控制指令 - 修复hex()调用问题"""
    global serial_port, serial_lock

    if serial_port is None:
        rospy.logwarn("[SERIAL] Serial port not initialized, skipping command")
        return False

    try:
        with serial_lock:
            if not serial_port.isOpen():
                rospy.logwarn("[SERIAL] Port closed, trying to reopen")
                try:
                    serial_port.open()
                except:
                    rospy.logerr("[SERIAL] Failed to reopen serial port")
                    return False

            if serial_port.isOpen():
                # 确保command是bytes类型
                if isinstance(command, str):
                    # 如果是字符串，尝试转换为bytes
                    command = command.encode('latin-1')

                serial_port.write(command)
                # 安全地记录命令内容
                try:
                    if isinstance(command, bytes):
                        cmd_hex = command.hex()
                    else:
                        cmd_hex = str(command)
                    rospy.loginfo("[SERIAL] Command sent: %s" % cmd_hex)
                except AttributeError:
                    rospy.loginfo("[SERIAL] Command sent (hex format not available)")
                return True
        return False
    except Exception as e:
        rospy.logerr("[SERIAL ERROR] Failed to send command: %s" % e)
        return False


def temperature_out_thread_safe():
    """线程安全的温度获取函数 - 使用轮询方式实现超时[5](@ref)"""
    try:
        script_path = '/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/stream_usb_max250804.py'
        command = ['python3', script_path]

        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        timeout = 10.0
        poll_interval = 0.1
        elapsed_time = 0.0

        while elapsed_time < timeout:
            if process.poll() is not None:
                stdout, stderr = process.communicate()
                if process.returncode == 0:
                    temp_out = stdout.decode('utf-8').strip()
                    rospy.loginfo("[TEMPERATURE] Successfully got temperature: %s" % temp_out)
                    return temp_out
                else:
                    rospy.logerr("[TEMPERATURE ERROR] Script failed with error: %s" % stderr.decode('utf-8'))
                    return None
            time.sleep(poll_interval)
            elapsed_time += poll_interval

        rospy.logerr("[TEMPERATURE ERROR] Temperature script timeout")
        try:
            process.terminate()
            time.sleep(0.5)
            if process.poll() is None:
                process.kill()
            process.wait()
        except:
            pass
        return None

    except Exception as e:
        rospy.logerr("[TEMPERATURE ERROR] Failed to get temperature: %s" % str(e))
        return None


def update_global_temperature(temp_value):
    """更新全局温度变量"""
    global current_temperature, temperature_lock
    with temperature_lock:
        current_temperature = temp_value if temp_value is not None else "Error"


def get_current_temperature():
    """获取当前温度值"""
    global current_temperature
    return current_temperature


def check_temperature_and_send():
    """检测温度并发送到服务器的独立函数"""
    rospy.loginfo("[TEMPERATURE] Getting temperature reading...")
    temperature_data = temperature_out_thread_safe()
    update_global_temperature(temperature_data)

    if temperature_data is not None and mqtt_controller is not None:
        success = mqtt_controller.send_temperature_data(temperature_data)
        if success:
            rospy.loginfo("[TEMPERATURE] Temperature data sent to server successfully")
        else:
            rospy.logwarn("[TEMPERATURE] Failed to send temperature data to server")
    else:
        rospy.logwarn("[TEMPERATURE] Failed to get temperature data or MQTT not available")


def execute_relay_sequence():
    """执行复杂的继电器控制序列"""
    rospy.loginfo("[RELAY] Starting relay sequence")

    # 先获取并发送温度数据
    check_temperature_and_send()

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

    check_temperature_and_send()  # 增加一次温度获取并通过mqtt发送

    # 第四路开启
    send_relay_command(RELAY_COMMANDS["4_ON"])
    time.sleep(1)

    # 第四路关闭再开启（重复3次）
    for _ in range(3):
        send_relay_command(RELAY_COMMANDS["4_OFF"])
        time.sleep(1)
        send_relay_command(RELAY_COMMANDS["4_ON"])
        time.sleep(1)

    # 第四路关闭及第三路开启
    send_relay_command(RELAY_COMMANDS["4_OFF"])
    time.sleep(1)
    send_relay_command(RELAY_COMMANDS["3_ON"])
    time.sleep(2)

    check_temperature_and_send()  # 增加一次温度获取并通过mqtt发送

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


class LineFollower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.use_usb_cam = rospy.get_param("/use_usb_cam", 'no')

        # 图像订阅者
        if self.use_usb_cam == 'yes':
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        else:
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel_ori", Twist, queue_size=1)
        self.twist = Twist()
        self.temp = 0

        # 状态机初始化
        self.state = "INIT"  # INIT, FOLLOWING, STOPPED, AVOIDING, IDLE
        self.stop_start_time = 0
        self.avoid_start_time = 0
        self.follow_start_time = time.time()
        self.avoid_duration = 2.0
        self.red_detected = False
        self.last_erro = 0
        self.sequence_thread = None
        self.line_following_enabled = False

        # 初始发送第八路开启指令
        if send_relay_command(RELAY_COMMANDS["8_ON"]):
            self.state = "IDLE"
            rospy.loginfo("[LINE FOLLOWER] Initialized in IDLE state, waiting for start command")
        else:
            self.state = "ERROR"
            rospy.logerr("[LINE FOLLOWER] Failed to initialize relay")

    def enable_line_following(self):
        """启用寻线功能 - 由MQTT控制器调用"""
        if self.state == "IDLE":
            self.state = "FOLLOWING"
            self.line_following_enabled = True
            self.follow_start_time = time.time()
            rospy.loginfo("[LINE FOLLOWER] Line following enabled")
            return True
        return False

    def disable_line_following(self):
        """禁用寻线功能 - 由MQTT控制器调用"""
        self.line_following_enabled = False
        self.state = "IDLE"
        # 停止机器人运动
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("[LINE FOLLOWER] Line following disabled")

    def detect_red_segment(self, hsv_img):
        """检测红色线段[1](@ref)"""
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
        """绕过红色线段的策略[1](@ref)"""
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
        """图像回调函数 - 主要的控制逻辑[7](@ref)"""
        if not self.line_following_enabled:
            # 如果寻线未启用，只显示图像但不进行控制
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
                cv2.putText(image, "STATE: IDLE (Waiting for start command)", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.imshow("window", image)
                cv2.waitKey(1)
            except:
                pass
            return

        # 初始化图像处理
        mask = numpy.zeros((240, 320), dtype=numpy.uint8)

        if self.temp == 0:
            cv2.namedWindow('Adjust_hsv', cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch, 'Adjust_hsv', 4, 4, nothing)
            self.temp = 1

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            rospy.logerr("[IMAGE PROCESSING] Error processing image: %s" % str(e))
            return

        # ================== 状态机处理 ==================
        if self.state == "FOLLOWING":
            if self.detect_red_segment(hsv):
                self.state = "STOPPED"
                self.stop_start_time = time.time()
                rospy.loginfo("[STATE] Red segment detected, switching to STOPPED state")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

                # 启动继电器控制序列线程
                if self.sequence_thread is None or not self.sequence_thread.is_alive():
                    self.sequence_thread = threading.Thread(target=execute_relay_sequence)
                    self.sequence_thread.daemon = True
                    self.sequence_thread.start()

        elif self.state == "STOPPED":
            if time.time() - self.stop_start_time > 10.0 and (
                    self.sequence_thread is None or not self.sequence_thread.is_alive()):
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

        # ================== 黄线跟踪逻辑 ==================[6](@ref)
        if self.state == "FOLLOWING":
            kernel = numpy.ones((5, 5), numpy.uint8)
            hsv_erode = cv2.erode(hsv, kernel, iterations=1)
            hsv_dilate = cv2.dilate(hsv_erode, kernel, iterations=1)
            mask = cv2.inRange(hsv_dilate, (col_yellow[0], col_yellow[1], col_yellow[2]),
                               (col_yellow[3], col_yellow[4], col_yellow[5]))
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
        elif self.state == "STOPPED":
            # 停止状态，保持静止
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        elif self.state == "AVOIDING":
            # 避障状态，使用avoid_red_segment函数控制
            pass  # 速度已在avoid_red_segment中设置

        # ================== 发布速度命令 ==================
        self.cmd_vel_pub.publish(self.twist)

        # ================== 可视化增强 ==================
        state_colors = {
            "INIT": (200, 200, 200),
            "IDLE": (150, 150, 150),
            "FOLLOWING": (0, 255, 0),
            "STOPPED": (0, 0, 255),
            "AVOIDING": (0, 255, 255),
            "ERROR": (255, 0, 0)
        }
        color = state_colors.get(self.state, (255, 255, 255))
        cv2.rectangle(image, (5, 5), (300, 100), color, -1)

        state_duration = time.time() - {
            "FOLLOWING": self.follow_start_time,
            "STOPPED": self.stop_start_time,
            "AVOIDING": self.avoid_start_time
        }.get(self.state, time.time())

        # 显示状态信息
        cv2.putText(image, "State: {}".format(self.state), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "RedDetected: {}".format(self.red_detected), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "Duration: {:.1f}s".format(state_duration), (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "Temp: {}".format(get_current_temperature()), (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # ================== 显示窗口 ==================
        cv2.imshow("window", image)
        cv2.imshow("Adjust_hsv", mask)
        cv2.waitKey(3)


def mqtt_command_callback(command):
    """MQTT命令回调函数"""
    global mqtt_controller
    rospy.loginfo("[MQTT] Received command: %s" % command)

    if command.lower() == "start":
        if follower.enable_line_following():
            mqtt_controller.send_status("RUNNING")
        else:
            rospy.logwarn("[MQTT] Cannot start line following - not in IDLE state")
    elif command.lower() == "stop":
        follower.disable_line_following()
        mqtt_controller.send_status("IDLE")
    elif command.lower() == "status":
        mqtt_controller.send_status()
    elif command.lower() == "temperature":
        check_temperature_and_send()


def main():
    rospy.init_node("line_follower")
    rospy.loginfo("[MAIN] Starting Line Follower Node")

    # 初始化串口
    if not init_serial_port():
        rospy.logerr("[MAIN] Failed to initialize serial port, exiting")
        return

    # 创建寻线器实例 - 移除了MQTT控制器初始化
    follower = LineFollower()

    rospy.loginfo("[MAIN] Line follower node ready. Waiting for start command...")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[MAIN] Shutting down line follower node")
    finally:
        # 清理资源
        temperature_timer.shutdown()
        follower.disable_line_following()

        # 关闭所有继电器
        if serial_port is not None and serial_port.isOpen():
            rospy.loginfo("[MAIN] Closing serial port and turning off all relays")
            send_relay_command(RELAY_COMMANDS["8_OFF"])
            time.sleep(0.5)
            serial_port.close()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
