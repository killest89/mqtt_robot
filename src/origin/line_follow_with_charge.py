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
import subprocess
import json
from datetime import datetime
import signal
import os
import sys  # 新增：用于程序退出

# 导入MQTT模块
try:
    from mqtt_module import MqttClientWrapper

    MQTT_AVAILABLE = True
except ImportError:
    rospy.logwarn("[MQTT] mqtt_module not found, MQTT functionality disabled")
    MQTT_AVAILABLE = False

# MQTT配置
MQTT_CONFIG = {
    "broker": "175.178.8.235",
    "port": 1883,
    "username": "jks",
    "password": "Lcz20240!",
    "pub_topic": "Car/1",
    "sub_topic": "Server1",
    "client_id": "car_client_001"
}

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

# 全局MQTT客户端
mqtt_client = None

# 全局温度变量
current_temperature = "N/A"
temperature_lock = threading.Lock()

# 记录主线程ID - Python 2.7兼容方式
main_thread_id = threading.current_thread().ident

# 全局回充状态标志
charging_in_progress = False
charging_lock = threading.Lock()

# 全局程序退出标志
program_shutdown = False
shutdown_lock = threading.Lock()


def nothing(s):
    pass


# 颜色阈值定义
col_black = (0, 0, 0, 180, 255, 46)  # black
col_red = (0, 100, 80, 10, 255, 255)  # red
col_blue = (90, 90, 90, 110, 255, 255)  # blue
col_green = (65, 70, 70, 85, 255, 255)  # green
col_yellow = (26, 43, 46, 34, 255, 255)  # yellow

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


def is_main_thread():
    """检查当前是否为主线程 - Python 2.7兼容版本"""
    return threading.current_thread().ident == main_thread_id


def timeout_handler(signum, frame):
    """超时信号处理函数"""
    raise Exception("Process timeout")


def temperature_out_with_signal():
    """使用信号的温度获取函数 - 仅在主线程中使用"""
    try:
        # 定义要执行的脚本路径
        script_path = '/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/stream_usb_max250804.py'
        # 构建完整的命令列表
        command = ['python3', script_path]

        # 设置超时信号处理
        old_handler = signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(10)  # 10秒超时

        try:
            # 使用 subprocess.Popen 执行脚本 (Python 2.7兼容)
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()

            # 取消超时信号
            signal.alarm(0)
            signal.signal(signal.SIGALRM, old_handler)

            if process.returncode == 0:
                temp_out = stdout.decode('utf-8').strip()
                rospy.loginfo("[TEMPERATURE MAIN] Successfully got temperature: %s" % temp_out)
                return temp_out
            else:
                rospy.logerr("[TEMPERATURE MAIN ERROR] Script failed with error: %s" % stderr.decode('utf-8'))
                return None

        except Exception as e:
            # 取消超时信号
            signal.alarm(0)
            signal.signal(signal.SIGALRM, old_handler)

            # 如果是超时异常，杀死进程
            if "timeout" in str(e).lower():
                rospy.logerr("[TEMPERATURE MAIN ERROR] Temperature script timeout")
                try:
                    process.kill()
                    process.wait()
                except:
                    pass
            else:
                rospy.logerr("[TEMPERATURE MAIN ERROR] Process error: %s" % str(e))
            return None

    except Exception as e:
        rospy.logerr("[TEMPERATURE MAIN ERROR] Failed to get temperature: %s" % str(e))
        return None


def temperature_out_thread_safe():
    """线程安全的温度获取函数 - 使用轮询方式实现超时"""
    try:
        # 定义要执行的脚本路径
        script_path = '/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/stream_usb_max250804.py'
        # 构建完整的命令列表
        command = ['python3', script_path]

        # 启动进程
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # 使用轮询方式实现超时控制
        timeout = 10.0  # 10秒超时
        poll_interval = 0.1  # 100ms轮询间隔
        elapsed_time = 0.0

        while elapsed_time < timeout:
            # 检查进程是否完成
            if process.poll() is not None:
                # 进程已完成
                stdout, stderr = process.communicate()

                if process.returncode == 0:
                    temp_out = stdout.decode('utf-8').strip()
                    rospy.loginfo("[TEMPERATURE THREAD] Successfully got temperature: %s" % temp_out)
                    return temp_out
                else:
                    rospy.logerr("[TEMPERATURE THREAD ERROR] Script failed with error: %s" % stderr.decode('utf-8'))
                    return None

            # 等待并更新已用时间
            time.sleep(poll_interval)
            elapsed_time += poll_interval

        # 超时处理
        rospy.logerr("[TEMPERATURE THREAD ERROR] Temperature script timeout after %.1f seconds" % timeout)
        try:
            process.terminate()  # 先尝试优雅终止
            time.sleep(0.5)
            if process.poll() is None:
                process.kill()  # 强制终止
            process.wait()
        except Exception as kill_e:
            rospy.logerr("[TEMPERATURE THREAD ERROR] Failed to kill process: %s" % str(kill_e))

        return None

    except Exception as e:
        rospy.logerr("[TEMPERATURE THREAD ERROR] Failed to get temperature: %s" % str(e))
        return None


def temperature_out():
    """温度获取函数 - 自动选择合适的方法"""
    # 检查是否在主线程中 - Python 2.7兼容方式
    if is_main_thread():
        # 在主线程中，使用信号方式
        rospy.loginfo("[TEMPERATURE] temperature_out in main thread")
        return temperature_out_with_signal()
    else:
        rospy.loginfo("[TEMPERATURE] temperature_out in sub thread")
        # 在子线程中，使用轮询方式
        return temperature_out_thread_safe()


def check_temperature_and_send():
    """检测温度并发送到服务器的独立函数"""
    rospy.loginfo("[TEMPERATURE] Getting temperature reading...")
    temperature_data = temperature_out()

    # 更新全局温度变量
    update_global_temperature(temperature_data)

    if temperature_data is not None:
        # 发送温度数据到服务器
        success = send_temperature_to_server(temperature_data)
        if success:
            rospy.loginfo("[TEMPERATURE] Periodic temperature data sent to server successfully")
        else:
            rospy.logwarn("[TEMPERATURE] Failed to send periodic temperature data to server")
    else:
        rospy.logwarn("[TEMPERATURE] Failed to get periodic temperature data")


def server_callback(topic, message):
    """MQTT服务器消息回调函数"""
    try:
        # 直接发送字节数据
        if isinstance(message, bytes):
            send_relay_command(message)
            rospy.loginfo("[MQTT CALLBACK] Binary command sent")
            return

        # 处理文本数据
        hex_data = str(message).strip().replace(' ', '').replace('0x', '').replace('0X', '')

        # 转换为字节字符串（Python 2.7方式）
        byte_data = ''
        for i in range(0, len(hex_data), 2):
            byte_data += chr(int(hex_data[i:i + 2], 16))

        send_relay_command(byte_data)
        rospy.loginfo("[MQTT CALLBACK] Command sent: %s" % hex_data)

    except Exception as e:
        rospy.logerr("[MQTT CALLBACK] Error: %s" % str(e))


def update_global_temperature(temp_value):
    """更新全局温度变量"""
    global current_temperature, temperature_lock
    with temperature_lock:
        current_temperature = temp_value if temp_value is not None else "Error"


def get_current_temperature():
    """获取当前温度值"""
    global current_temperature
    return current_temperature


def init_mqtt_client():
    """初始化MQTT客户端"""
    global mqtt_client
    if not MQTT_AVAILABLE:
        rospy.logwarn("[MQTT] MQTT module not available")
        return False

    try:
        mqtt_client = MqttClientWrapper(
            broker=MQTT_CONFIG["broker"],
            port=MQTT_CONFIG["port"],
            client_id=MQTT_CONFIG["client_id"],
            username=MQTT_CONFIG["username"],
            password=MQTT_CONFIG["password"]
        )
        rospy.loginfo("[MQTT] MQTT client initialized successfully")

        # 订阅服务器消息
        try:
            rospy.loginfo("[MQTT] Starting subscription to topic: %s" % MQTT_CONFIG["sub_topic"])
            mqtt_client.subscribe(MQTT_CONFIG["sub_topic"], server_callback, qos=1, run_in_thread=True)
            rospy.loginfo("[MQTT] Successfully subscribed to server messages")
        except Exception as e:
            rospy.logerr("[MQTT ERROR] Failed to subscribe to server messages: %s" % str(e))

        return True
    except Exception as e:
        rospy.logerr("[MQTT ERROR] Failed to initialize MQTT client: %s" % str(e))
        mqtt_client = None
        return False


def send_temperature_to_server(temperature_data):
    """发送温度数据到服务器"""
    global mqtt_client

    if mqtt_client is None:
        rospy.logwarn("[MQTT] MQTT client not initialized")
        return False

    try:
        # 构建消息数据
        message_data = {
            "timestamp": datetime.now().isoformat(),
            "device_id": MQTT_CONFIG["client_id"],
            "temperature": temperature_data,
            "data_type": "temperature_reading"
        }

        # 转换为JSON字符串
        message_json = json.dumps(message_data)

        # 发送MQTT消息
        mqtt_client.publish(MQTT_CONFIG["pub_topic"], message_json, qos=1)
        rospy.loginfo("[MQTT] Temperature data sent successfully: %s" % message_json)
        return True

    except Exception as e:
        rospy.logerr("[MQTT ERROR] Failed to send temperature data: %s" % str(e))
        return False


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
    # ========== 在这里添加温度获取和发送 ==========
    check_temperature_and_send()
    # ============================================

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

    check_temperature_and_send()  # 增加一次温度获取并通过mqtt发送

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


# ==================== 新增：程序退出和回充功能 ====================
def graceful_shutdown():
    """优雅地关闭程序"""
    global program_shutdown, serial_port

    with shutdown_lock:
        if program_shutdown:
            return
        program_shutdown = True

    rospy.loginfo("[SHUTDOWN] Starting graceful shutdown...")

    # 停止机器人运动
    try:
        if 'follower' in globals():
            follower.twist.linear.x = 0
            follower.twist.angular.z = 0
            follower.cmd_vel_pub.publish(follower.twist)
            rospy.loginfo("[SHUTDOWN] Robot stopped")
    except:
        pass

    # 关闭所有继电器
    if serial_port is not None and serial_port.isOpen():
        rospy.loginfo("[SHUTDOWN] Closing serial port and turning off all relays")
        send_relay_command(RELAY_COMMANDS["8_OFF"])
        time.sleep(0.5)
        serial_port.close()

    # 关闭OpenCV窗口
    try:
        cv2.destroyAllWindows()
    except:
        pass

    rospy.loginfo("[SHUTDOWN] Line following program shutdown completed")


def execute_auto_recharge():
    """执行自动回充程序 - 使用subprocess调用独立脚本"""
    global charging_in_progress

    with charging_lock:
        if charging_in_progress:
            rospy.logwarn("[RECHARGE] Charging already in progress, skipping")
            return
        charging_in_progress = True

    try:
        rospy.loginfo("[RECHARGE] Starting auto recharge sequence...")

        # 发送回充开始状态到服务器
        send_charging_status_to_server("CHARGING_STARTED")

        # 使用subprocess调用独立的回充脚本
        script_path = '/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/auto_recharger_v1.py'

        rospy.loginfo("[RECHARGE] Launching auto_recharger_v1.py script...")
        process = subprocess.Popen(['python', script_path],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

        # 等待进程完成
        try:
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                rospy.loginfo("[RECHARGE] Auto recharge completed successfully")
                send_charging_status_to_server("CHARGING_COMPLETED")
            else:
                rospy.logerr("[RECHARGE ERROR] Auto recharge failed: %s" % stderr.decode('utf-8'))
                send_charging_status_to_server("CHARGING_FAILED")

        except Exception as comm_e:
            rospy.logerr("[RECHARGE ERROR] Communication error: %s" % str(comm_e))
            send_charging_status_to_server("CHARGING_FAILED")

    except Exception as e:
        rospy.logerr("[RECHARGE ERROR] Failed to execute auto recharge: %s" % str(e))
        send_charging_status_to_server("CHARGING_FAILED")
    finally:
        with charging_lock:
            charging_in_progress = False


def send_charging_status_to_server(status):
    """发送回充状态到服务器"""
    global mqtt_client

    if mqtt_client is None:
        rospy.logwarn("[RECHARGE] MQTT client not initialized, cannot send charging status")
        return False

    try:
        # 构建回充状态消息
        message_data = {
            "timestamp": datetime.now().isoformat(),
            "device_id": MQTT_CONFIG["client_id"],
            "charging_status": status,
            "data_type": "charging_status"
        }

        # 转换为JSON字符串
        message_json = json.dumps(message_data)

        # 发送MQTT消息
        mqtt_client.publish(MQTT_CONFIG["pub_topic"], message_json, qos=1)
        rospy.loginfo("[RECHARGE] Charging status sent: %s" % message_json)
        return True

    except Exception as e:
        rospy.logerr("[RECHARGE ERROR] Failed to send charging status: %s" % str(e))
        return False


def shutdown_and_start_charging():
    """关闭巡线程序并启动回充进程"""
    rospy.loginfo("[RECHARGE] Shutting down line following and starting charging...")

    # 先启动回充进程（在独立线程中）
    charging_thread = threading.Thread(target=execute_auto_recharge)
    charging_thread.daemon = False  # 不设置为守护线程，确保回充完成
    charging_thread.start()

    # 等待一小段时间确保回充进程启动
    time.sleep(2)

    # 优雅关闭当前程序
    graceful_shutdown()

    # 退出程序
    rospy.loginfo("[RECHARGE] Exiting line following program for charging...")
    rospy.signal_shutdown("Starting charging process")
    sys.exit(0)


# ================================================================

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
        self.state = "INIT"  # INIT, FOLLOWING, STOPPED, AVOIDING, BLUE_DETECTED
        self.stop_start_time = 0
        self.avoid_start_time = 0
        self.blue_detect_start_time = 0  # 新增：蓝色检测开始时间
        self.follow_start_time = time.time()
        self.avoid_duration = 2.0
        self.blue_wait_duration = 5.0  # 新增：蓝色检测等待时间
        self.red_detected = False
        self.blue_detected = False  # 新增：蓝色检测标志
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

    def detect_blue_segment(self, hsv_img):
        """检测蓝色线段"""
        blue_mask = cv2.inRange(hsv_img, (col_blue[0], col_blue[1], col_blue[2]),
                                (col_blue[3], col_blue[4], col_blue[5]))
        h, w = blue_mask.shape
        roi = blue_mask[3 * h // 4:h, :]  # 检测图像下方区域
        blue_pixels = cv2.countNonZero(roi)
        total_pixels = roi.size
        ratio = float(blue_pixels) / total_pixels

        # 如果蓝色像素比例超过阈值且之前未检测到蓝色
        if ratio > 0.05 and not self.blue_detected:
            self.blue_detected = True
            return True
        elif ratio < 0.01:
            self.blue_detected = False
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
        global program_shutdown

        # 检查程序是否正在关闭
        if program_shutdown:
            return

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
            # 检测蓝色线段（回充标志）
            if self.detect_blue_segment(hsv):
                self.state = "BLUE_DETECTED"
                self.blue_detect_start_time = time.time()
                rospy.loginfo("[STATE] Blue segment detected! Waiting 5 seconds before charging...")
                # 立即停止机器人
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

            # 检测红色线段（停止标志）
            elif self.detect_red_segment(hsv):
                self.state = "STOPPED"
                self.stop_start_time = time.time()
                rospy.loginfo("[STATE] Red segment detected! Switching to STOPPED state")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

                # 启动继电器控制序列线程
                if self.sequence_thread is None or not self.sequence_thread.is_alive():
                    self.sequence_thread = threading.Thread(target=execute_relay_sequence)
                    self.sequence_thread.start()

        elif self.state == "BLUE_DETECTED":
            # 蓝色检测状态：等待5秒后启动回充
            elapsed = time.time() - self.blue_detect_start_time
            remaining_time = self.blue_wait_duration - elapsed

            rospy.loginfo("[BLUE_DETECTED] Waiting for charging... %.1f seconds remaining" % remaining_time)

            # 保持停止状态
            self.twist.linear.x = 0
            self.twist.angular.z = 0

            if elapsed >= self.blue_wait_duration:
                rospy.loginfo("[BLUE_DETECTED] 5 seconds elapsed, starting charging process...")
                # 关闭程序并启动回充
                shutdown_and_start_charging()
                return

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

        elif self.state == "CHARGING":
            # 在回充状态下，机器人停止运动，等待回充完成
            # 可以添加回充完成的检测逻辑，这里暂时保持停止状态
            self.twist.linear.x = 0
            self.twist.angular.z = 0

        # ================== 黑线跟踪 ==================
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

        # ================== 发布速度命令 ==================
        if not program_shutdown:
            self.cmd_vel_pub.publish(self.twist)

        # ================== 可视化增强 ==================
        state_colors = {
            "INIT": (200, 200, 200),
            "FOLLOWING": (0, 255, 0),
            "STOPPED": (0, 0, 255),
            "AVOIDING": (0, 255, 255),
            "BLUE_DETECTED": (255, 165, 0)  # 新增：蓝色检测状态用橙色
        }
        color = state_colors.get(self.state, (255, 255, 255))
        cv2.rectangle(image, (5, 5), (250, 140), color, -1)  # 增加矩形高度以容纳更多信息

        state_duration = time.time() - {
            "FOLLOWING": self.follow_start_time,
            "STOPPED": self.stop_start_time,
            "AVOIDING": self.avoid_start_time,
            "BLUE_DETECTED": self.blue_detect_start_time
        }.get(self.state, 0)

        cv2.putText(image, "State: {}".format(self.state), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "RedDetected: {}".format(self.red_detected), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "BlueDetected: {}".format(self.blue_detected), (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(image, "Duration: {:.1f}s".format(state_duration), (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 添加温度显示
        cv2.putText(image, "Temp: {}".format(get_current_temperature()), (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 显示回充状态
        global charging_in_progress
        charging_status = "YES" if charging_in_progress else "NO"
        cv2.putText(image, "Charging: {}".format(charging_status), (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 如果在蓝色检测状态，显示倒计时
        if self.state == "BLUE_DETECTED":
            elapsed = time.time() - self.blue_detect_start_time
            remaining = max(0, self.blue_wait_duration - elapsed)
            cv2.putText(image, "Charging in: {:.1f}s".format(remaining), (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # ================== 显示窗口 ==================
        if not program_shutdown:
            cv2.imshow("window", image)
            cv2.imshow("Adjust_hsv", mask)
            cv2.waitKey(3)


# ==================== 主程序入口 ====================
if __name__ == "__main__":
    try:
        rospy.init_node("opencv")
        init_serial_port()
        init_mqtt_client()  # 初始化MQTT客户端

        # 创建全局follower对象
        follower = Follower()

        rospy.loginfo("[MAIN] Line following program started successfully")

        # 注册关闭回调
        rospy.on_shutdown(graceful_shutdown)

        # 开始主循环
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("[MAIN] ROS interrupt received")
    except KeyboardInterrupt:
        rospy.loginfo("[MAIN] Keyboard interrupt received")
    except Exception as e:
        rospy.logerr("[MAIN ERROR] Unexpected error: %s" % str(e))
    finally:
        # 确保程序退出时执行清理
        graceful_shutdown()
