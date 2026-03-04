#!/usr/bin/env python
# coding=utf-8

import logging
import os
import rospy
import threading
import time
from mqtt_module import MqttClientWrapper

# MQTT配置参数
MQTT_CONFIG = {
    "broker": "175.178.8.235",
    "port": 1883,
    "username": "jks",
    "password": "Lcz20240!",
    "pub_topic": "Car/1",
    "sub_topic": "Server1",
    "client_id": "car_client_001",
    "keepalive": 60
}


class MQTTController:
    def __init__(self, command_callback=None):
        """初始化MQTT控制器

        Args:
            command_callback: 可选的命令回调函数
        """
        self.setup_logging()
        self.logger.info("MQTT控制器初始化开始")
        self.mqtt_client = None
        self.line_following_active = False
        self.line_follow_process = None
        self.command_callback = command_callback  # 存储回调函数

        self.init_ros()
        self.init_mqtt()

    def setup_logging(self):
        """配置日志系统"""
        # 创建logger
        self.logger = logging.getLogger('MQTTController')
        self.logger.setLevel(logging.DEBUG)

        # 避免重复添加处理器
        if self.logger.handlers:
            return

        # 日志格式
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        # 文件处理器
        log_dir = '/home/wheeltec/wheeltec_robot/logs'
        # os.makedirs(log_dir, exist_ok=True)

        file_handler = logging.FileHandler('/home/wheeltec/wheeltec_robot/logs/mqtt_controller_detail.log')
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

        self.logger.debug("日志系统配置完成")

    def init_ros(self):
        """初始化ROS节点"""
        if not rospy.get_node_uri():
            rospy.init_node('mqtt_controller', anonymous=True)
        rospy.loginfo("[MQTT Controller] ROS node initialized")

    def init_mqtt(self):
        """初始化MQTT客户端"""
        try:
            self.mqtt_client = MqttClientWrapper(
                broker=MQTT_CONFIG["broker"],
                port=MQTT_CONFIG["port"],
                client_id=MQTT_CONFIG["client_id"],
                username=MQTT_CONFIG["username"],
                password=MQTT_CONFIG["password"]
            )

            # 订阅控制主题
            self.mqtt_client.subscribe(
                topic=MQTT_CONFIG["sub_topic"],
                callback=self._command_callback_handler,  # 使用统一处理函数
                qos=1,
                run_in_thread=True
            )

            rospy.loginfo("[MQTT Controller] MQTT client initialized and subscribed")

            # 发布就绪状态
            self._publish_status("READY")

        except Exception as e:
            rospy.logerr("[MQTT Controller] Failed to initialize MQTT: %s", str(e))

    def _command_callback_handler(self, topic, message):
        """统一处理MQTT命令回调"""
        try:
            command = message.strip().lower()
            rospy.loginfo("[MQTT Controller] Received command: %s", command)

            # 首先执行内部命令处理
            if command == "start":
                self._start_line_following()
            elif command == "stop":
                self._stop_line_following()
            elif command == "status":
                self._publish_status()
            else:
                rospy.logwarn("[MQTT Controller] Unknown command: %s", command)

            # 如果有外部回调函数，也执行它
            if self.command_callback:
                self.command_callback(command)

        except Exception as e:
            rospy.logerr("[MQTT Controller] Error processing command: %s", str(e))

    def _start_line_following(self):
        """启动寻线逻辑"""
        if self.line_following_active:
            rospy.logwarn("[MQTT Controller] Line following is already active")
            return

        try:
            rospy.loginfo("[MQTT Controller] Starting line following...")

            # 使用roslaunch启动寻线节点
            import subprocess
            self.line_follow_process = subprocess.Popen([
                'roslaunch', 'simple_follower', 'line_follower.launch'
            ])

            self.line_following_active = True
            self._publish_status("RUNNING")
            rospy.loginfo("[MQTT Controller] Line following started successfully")

        except Exception as e:
            rospy.logerr("[MQTT Controller] Failed to start line following: %s", str(e))
            self._publish_status("ERROR")

    def _stop_line_following(self):
        """停止寻线逻辑"""
        if not self.line_following_active:
            rospy.logwarn("[MQTT Controller] Line following is not active")
            return

        try:
            rospy.loginfo("[MQTT Controller] Stopping line following...")

            if self.line_follow_process:
                self.line_follow_process.terminate()
                self.line_follow_process.wait(timeout=5)

            self.line_following_active = False
            self._publish_status("READY")
            rospy.loginfo("[MQTT Controller] Line following stopped successfully")

        except Exception as e:
            rospy.logerr("[MQTT Controller] Failed to stop line following: %s", str(e))
            self._publish_status("ERROR")

    def _publish_status(self, status=None):
        """发布系统状态"""
        if not self.mqtt_client:
            return

        try:
            status_msg = {
                "timestamp": time.time(),
                "device_id": MQTT_CONFIG["client_id"],
                "status": status or ("RUNNING" if self.line_following_active else "READY"),
                "line_following_active": self.line_following_active
            }

            import json
            self.mqtt_client.publish(
                topic=MQTT_CONFIG["pub_topic"],
                message=json.dumps(status_msg),
                qos=1
            )

        except Exception as e:
            rospy.logerr("[MQTT Controller] Failed to publish status: %s", str(e))

    def run(self):
        """主循环"""
        rospy.loginfo("[MQTT Controller] Controller is running...")
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            # 定期发布状态
            if self.mqtt_client:
                self._publish_status()
            rate.sleep()


def main():
    rospy.loginfo("[MQTT Controller] Main start...")
    controller = MQTTController()  # 不传递callback参数，使用内部处理

    try:
        controller.logger.info("[MQTT Controller] MQTT控制器启动成功")
        controller.run()
    except Exception as e:
        controller.logger.error("[MQTT Controller] 控制器运行错误: %s", str(e))
        rospy.logerr("[MQTT Controller] MQTTController Failed: %s", str(e))
    finally:
        controller.logger.info("[MQTT Controller] MQTT控制器退出")
        rospy.loginfo("[MQTT Controller] MQTTController Shutting down...")


if __name__ == "__main__":
    main()