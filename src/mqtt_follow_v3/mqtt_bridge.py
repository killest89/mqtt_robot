#!/usr/bin/env python
# coding=utf-8
# mqtt_bridge.py - MQTT通信节点（增强稳定性版）

import sys
if sys.version_info[0] == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')

import rospy
import json
import threading
import time
import serial
import subprocess
import binascii
import socket
import struct
import traceback
import os
import signal
from std_msgs.msg import String
from datetime import datetime

# MQTT配置
MQTT_CONFIG = {
    "broker": "175.178.8.235",
    "port": 1883,
    "username": "jks",
    "password": "Lcz20240!",
    "pub_topic": "Car/1",
    "sub_topic": "Server1",
    "client_id": "car_client_001",
    "keepalive": 60,  # 增加心跳间隔
    "reconnect_min_delay": 5,  # 最小重连延迟
    "reconnect_max_delay": 60  # 最大重连延迟
}

# 继电器指令定义
RELAY_COMMANDS = {
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

# Python 2.7兼容性定义
try:
    ConnectionError
except NameError:
    ConnectionError = socket.error


class MQTTBridge:
    def __init__(self):
        rospy.init_node('mqtt_bridge', anonymous=True)

        # 首先初始化所有锁对象
        self.serial_lock = threading.Lock()
        self.mqtt_lock = threading.Lock()
        self.temperature_lock = threading.Lock()

        # 然后再初始化其他可能抛出异常的组件
        self.control_pub = rospy.Publisher('/line_follower/control', String, queue_size=10)
        self.relay_trigger_sub = rospy.Subscriber('/line_follower/relay_trigger', String, self.relay_sequence_handler)

        # 初始化状态变量（在锁之后）
        self.relay_sequence_running = False
        self.mqtt_connected = False
        self.reconnect_count = 0
        self.last_reconnect_attempt = 0
        self.current_temperature = "N/A"

        # 回充相关状态
        self.recharge_triggered = False
        self.recharge_proc = None
        self.recharge_lock = threading.Lock()
        
        # 巡线节点相关状态
        self.line_follower_running = True  # 初始启动时巡线节点是运行的
        self.current_mode = "idle"  # 当前运行模式: "idle", "auto", "manual"

        # 最后初始化串口和mqtt
        self.serial_port = None
        self.init_serial_port()

        self.mqtt_client = None
        self.init_mqtt_client()

        rospy.loginfo("[MQTT Bridge] MQTT桥接节点初始化完成")

    def init_serial_port(self):
        """初始化串口连接"""
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)
            rospy.loginfo("[MQTT Bridge] 串口初始化成功")
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 串口初始化失败: %s", str(e))
            self.serial_port = None

    def init_mqtt_client(self):
        """初始化MQTT客户端 - 增强错误处理"""
        try:
            from mqtt_module import MqttClientWrapper

            with self.mqtt_lock:
                if self.mqtt_client is not None:
                    try:
                        self.mqtt_client.disconnect()
                    except:
                        pass
                    self.mqtt_client = None

                self.mqtt_client = MqttClientWrapper(
                    broker=MQTT_CONFIG["broker"],
                    port=MQTT_CONFIG["port"],
                    client_id=MQTT_CONFIG["client_id"],
                    username=MQTT_CONFIG["username"],
                    password=MQTT_CONFIG["password"]
                )

                # 设置更保守的连接参数
                if hasattr(self.mqtt_client, 'client'):
                    client = self.mqtt_client.client
                    client.connect_timeout = 10  # 增加连接超时
                    client.keepalive = MQTT_CONFIG["keepalive"]
                    # 设置自动重连策略（指数退避）
                    client.reconnect_delay_set(
                        min_delay=MQTT_CONFIG["reconnect_min_delay"],
                        max_delay=MQTT_CONFIG["reconnect_max_delay"]
                    )
                    # 添加连接状态回调
                    client.on_connect = self.on_mqtt_connect
                    client.on_disconnect = self.on_mqtt_disconnect

                # 订阅服务器消息 - 不使用线程模式，避免内部线程问题
                self.mqtt_client.subscribe(MQTT_CONFIG["sub_topic"], self.server_message_callback, qos=1,
                                           run_in_thread=False)

                rospy.loginfo("[MQTT Bridge] MQTT客户端初始化成功")
                self.mqtt_connected = True
                self.reconnect_count = 0
                self.last_reconnect_attempt = time.time()

        except ImportError:
            rospy.logerr("[MQTT Bridge] mqtt_module未找到，MQTT功能禁用")
            self.mqtt_client = None
            self.mqtt_connected = False
        except Exception as e:
            rospy.logerr("[MQTT Bridge] MQTT初始化失败: %s", str(e))
            self.mqtt_client = None
            self.mqtt_connected = False

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接成功回调"""
        if rc == 0:
            rospy.loginfo("[MQTT Bridge] 成功连接到MQTT服务器")
            self.mqtt_connected = True
            self.reconnect_count = 0
        else:
            rospy.logwarn("[MQTT Bridge] 连接失败，返回码: %s", rc)
            self.mqtt_connected = False

    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调"""
        rospy.logwarn("[MQTT Bridge] MQTT连接断开，返回码: %s", rc)
        self.mqtt_connected = False

    def safe_mqtt_loop(self):
        """安全的MQTT循环处理 - 增强异常处理"""
        if not self.mqtt_connected or self.mqtt_client is None:
            return False

        try:
            with self.mqtt_lock:
                if hasattr(self.mqtt_client, 'client'):
                    # 使用更短超时的loop方法，避免阻塞过久
                    self.mqtt_client.client.loop(timeout=0.01)
                    return True
        except struct.error as e:
            rospy.logwarn("[MQTT Bridge] MQTT循环结构错误: %s", str(e))
            # 重置MQTT连接
            self.mqtt_connected = False
            return False
        except Exception as e:
            rospy.logwarn("[MQTT Bridge] MQTT循环异常: %s", str(e))
            self.mqtt_connected = False
            return False

        return False

    def check_and_reconnect_mqtt(self):
        """检查并重新连接MQTT - 使用指数退避策略"""
        current_time = time.time()

        # 如果已经连接，直接返回
        if self.mqtt_connected and self.mqtt_client is not None and \
                hasattr(self.mqtt_client, 'client') and \
                self.mqtt_client.client.is_connected():
            return True

        # 控制重连频率 - 指数退避
        reconnect_delay = min(MQTT_CONFIG["reconnect_min_delay"] * (2 ** self.reconnect_count),
                              MQTT_CONFIG["reconnect_max_delay"])

        if current_time - self.last_reconnect_attempt < reconnect_delay:
            return False

        rospy.logwarn("[MQTT Bridge] MQTT连接断开，尝试重连 (尝试次数: %d)...", self.reconnect_count + 1)
        self.last_reconnect_attempt = current_time
        self.reconnect_count += 1

        # 尝试重新初始化MQTT客户端
        self.init_mqtt_client()
        return self.mqtt_connected

    def server_message_callback(self, topic, message):
        """简化的服务器消息处理函数 - 仅支持十六进制指令，兼容 Python 2.7"""
        try:
            # 记录原始消息长度和类型
            msg_type = type(message).__name__
            msg_len = len(message) if hasattr(message, '__len__') else 0
            rospy.loginfo("[MQTT Bridge] 收到消息: %s, 类型=%s, 长度=%d", repr(message), msg_type, msg_len)

            # 定义特殊指令（十六进制）
            START_CMD_HEX = "01050002ff002dfa"          # 自动模式开始巡线指令
            STOP_CMD_HEX = "01050002000028ca"            # 停止巡线指令（预留）
            MANUAL_START_CMD_HEX = "01050002ff002aba"    # 手动模式开始巡线指令
            MANUAL_STOP_CMD_HEX = "01050002ff002bcb"     # 手动模式停止指令
            MANUAL_TEMP_CMD_HEX = "002ff002ccc"          # 手动采集温度指令

            # 统一转换为字节数据进行处理（兼容 Python 2.7）
            # Python 2.7 下 MQTT payload 可能是 str 类型但包含二进制数据
            byte_data = None
            if msg_type == 'bytes':
                byte_data = message
            elif msg_type == 'str':
                # Python 2.7: str 就是字节串；Python 3: 需要 encode
                if sys.version_info[0] == 2:
                    byte_data = message
                else:
                    # 尝试将字符串当作二进制数据处理
                    try:
                        byte_data = message.encode('latin-1')  # latin-1 保留原始字节
                    except:
                        byte_data = None

            # 获取消息的十六进制表示（用于指令匹配）
            hex_representation = ""
            if byte_data is not None:
                try:
                    hex_representation = binascii.hexlify(byte_data).lower()
                    # Python 3 下 hexlify 返回 bytes，需要 decode
                    if isinstance(hex_representation, bytes):
                        hex_representation = hex_representation.decode('utf-8')
                except Exception as e:
                    rospy.logwarn("[MQTT Bridge] hexlify 转换失败: %s", str(e))
                    hex_representation = ""

            rospy.loginfo("[MQTT Bridge] 消息十六进制: %s", hex_representation)

            # 检查是否为自动模式开始巡线指令
            if hex_representation == START_CMD_HEX:
                rospy.loginfo("[MQTT Bridge] 收到自动模式开始巡线指令 (0x%s)", START_CMD_HEX.upper())
                self.current_mode = "auto"
                self.handle_start_command()
                return

            # 检查是否为停止巡线指令（自动模式停止）
            if hex_representation == STOP_CMD_HEX:
                rospy.loginfo("[MQTT Bridge] 收到停止巡线指令 (0x%s)", STOP_CMD_HEX.upper())
                self.current_mode = "idle"
                control_msg = String()
                control_msg.data = "stop"
                self.control_pub.publish(control_msg)
                rospy.loginfo("[MQTT Bridge] 发送停止寻线指令")
                self.send_relay_command(RELAY_COMMANDS["8_OFF"])
                return

            # 检查是否为手动模式开始巡线指令
            if hex_representation == MANUAL_START_CMD_HEX:
                rospy.loginfo("[MQTT Bridge] 收到手动模式开始巡线指令 (0x%s)", MANUAL_START_CMD_HEX.upper())
                self.current_mode = "manual"
                self.handle_manual_start_command()
                return

            # 检查是否为手动模式停止指令
            if hex_representation == MANUAL_STOP_CMD_HEX:
                rospy.loginfo("[MQTT Bridge] 收到手动模式停止指令 (0x%s)", MANUAL_STOP_CMD_HEX.upper())
                self.current_mode = "idle"
                self.handle_manual_stop_command()
                return

            # 检查是否为手动采集温度指令（注意：此指令较短，使用 endswith 匹配）
            if hex_representation == MANUAL_TEMP_CMD_HEX or hex_representation.endswith(MANUAL_TEMP_CMD_HEX):
                rospy.loginfo("[MQTT Bridge] 收到手动采集温度指令 (0x%s)", MANUAL_TEMP_CMD_HEX.upper())
                self.handle_manual_temperature()
                return

            # 处理其他字节数据（仅手动模式下转发到继电器）
            if byte_data is not None and len(byte_data) > 0:
                if self.current_mode == "manual":
                    self.send_relay_command(byte_data)
                    rospy.loginfo("[MQTT Bridge] [手动模式] 继电器指令已发送: %s", hex_representation)
                else:
                    rospy.loginfo("[MQTT Bridge] [%s模式] 忽略继电器指令: %s（仅手动模式可控制）",
                                 self.current_mode, hex_representation)
                return

            rospy.logwarn("[MQTT Bridge] 收到无法处理的消息")

        except Exception as e:
            rospy.logerr("[MQTT Bridge] 处理服务器消息错误: %s", str(e))
            rospy.logdebug(traceback.format_exc())

    def get_temperature(self, max_retries=2, timeout_per_try=30.0):
        """
        获取热成像仪数据 - 带重试机制的 Python 2.7 兼容版本
        
        Args:
            max_retries: 最大重试次数（默认2次，即总共最多尝试3次）
            timeout_per_try: 每次尝试的超时时间（默认30秒）
        
        Returns:
            成功返回温度数据字符串，失败返回 0
        """
        script_path = '/home/wheeltec/wheeltec_robot/src/USB_sample/build/thermal_data.py'
        command = ['python3', script_path]
        
        for attempt in range(max_retries + 1):
            try:
                if attempt > 0:
                    rospy.logwarn("[MQTT Bridge] 热成像数据获取重试 (%d/%d)...", attempt, max_retries)
                    # 重试前等待一小段时间，让设备恢复
                    time.sleep(2.0)
                
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                time.sleep(18.0)
                # Python 2.7 不支持 communicate(timeout=)，使用线程实现超时
                result = [None, None]  # [stdout, stderr]
                communicate_error = [None]  # 用于捕获线程内异常

                def communicate_thread():
                    try:
                        result[0], result[1] = process.communicate()
                    except Exception as e:
                        communicate_error[0] = e

                thread = threading.Thread(target=communicate_thread)
                thread.daemon = True
                thread.start()
                thread.join(timeout=timeout_per_try)

                if thread.is_alive():
                    # 超时，终止进程
                    rospy.logwarn("[MQTT Bridge] 热成像数据获取超时 (尝试 %d/%d)", attempt + 1, max_retries + 1)
                    try:
                        process.kill()
                        process.wait()
                    except:
                        pass
                    # 继续重试
                    continue

                # 检查线程内是否有异常
                if communicate_error[0] is not None:
                    rospy.logerr("[MQTT Bridge] communicate 线程异常: %s", str(communicate_error[0]))
                    continue

                stdout, stderr = result[0], result[1]

                if stdout is None:
                    rospy.logerr("[MQTT Bridge] 热成像数据获取失败: communicate 未返回数据")
                    continue

                if process.returncode == 0:
                    temp_out = stdout.decode('utf-8').strip()
                    if len(temp_out) > 0:
                        rospy.loginfo("[MQTT Bridge] Successfully got temperature size: %d, content: %s"
                                      % (len(temp_out), (temp_out[:100] + "[...]") if len(temp_out) > 100 else temp_out))
                        return temp_out
                    else:
                        rospy.logwarn("[MQTT Bridge] 热成像脚本返回空数据")
                        continue
                else:
                    error_msg = stderr.decode('utf-8') if stderr else "unknown error"
                    rospy.logerr("[MQTT Bridge] 热成像脚本执行失败: %s", error_msg)
                    continue

            except Exception as e:
                rospy.logerr(u"[MQTT Bridge] 热成像数据获取异常 (尝试 %d/%d): %s", attempt + 1, max_retries + 1, str(e))
                continue
        
        # 所有重试都失败
        rospy.logerr("[MQTT Bridge] 热成像数据获取失败，已重试 %d 次", max_retries)
        return 0

    def send_temperature_data(self, temperature_data, index=0):
        """发送温度数据到服务器"""
        if not self.check_and_reconnect_mqtt():
            rospy.logwarn("[MQTT Bridge] MQTT客户端未连接")
            return False

        try:
            # 更新当前温度
            self.current_temperature = temperature_data

            # 构建MQTT消息
            message_data = {
                "timestamp": datetime.now().isoformat(),
                "device_id": MQTT_CONFIG["client_id"],
                "temperature": temperature_data,
                "data_type": "temperature_reading",
                "sequence_index": index
            }

            message_json = json.dumps(message_data)

            if self.mqtt_client is not None:
                self.mqtt_client.publish(MQTT_CONFIG["pub_topic"], message_json, qos=1)
                rospy.loginfo("[MQTT Bridge] (序号: %s)温度数据发送成功", str(index))
                return True
            else:
                rospy.logwarn("[MQTT Bridge] (序号: %s)温度数据发送失败，mqtt_client为None", str(index))
                return False

        except Exception as e:
            rospy.logerr("[MQTT Bridge] (序号: %s)温度数据发送失败: %s", str(index), str(e))
            self.mqtt_connected = False
            return False

    def check_temperature_and_send(self, index=0):
        """检测温度并发送到服务器"""
        rospy.loginfo("[MQTT Bridge] (序号: %s)温度读数获取...", str(index))
        temperature_data = self.get_temperature()

        if temperature_data is not None:
            # 发送温度数据到服务器，携带序号
            success = self.send_temperature_data(temperature_data, index=index)
            if success:
                rospy.loginfo("[MQTT Bridge] (序号: %s)温度数据发送到服务器成功", str(index))
            else:
                rospy.logwarn("[MQTT Bridge] (序号: %s)温度数据发送到服务器失败", str(index))
        else:
            rospy.logwarn("[MQTT Bridge] (序号: %s)获取温度数据失败", str(index))

    def send_relay_command(self, command):
        """发送单个继电器控制指令"""
        if self.serial_port is None:
            rospy.logwarn("[MQTT Bridge] 串口未初始化")
            return

        try:
            with self.serial_lock:
                if not self.serial_port.isOpen():
                    self.serial_port.open()

                if self.serial_port.isOpen():
                    # 直接发送字节数据
                    self.serial_port.write(command)
                    # 使用binascii.hexlify进行Python 2.7兼容的十六进制显示
                    hex_representation = binascii.hexlify(command)
                    rospy.loginfo("[MQTT Bridge] 继电器指令已发送: %s", hex_representation)

        except Exception as e:
            rospy.logerr("[MQTT Bridge] 发送继电器指令失败: %s", str(e))

    def is_line_follower_running(self):
        """检查巡线节点是否在运行"""
        try:
            # 方式1：通过 rosnode list 检查关键节点
            result = subprocess.Popen(
                ['rosnode', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            # Python 2.7 兼容：使用轮询方式实现超时
            timeout = 5.0
            poll_interval = 0.1
            elapsed = 0.0
            while elapsed < timeout:
                if result.poll() is not None:
                    break
                time.sleep(poll_interval)
                elapsed += poll_interval
            
            if result.poll() is None:
                # 超时，终止进程
                result.terminate()
                rospy.logwarn("[MQTT Bridge] 检查巡线节点超时")
                return self.line_follower_running
            
            stdout, _ = result.communicate()
            nodes = stdout.decode('utf-8').strip().split('\n')
            
            # 检查巡线核心节点是否存在
            line_tracker_exists = '/line_tracker' in nodes
            rospy.loginfo("[MQTT Bridge] 巡线节点 /line_tracker 存在: %s", line_tracker_exists)
            return line_tracker_exists
            
        except Exception as e:
            rospy.logwarn("[MQTT Bridge] 检查巡线节点异常: %s", str(e))
            return self.line_follower_running  # 返回缓存状态

    def is_recharge_running(self):
        """检查回充流程是否在运行"""
        with self.recharge_lock:
            if not self.recharge_triggered:
                return False
        
        # 检查回充子进程是否还在运行
        if self.recharge_proc is not None and self.recharge_proc.poll() is None:
            return True
        
        # 也可以通过检查回充相关节点来判断
        try:
            result = subprocess.Popen(
                ['rosnode', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            # Python 2.7 兼容：使用轮询方式实现超时
            timeout = 5.0
            poll_interval = 0.1
            elapsed = 0.0
            while elapsed < timeout:
                if result.poll() is not None:
                    break
                time.sleep(poll_interval)
                elapsed += poll_interval
            
            if result.poll() is None:
                result.terminate()
                return self.recharge_triggered
            
            stdout, _ = result.communicate()
            nodes = stdout.decode('utf-8').strip().split('\n')
            
            # 检查回充/导航相关节点
            recharge_nodes = ['/move_base', '/amcl', '/auto_recharger']
            for node in recharge_nodes:
                if node in nodes:
                    rospy.loginfo("[MQTT Bridge] 发现回充相关节点: %s", node)
                    return True
            return False
            
        except Exception as e:
            rospy.logwarn("[MQTT Bridge] 检查回充节点异常: %s", str(e))
            return self.recharge_triggered

    def start_line_follower_launch(self):
        """启动巡线节点"""
        try:
            rospy.loginfo("[MQTT Bridge] ========== 开始启动巡线节点 ==========")
            
            # 注意：不在这里删除回充模式标志文件！
            # 标志文件的删除移到节点完全启动并写入 PID 文件之后
            # 这样可以防止 start_line.sh 在节点启动过程中检测到"节点挂了"而重复启动
            
            # 使用 roslaunch 启动巡线节点
            launch_proc = subprocess.Popen(
                ['roslaunch', 'simple_follower', 'line_follower.launch'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # 保存 PID 到文件（必须在删除标志文件之前完成）
            pid_file = '/tmp/line_follower.pid'
            with open(pid_file, 'w') as f:
                f.write(str(launch_proc.pid))
            
            rospy.loginfo("[MQTT Bridge] 巡线节点启动成功，PID: %d", launch_proc.pid)
            
            # 等待节点初始化
            time.sleep(5)
            
            # 更新状态
            self.line_follower_running = True
            
            # 现在才删除回充模式标志文件（节点已启动且 PID 已写入）
            # 这样 start_line.sh 的监控循环会使用新的 PID 来检查进程状态
            try:
                if os.path.exists('/tmp/recharge_mode.flag'):
                    os.remove('/tmp/recharge_mode.flag')
                    rospy.loginfo("[MQTT Bridge] 已删除回充模式标志文件")
            except Exception as e:
                rospy.logwarn("[MQTT Bridge] 删除回充模式标志文件失败: %s", str(e))
            
            rospy.loginfo("[MQTT Bridge] ========== 巡线节点启动完成 ==========")
            return True
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 启动巡线节点异常: %s", str(e))
            return False

    def stop_recharge_nodes(self):
        """停止回充相关节点"""
        try:
            rospy.loginfo("[MQTT Bridge] ========== 开始停止回充节点 ==========")
            
            # 终止回充子进程
            if self.recharge_proc is not None and self.recharge_proc.poll() is None:
                try:
                    rospy.loginfo("[MQTT Bridge] 停止回充子进程 PID: %d", self.recharge_proc.pid)
                    os.killpg(os.getpgid(self.recharge_proc.pid), signal.SIGTERM)
                    time.sleep(2)
                    if self.recharge_proc.poll() is None:
                        os.killpg(os.getpgid(self.recharge_proc.pid), signal.SIGKILL)
                except Exception as e:
                    rospy.logwarn("[MQTT Bridge] 停止回充子进程异常: %s", str(e))
            
            self.recharge_proc = None
            
            # 停止回充/导航相关节点
            recharge_nodes_to_kill = [
                '/move_base',
                '/amcl',
                '/auto_recharger',
                '/map_server',
                '/launch_looper'
            ]
            
            for node in recharge_nodes_to_kill:
                try:
                    # Python 2.7 兼容：使用 Popen + 轮询代替 call(timeout=)
                    proc = subprocess.Popen(
                        ['rosnode', 'kill', node],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    timeout = 3.0
                    elapsed = 0.0
                    while elapsed < timeout:
                        if proc.poll() is not None:
                            break
                        time.sleep(0.1)
                        elapsed += 0.1
                    if proc.poll() is None:
                        proc.terminate()
                    rospy.loginfo("[MQTT Bridge] 已停止回充节点: %s", node)
                except:
                    pass
            
            # 重置回充状态
            with self.recharge_lock:
                self.recharge_triggered = False
            
            # 注意：不在这里删除回充模式标志文件！
            # 标志文件的删除移到 start_line_follower_launch() 中，
            # 在巡线节点完全启动并更新 PID 文件之后再删除。
            # 这样可以防止 start_line.sh 在切换过程中检测到"节点挂了"而重复启动。
            
            # 等待节点完全停止
            time.sleep(3)
            
            rospy.loginfo("[MQTT Bridge] ========== 回充节点停止完成 ==========")
            return True
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 停止回充节点异常: %s", str(e))
            return False

    def handle_manual_start_command(self):
        """处理手动模式 start 指令：启动巡线节点但发送 manual_start（不触发黄线动作）"""
        rospy.loginfo("[MQTT Bridge] ========== 处理手动模式 start 指令 ==========")

        # 检查巡线节点是否在运行
        if self.is_line_follower_running():
            rospy.loginfo("[MQTT Bridge] 巡线节点已运行，直接发送 manual_start 指令")
            self._send_manual_start_and_light_on()

            # 重试机制
            def verify_and_retry():
                time.sleep(2)
                if not self.is_line_follower_running():
                    rospy.logwarn("[MQTT Bridge] 巡线节点在发送manual_start后消失，重新启动...")
                    self._switch_to_manual_follow_mode(stop_recharge_first=False)

            verify_thread = threading.Thread(target=verify_and_retry)
            verify_thread.daemon = True
            verify_thread.start()
        else:
            rospy.loginfo("[MQTT Bridge] 巡线节点不存在，检查回充状态...")
            need_stop_recharge = self.is_recharge_running()
            thread = threading.Thread(target=self._switch_to_manual_follow_mode, args=(need_stop_recharge,))
            thread.daemon = True
            thread.start()

        rospy.loginfo("[MQTT Bridge] ========== 手动模式 start 指令处理完成 ==========")

    def handle_manual_stop_command(self):
        """处理手动模式停止指令：仅暂停运行，不关灯"""
        rospy.loginfo("[MQTT Bridge] ========== 处理手动模式停止指令 ==========")
        control_msg = String()
        control_msg.data = "manual_stop"
        self.control_pub.publish(control_msg)
        rospy.loginfo("[MQTT Bridge] 已发送 manual_stop 指令（保持灯光）")

    def handle_manual_temperature(self):
        """处理手动采集温度指令：采集一次温度并上报"""
        rospy.loginfo("[MQTT Bridge] ========== 处理手动采集温度指令 ==========")

        def _do_temperature():
            rospy.loginfo("[MQTT Bridge] 手动模式：开始采集温度...")
            self.check_temperature_and_send(index=0)
            rospy.loginfo("[MQTT Bridge] 手动模式：温度采集完成")

        thread = threading.Thread(target=_do_temperature)
        thread.daemon = True
        thread.start()

    def _send_manual_start_and_light_on(self):
        """发送 manual_start 指令并开灯"""
        control_msg = String()
        control_msg.data = "manual_start"
        self.control_pub.publish(control_msg)
        self.send_relay_command(RELAY_COMMANDS["8_ON"])
        rospy.loginfo("[MQTT Bridge] 已发送 manual_start 指令并开灯")

    def _switch_to_manual_follow_mode(self, stop_recharge_first=False):
        """切换到手动巡线模式：[停止回充] -> 启动巡线 -> 发送 manual_start"""
        try:
            rospy.loginfo("[MQTT Bridge] 开始切换到手动巡线模式 (需停止回充: %s)...", stop_recharge_first)

            if stop_recharge_first:
                if not self.stop_recharge_nodes():
                    rospy.logerr("[MQTT Bridge] 停止回充失败，但继续尝试启动巡线")

            if not self.start_line_follower_launch():
                rospy.logerr("[MQTT Bridge] 启动巡线节点失败")
                return

            rospy.loginfo("[MQTT Bridge] 等待巡线节点订阅者就绪...")
            max_wait = 30
            for i in range(max_wait):
                subscriber_count = self.control_pub.get_num_connections()
                rospy.loginfo("[MQTT Bridge] /line_follower/control 订阅者数量: %d (%d/%d秒)",
                             subscriber_count, i + 1, max_wait)
                if subscriber_count > 0:
                    rospy.loginfo("[MQTT Bridge] 订阅者已就绪!")
                    break
                time.sleep(1)
            else:
                rospy.logwarn("[MQTT Bridge] 等待订阅者超时，仍尝试发送 manual_start 指令")

            time.sleep(0.5)
            self._send_manual_start_and_light_on()
            rospy.loginfo("[MQTT Bridge] 切换到手动巡线模式完成")

        except Exception as e:
            rospy.logerr("[MQTT Bridge] 切换到手动巡线模式异常: %s", str(e))

    def handle_start_command(self):
        """处理 start 指令：智能判断并切换模式，带重试机制"""
        rospy.loginfo("[MQTT Bridge] ========== 处理 start 指令 ==========")
        
        # 检查巡线节点是否在运行
        if self.is_line_follower_running():
            # 巡线节点存在，直接发送 start 指令
            rospy.loginfo("[MQTT Bridge] 巡线节点已运行，直接发送 start 指令")
            self._send_start_and_light_on()
            
            # 重试机制：延迟后再次检查节点是否还在运行
            # 防止节点因重名冲突被关闭导致 start 指令丢失
            def verify_and_retry():
                time.sleep(2)
                if not self.is_line_follower_running():
                    rospy.logwarn("[MQTT Bridge] 巡线节点在发送start后消失，重新启动...")
                    self._switch_to_line_follow_mode(stop_recharge_first=False)
            
            verify_thread = threading.Thread(target=verify_and_retry)
            verify_thread.daemon = True
            verify_thread.start()
        else:
            # 巡线节点不存在，需要先处理
            rospy.loginfo("[MQTT Bridge] 巡线节点不存在，检查回充状态...")
            need_stop_recharge = self.is_recharge_running()
            # 在新线程中执行，避免阻塞
            thread = threading.Thread(target=self._switch_to_line_follow_mode, args=(need_stop_recharge,))
            thread.daemon = True
            thread.start()
        
        rospy.loginfo("[MQTT Bridge] ========== start 指令处理完成 ==========")

    def _send_start_and_light_on(self):
        """发送 start 指令并开灯"""
        control_msg = String()
        control_msg.data = "start"
        self.control_pub.publish(control_msg)
        self.send_relay_command(RELAY_COMMANDS["8_ON"])
        rospy.loginfo("[MQTT Bridge] 已发送 start 指令并开灯")

    def _switch_to_line_follow_mode(self, stop_recharge_first=False):
        """切换到巡线模式：[停止回充] -> 启动巡线 -> 发送 start"""
        try:
            rospy.loginfo("[MQTT Bridge] 开始切换到巡线模式 (需停止回充: %s)...", stop_recharge_first)
            
            # 步骤1：停止回充（如果需要）
            if stop_recharge_first:
                if not self.stop_recharge_nodes():
                    rospy.logerr("[MQTT Bridge] 停止回充失败，但继续尝试启动巡线")
            
            # 步骤2：启动巡线节点
            if not self.start_line_follower_launch():
                rospy.logerr("[MQTT Bridge] 启动巡线节点失败")
                return
            
            # 步骤3：等待订阅者真正就绪（检查 topic 连接数）
            rospy.loginfo("[MQTT Bridge] 等待巡线节点订阅者就绪...")
            max_wait = 30  # 最多等待30秒
            for i in range(max_wait):
                subscriber_count = self.control_pub.get_num_connections()
                rospy.loginfo("[MQTT Bridge] /line_follower/control 订阅者数量: %d (%d/%d秒)", 
                             subscriber_count, i + 1, max_wait)
                if subscriber_count > 0:
                    rospy.loginfo("[MQTT Bridge] 订阅者已就绪!")
                    break
                time.sleep(1)
            else:
                rospy.logwarn("[MQTT Bridge] 等待订阅者超时，仍尝试发送 start 指令")
            
            # 步骤4：发送 start 指令
            time.sleep(0.5)  # 额外等待确保连接稳定
            self._send_start_and_light_on()
            rospy.loginfo("[MQTT Bridge] 切换到巡线模式完成")
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 切换到巡线模式异常: %s", str(e))

    def execute_relay_sequence(self):
        """执行复杂的继电器控制序列"""
        # 检查是否已有序列在执行
        if self.relay_sequence_running:
            rospy.logwarn("[MQTT Bridge] 继电器序列已在执行中")
            return
        self.relay_sequence_running = True

        try:
            rospy.loginfo("[MQTT Bridge] 开始执行继电器控制序列")

            # ========== 第一次获取温度和发送 ==========
            self.check_temperature_and_send(index=1)
            # =========================================
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(3)

            # 第三路关闭
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(1)

            # 第三路开启
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(1)

            # 第三路关闭
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(2)

            # ========== 第二次获取温度和发送 ==========
            self.check_temperature_and_send(index=2)
            # =========================================

            # 第四路开启
            self.send_relay_command(RELAY_COMMANDS["4_ON"])
            time.sleep(1)

            # 第四路关闭再开启（重复3次）
            for _ in range(3):
                self.send_relay_command(RELAY_COMMANDS["4_OFF"])
                time.sleep(1)
                self.send_relay_command(RELAY_COMMANDS["4_ON"])
                time.sleep(1)

            # 第四路关闭
            self.send_relay_command(RELAY_COMMANDS["4_OFF"])
            time.sleep(1)
            # ========== 第三次获取温度和发送 ==========
            self.check_temperature_and_send(index=3)
            # =========================================

            # 第三路开启
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(2)


            # 第三路关闭再开启
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(2)
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(2)

            # 第三路关闭后再开启后关闭
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(1)

            # 第一路开启--升杆
            self.send_relay_command(RELAY_COMMANDS["1_ON"])
            time.sleep(10)

            # 第一路关闭
            self.send_relay_command(RELAY_COMMANDS["1_OFF"])
            time.sleep(1)
           
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(1)

            # 第三路关闭
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(1)

            # 第三路开启
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(1)

            # 第三路关闭
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(2)

            # 第四路开启
            self.send_relay_command(RELAY_COMMANDS["4_ON"])
            time.sleep(1)

            # 第四路关闭再开启（重复3次）
            for _ in range(3):
                self.send_relay_command(RELAY_COMMANDS["4_OFF"])
                time.sleep(1)
                self.send_relay_command(RELAY_COMMANDS["4_ON"])
                time.sleep(1)

            # 第四路关闭
            self.send_relay_command(RELAY_COMMANDS["4_OFF"])
            time.sleep(1)

            # 第三路开启
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(2)

            # 第三路关闭再开启
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(2)
            self.send_relay_command(RELAY_COMMANDS["3_ON"])
            time.sleep(2)

            # 第三路关闭后再开启后关闭
            self.send_relay_command(RELAY_COMMANDS["3_OFF"])
            time.sleep(1)

            # 第二路开启
            self.send_relay_command(RELAY_COMMANDS["2_ON"])
            time.sleep(10)

            # 第二路关闭
            self.send_relay_command(RELAY_COMMANDS["2_OFF"])
            rospy.loginfo("[MQTT Bridge] 继电器控制序列执行完成")

        except Exception as e:
            rospy.logerr("[MQTT Bridge] 执行继电器序列失败: %s", str(e))
        finally:
            self.relay_sequence_running = False
            # 无论成功还是失败，都通知line_follow继电器序列已结束
            try:
                control_msg = String()
                control_msg.data = "relay_sequence_finish"
                self.control_pub.publish(control_msg)
                rospy.loginfo("[MQTT Bridge] 已发布继电器序列完成通知")
            except Exception as pub_error:
                rospy.logerr("[MQTT Bridge] 发布继电器完成通知时出错: %s", str(pub_error))

    def relay_sequence_handler(self, msg):
        """处理继电器序列触发信号"""
        try:
            rospy.loginfo("[MQTT Bridge] 收到继电器序列触发信号: %s, 消息类型: %s", msg.data, type(msg.data).__name__)

            # 检查消息内容
            if msg.data == "trigger":
                rospy.loginfo("[MQTT Bridge] 继电器触发消息验证通过，启动执行线程")
                # 在新线程中执行继电器序列，避免阻塞主线程
                thread = threading.Thread(target=self.execute_relay_sequence)
                thread.daemon = True
                thread.start()
                rospy.loginfo("[MQTT Bridge] 继电器执行线程已启动")
            elif msg.data == "follow_end":
                rospy.loginfo("[MQTT Bridge] 收到 follow_end 消息，关闭灯光并启动回充流程")
                self.current_mode = "idle"
                self.send_relay_command(RELAY_COMMANDS["8_OFF"])
                # 启动回充流程（在新线程中执行，避免阻塞）
                thread = threading.Thread(target=self.start_recharge)
                thread.daemon = True
                thread.start()
            else:
                rospy.logwarn("[MQTT Bridge] 收到未知的继电器触发消息: %s", msg.data)

        except Exception as e:
            rospy.logerr("[MQTT Bridge] 处理继电器触发信号异常: %s", str(e))
            rospy.logerr("[MQTT Bridge] 异常详情: %s", traceback.format_exc())

    def _check_nodes_exist(self, node_list):
        """检查指定节点是否还在运行"""
        try:
            result = subprocess.Popen(
                ['rosnode', 'list'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            # Python 2.7 兼容：使用轮询代替 communicate(timeout=)
            timeout = 5.0
            elapsed = 0.0
            while elapsed < timeout:
                if result.poll() is not None:
                    break
                time.sleep(0.1)
                elapsed += 0.1
            
            if result.poll() is None:
                result.terminate()
                return False
            
            stdout, _ = result.communicate()
            running_nodes = stdout.decode('utf-8').strip().split('\n')
            
            for node in node_list:
                if node in running_nodes:
                    rospy.logdebug("[MQTT Bridge] 节点仍在运行: %s", node)
                    return True
            return False
        except Exception as e:
            rospy.logwarn("[MQTT Bridge] 检查节点状态异常: %s", str(e))
            return False

    def stop_line_follower_launch(self):
        """停止寻线节点，为回充腾出资源"""
        try:
            rospy.loginfo("[MQTT Bridge] ========== 开始停止寻线节点 ==========")
            
            # 更新状态
            self.line_follower_running = False
            
            # 方式1：通过 rosnode kill 停止相关节点
            nodes_to_kill = [
                '/line_tracker',
                '/avoidance',
                '/camera_nodelet_manager',
                '/wheeltec_camera',
                '/wheeltec_robot',
                '/base_to_link',
                '/base_to_laser', 
                '/base_to_camera',
                '/base_to_gyro',
                '/robot_pose_ekf',
                '/robot_state_publisher',
                '/joint_state_publisher'
            ]
            
            for node in nodes_to_kill:
                try:
                    # Python 2.7 兼容：使用 Popen + 轮询代替 call(timeout=)
                    proc = subprocess.Popen(['rosnode', 'kill', node], 
                                           stdout=subprocess.PIPE, 
                                           stderr=subprocess.PIPE)
                    timeout = 3.0
                    elapsed = 0.0
                    while elapsed < timeout:
                        if proc.poll() is not None:
                            break
                        time.sleep(0.1)
                        elapsed += 0.1
                    
                    if proc.poll() is None:
                        proc.terminate()
                        rospy.logwarn("[MQTT Bridge] 停止节点超时: %s", node)
                    elif proc.returncode == 0:
                        rospy.loginfo("[MQTT Bridge] 成功停止节点: %s", node)
                except Exception as e:
                    rospy.logdebug("[MQTT Bridge] 停止节点 %s 时出错: %s", node, str(e))
            
            # 方式2：通过 PID 文件终止 launch 进程
            pid_file = '/tmp/line_follower.pid'
            if os.path.exists(pid_file):
                try:
                    with open(pid_file, 'r') as f:
                        pid = int(f.read().strip())
                    rospy.loginfo("[MQTT Bridge] 尝试终止寻线 launch 进程 PID: %d", pid)
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(2)
                    # 检查进程是否还存在
                    try:
                        os.kill(pid, 0)
                        # 进程还在，强制终止
                        os.kill(pid, signal.SIGKILL)
                        rospy.loginfo("[MQTT Bridge] 强制终止寻线 launch 进程")
                    except OSError:
                        rospy.loginfo("[MQTT Bridge] 寻线 launch 进程已终止")
                except Exception as e:
                    rospy.logwarn("[MQTT Bridge] 通过 PID 文件终止进程失败: %s", str(e))
            
            # 等待并验证关键节点已停止
            rospy.loginfo("[MQTT Bridge] 等待节点完全停止...")
            critical_nodes = ['/wheeltec_robot', '/robot_pose_ekf', '/robot_state_publisher']
            max_wait = 10  # 最多等待10秒
            for i in range(max_wait):
                time.sleep(1)
                still_running = self._check_nodes_exist(critical_nodes)
                if not still_running:
                    rospy.loginfo("[MQTT Bridge] 所有关键节点已停止")
                    break
                rospy.loginfo("[MQTT Bridge] 等待关键节点停止... (%d/%d)", i + 1, max_wait)
            else:
                rospy.logwarn("[MQTT Bridge] 部分关键节点可能未完全停止，继续执行")
            
            rospy.loginfo("[MQTT Bridge] ========== 寻线节点停止完成 ==========")
            return True
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 停止寻线节点异常: %s", str(e))
            return False

    def _launch_recharge_stack(self):
        """在独立线程中启动回充脚本"""
        try:
            script_path = '/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/line_auto_recharger_v2.py'
            if not os.path.isfile(script_path):
                rospy.logerr("[MQTT Bridge] 未找到回充脚本: %s", script_path)
                return
            
            rospy.loginfo("[MQTT Bridge] 以子进程启动回充脚本: %s", script_path)
            
            # 将输出重定向到日志文件，避免 PIPE 缓冲区满导致进程阻塞
            log_file_path = '/tmp/recharge_script.log'
            log_file = open(log_file_path, 'w')
            
            # 使用 python 解释器启动，独立进程组便于优雅终止
            self.recharge_proc = subprocess.Popen(
                ['python', script_path],
                stdout=log_file,
                stderr=log_file,
                preexec_fn=os.setsid
            )
            rospy.loginfo("[MQTT Bridge] 回充脚本进程 PID: %s, 日志文件: %s", str(self.recharge_proc.pid), log_file_path)
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 启动回充脚本失败: %s", str(e))

    def start_recharge(self):
        """启动回充流程：先停止寻线节点，再启动回充脚本"""
        with self.recharge_lock:
            if self.recharge_triggered:
                rospy.logwarn("[MQTT Bridge] 回充流程已在进行中，忽略重复触发")
                return
            self.recharge_triggered = True
        
        try:
            rospy.loginfo("[MQTT Bridge] ========== 触发自动回充流程 ==========")
            
            # 创建回充模式标志文件，防止 start_line.sh 自动重启巡线节点
            try:
                with open('/tmp/recharge_mode.flag', 'w') as f:
                    f.write(str(time.time()))
                rospy.loginfo("[MQTT Bridge] 已创建回充模式标志文件")
            except Exception as e:
                rospy.logwarn("[MQTT Bridge] 创建回充模式标志文件失败: %s", str(e))
            
            # 步骤1：停止寻线节点
            rospy.loginfo("[MQTT Bridge] 步骤1: 停止寻线节点...")
            if not self.stop_line_follower_launch():
                rospy.logerr("[MQTT Bridge] 停止寻线节点失败，但继续尝试启动回充")
            
            # 步骤2：启动回充脚本
            rospy.loginfo("[MQTT Bridge] 步骤2: 启动回充脚本...")
            self._launch_recharge_stack()
            
            rospy.loginfo("[MQTT Bridge] ========== 回充流程启动完成 ==========")
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 启动回充流程失败: %s", str(e))
            with self.recharge_lock:
                self.recharge_triggered = False

    def stop_recharge(self):
        """停止回充流程"""
        try:
            with self.recharge_lock:
                if not self.recharge_triggered:
                    rospy.loginfo("[MQTT Bridge] 没有运行中的回充流程")
                    return
            
            rospy.loginfo("[MQTT Bridge] ========== 开始停止回充流程 ==========")
            
            # 终止回充子进程
            if self.recharge_proc is not None and self.recharge_proc.poll() is None:
                try:
                    rospy.loginfo("[MQTT Bridge] 停止回充子进程 PID: %d", self.recharge_proc.pid)
                    os.killpg(os.getpgid(self.recharge_proc.pid), signal.SIGTERM)
                    # Python 2.7 兼容：使用轮询代替 wait(timeout=)
                    timeout = 5.0
                    elapsed = 0.0
                    while elapsed < timeout:
                        if self.recharge_proc.poll() is not None:
                            break
                        time.sleep(0.1)
                        elapsed += 0.1
                    
                    if self.recharge_proc.poll() is None:
                        rospy.logwarn("[MQTT Bridge] 回充子进程未响应，强制终止")
                        os.killpg(os.getpgid(self.recharge_proc.pid), signal.SIGKILL)
                except Exception as e:
                    rospy.logwarn("[MQTT Bridge] 停止回充子进程异常: %s", str(e))
            
            self.recharge_proc = None
            
            with self.recharge_lock:
                self.recharge_triggered = False
            
            rospy.loginfo("[MQTT Bridge] ========== 回充流程停止完成 ==========")
            
        except Exception as e:
            rospy.logerr("[MQTT Bridge] 停止回充流程异常: %s", str(e))
            with self.recharge_lock:
                self.recharge_triggered = False

    def run(self):
        """主循环 - 使用更安全的循环方式"""
        rate = rospy.Rate(10)  # 10Hz
        error_count = 0
        max_errors = 5  # 最大连续错误次数

        while not rospy.is_shutdown():
            try:
                # 检查并维持MQTT连接
                self.check_and_reconnect_mqtt()

                # 安全处理MQTT网络循环
                if not self.safe_mqtt_loop():
                    error_count += 1
                    if error_count >= max_errors:
                        rospy.logwarn("[MQTT Bridge] 连续错误过多，重置MQTT连接")
                        self.mqtt_connected = False
                        error_count = 0
                else:
                    error_count = 0  # 成功执行则重置错误计数

            except struct.error as e:
                rospy.logerr("[MQTT Bridge] 结构错误: %s", str(e))
                self.mqtt_connected = False
                error_count += 1
            except Exception as e:
                rospy.logerr("[MQTT Bridge] 主循环异常: %s", str(e))
                error_count += 1
                # 短暂的休眠避免频繁错误
                time.sleep(0.1)

            # 如果错误过多，短暂休眠
            if error_count >= max_errors:
                rospy.logwarn("[MQTT Bridge] 错误过多，短暂休眠")
                time.sleep(1.0)
                error_count = 0

            rate.sleep()

        # ROS关闭时清理资源
        if self.mqtt_client is not None:
            try:
                with self.mqtt_lock:
                    if hasattr(self.mqtt_client, 'disconnect'):
                        self.mqtt_client.disconnect()
            except:
                pass


if __name__ == '__main__':
    try:
        bridge = MQTTBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[MQTT Bridge] 节点关闭")
    except Exception as e:
        rospy.logerr("[MQTT Bridge] 未处理的异常: %s", str(e))
