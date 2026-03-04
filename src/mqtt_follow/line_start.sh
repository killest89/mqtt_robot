#!/bin/bash

# =============================================
# 自动寻线机器人启动脚本
# 修复rosrun命令未找到的问题
# =============================================

# 设置日志目录
LOG_DIR="/home/wheeltec/wheeltec_robot/logs"
mkdir -p $LOG_DIR

# 日志函数
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [line_start.sh] $1" >> "$LOG_DIR/startup.log"
}

log_message ""
log_message "********************************"
log_message "INFO: 开始执行启动脚本"

# 关键修复：显式加载ROS环境
if [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
    log_message "INFO: 已加载ROS Melodic环境"
else
    log_message "ERROR: 未找到ROS Melodic环境文件"
    exit 1
fi

# 加载工作空间环境
if [ -f "/home/wheeltec/wheeltec_robot/devel/setup.bash" ]; then
    source /home/wheeltec/wheeltec_robot/devel/setup.bash
    log_message "INFO: 已加载工作空间环境"
fi

log_message "INFO: ROS环境验证成功"

# 网络连接检查
MQTT_BROKER="175.178.8.235"
log_message "INFO: 检查网络连接到MQTT代理服务器 $MQTT_BROKER"

for i in {1..10}; do
    if ping -c 1 -W 1 "$MQTT_BROKER" &> /dev/null; then
        log_message "INFO: 网络连接成功 ($i 次尝试)"
        break
    fi
    if [ $i -eq 10 ]; then
        log_message "WARN: 网络连接检查失败，但继续启动流程"
    fi
    sleep 1
done

# 启动MQTT控制器
log_message "INFO: 启动MQTT控制器..."
roslaunch simple_follower mqtt_init.launch >> "$LOG_DIR/mqtt_controller.log" 2>&1 &

CONTROLLER_PID=$!
log_message "INFO: MQTT控制器进程ID: $CONTROLLER_PID"

# 验证进程是否启动成功
sleep 3
if ps -p $CONTROLLER_PID > /dev/null; then
    echo $CONTROLLER_PID > /tmp/mqtt_controller.pid
    log_message "INFO: MQTT控制器启动成功，PID已保存"
else
    log_message "ERROR: MQTT控制器启动失败"
    echo "错误: MQTT控制器启动失败，请检查日志: $LOG_DIR/mqtt_controller.log"
    exit 1
fi

log_message "INFO: 启动脚本执行完成"
echo "系统启动成功！"
echo "MQTT控制器运行中(PID: $CONTROLLER_PID)"
echo "发送 'start' 指令到MQTT主题开始寻线"
echo "日志文件: $LOG_DIR/mqtt_controller.log"