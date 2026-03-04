#!/bin/bash
# start_line.sh - 机器人自启动脚本（增强稳定性版）

echo "=========================================="
echo "   Wheeltec机器人系统启动脚本"
echo "   启动时间: $(date)"
echo "=========================================="

# 终止mqtt进程
if pgrep -f "mqtt_bridge" > /dev/null; then
    echo "警告: 仍有MQTT进程残留，强制终止..."
    pkill -9 -f "mqtt_bridge"
    sleep 2  # 等待进程终止
fi
# 清理临时文件
rm -f /tmp/mqtt_bridge.pid /tmp/line_follower.pid /tmp/recharge_mode.flag

# 终止所有相机相关进程
pkill -f "camera_nodelet_manager"
pkill -f "AstraDriverNodelet"
pkill -f "image_proc/rectify"
pkill -f "depth_image_proc/convert_metric"
# 确保所有进程被终止
sleep 1
pkill -9 -f "ros"  # 强制终止所有ROS相关进程
sleep 2

# 定义清理函数
cleanup() {
    echo "捕获到中断信号，清理资源..."

    # 终止MQTT桥接节点
    if [ -n "$MQTT_PID" ] && ps -p $MQTT_PID > /dev/null; then
        echo "终止MQTT桥接节点 (PID: $MQTT_PID)..."
        kill -TERM $MQTT_PID
        sleep 1
        if ps -p $MQTT_PID > /dev/null; then
            echo "强制终止MQTT桥接节点..."
            kill -KILL $MQTT_PID
        fi
    fi

    # 终止mqtt进程
    if pgrep -f "mqtt_bridge" > /dev/null; then
        echo "警告: 仍有MQTT进程残留，强制终止..."
        pkill -9 -f "mqtt_bridge"
        sleep 2  # 等待进程终止
    fi
    
    # 终止巡线相关进程
    pkill -9 -f "line_follow"
    
    # 清理临时文件
    rm -f /tmp/mqtt_bridge.pid /tmp/line_follower.pid /tmp/recharge_mode.flag

    pkill -9 -f "ros"  # 强制终止所有ROS相关进程
    sleep 2

    echo "机器人系统已关闭"
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM

# 等待ROS核心启动
echo "等待ROS核心启动..."
sleep 5  # 增加等待时间

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误: ROS环境未设置"
    exit 1
fi

# 启动MQTT桥接节点
echo "启动MQTT桥接节点..."
roslaunch simple_follower mqtt_bridge.launch &
MQTT_PID=$!
echo "MQTT桥接节点PID: $MQTT_PID"
echo $MQTT_PID > /tmp/mqtt_bridge.pid

# 等待MQTT桥接节点初始化
echo "等待MQTT桥接节点初始化..."
sleep 10  # 增加等待时间

# 检查MQTT节点是否正常运行
if ! ps -p $MQTT_PID > /dev/null; then
    echo "错误: MQTT桥接节点启动失败"
    echo "查看日志: /tmp/mqtt_bridge.log"
    cleanup
    exit 1
fi

# 注意：巡线节点由 mqtt_bridge.py 统一管理，不在此处启动
# 避免收到 start 消息时重复启动导致 "new node registered with same name" 冲突
echo "寻线节点将在收到 start 指令后由 MQTT Bridge 启动"

echo "=========================================="
echo "   机器人系统启动完成"
echo "   MQTT桥接节点: $MQTT_PID"
echo "   寻线节点: 等待 start 指令启动"
echo "=========================================="

# 添加进程监控（仅监控MQTT节点，巡线节点由mqtt_bridge.py管理）
while true; do
    # 检查MQTT节点是否存活
    if ! ps -p $MQTT_PID > /dev/null; then
        echo "警告: MQTT桥接节点已停止，尝试重启..."
        nohup roslaunch simple_follower mqtt_bridge.launch &
        MQTT_PID=$!
        echo "新的MQTT桥接节点PID: $MQTT_PID"
        echo $MQTT_PID > /tmp/mqtt_bridge.pid
    fi

    # 注意：不再监控巡线节点，由 mqtt_bridge.py 统一管理
    # 避免双重启动导致 "new node registered with same name" 冲突

    sleep 10
done
