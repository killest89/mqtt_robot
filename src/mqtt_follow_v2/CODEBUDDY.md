# CODEBUDDY.md This file provides guidance to CodeBuddy Code when working with code in this repository.

## 项目概述

基于 ROS (Robot Operating System) 的 Wheeltec 移动机器人自动巡线与回充系统，部署目录：`/home/wheeltec/wheeltec_robot/`，ROS 包名：`simple_follower`。

## 常用命令

### 系统启动
```bash
# 完整启动（MQTT桥接 + 巡线节点 + 进程监控 + 自动重启）
./start_line.sh

# 单独启动 MQTT 桥接节点
roslaunch simple_follower mqtt_bridge.launch

# 单独启动巡线节点（带/无调试窗口）
roslaunch simple_follower line_follower.launch show_debug_window:=true
roslaunch simple_follower line_follower.launch show_debug_window:=false
```

### 回充相关
```bash
# 启动导航 + 自动回充（顺序执行）
roslaunch turn_on_wheeltec_robot navigation.launch
roslaunch auto_recharge_ros auto_recharger_node_v1.launch
```

### 调试命令
```bash
# 发送控制指令
rostopic pub /line_follower/control std_msgs/String "data: 'start'"
rostopic pub /line_follower/control std_msgs/String "data: 'stop'"

# 监控速度指令
rostopic echo /cmd_vel

# 查看节点/话题
rosnode list && rostopic list
```

## 架构设计

### 核心节点通信

```
                    MQTT Broker (175.178.8.235:1883)
                           ↑↓
                    ┌──────────────┐
                    │ mqtt_bridge  │ ← 订阅 Server1，发布 Car/1
                    └──────┬───────┘
                           │ /line_follower/control (String)
                           │ /line_follower/relay_trigger (String)
                           ↓
                    ┌──────────────┐
                    │ line_follow  │ ← 状态机控制 + PD巡线
                    └──────┬───────┘
                           │ cmd_vel_ori (Twist)
                           ↓
                    ┌──────────────┐
                    │wheeltec_robot│ ← 底盘驱动
                    └──────────────┘
```

### 状态机流转

```
WAITING ──start──→ FOLLOWING ──黄色线段──→ STOPPED ──继电器完成(10s后)──→ AVOIDING ──绕行完成──→ FOLLOWING
    ↑                   │
    │                   │ 绿色线段
    │                   ↓
    └────stop────── RECHARGING ──→ 触发 follow_end ──→ mqtt_bridge 启动回充脚本
```

### ROS 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/line_follower/control` | String | mqtt_bridge → line_follow | 控制指令：start/stop/relay_sequence_finish |
| `/line_follower/relay_trigger` | String | line_follow → mqtt_bridge | 继电器触发：trigger/follow_end |
| `cmd_vel_ori` | Twist | line_follow → wheeltec_robot | 速度指令输出 |
| `/camera/rgb/image_raw` | Image | camera → line_follow | Astra 相机图像 |

### MQTT 消息协议

| 方向 | Topic | 消息格式 | 说明 |
|------|-------|----------|------|
| 订阅 | Server1 | `start`/`stop`/HEX字节 | 服务器控制指令 |
| 发布 | Car/1 | JSON `{timestamp, device_id, temperature, data_type, sequence_index}` | 温度数据上报 |

### 继电器控制（串口 /dev/ttyUSB0, 9600bps）

| 指令 | HEX | 功能 |
|------|-----|------|
| 1_ON/OFF | A0 01 01 A2 / A0 01 00 A1 | 第1路（升杆） |
| 2_ON/OFF | A0 02 01 A3 / A0 02 00 A2 | 第2路（降杆） |
| 3_ON/OFF | A0 03 01 A4 / A0 03 00 A3 | 第3路 |
| 4_ON/OFF | A0 04 01 A5 / A0 04 00 A4 | 第4路 |
| 8_ON/OFF | A0 08 01 A9 / A0 08 00 A8 | 第8路（照明） |

### 颜色检测阈值 (HSV) - 带去抖机制

| 颜色 | H范围 | S范围 | V范围 | 用途 | 触发条件 |
|------|-------|-------|-------|------|----------|
| 红色 | 0-10/160-180 | 90-255 | 60-255 | 主巡线路径 | 双区间掩码 |
| 黄色 | 26-34 | 43-255 | 46-255 | 暂停触发点 | 连续3帧>10%占比 |
| 绿色 | 65-85 | 70-255 | 70-255 | 回充触发点 | 连续3帧>10%占比 |

**注意**：红色占比>=15%时会抑制黄/绿检测，避免误触发。

## 关键文件说明

| 文件 | 职责 |
|------|------|
| `mqtt_bridge.py` | MQTT 通信（指数退避重连）、继电器序列控制、温度数据上报、回充流程触发 |
| `line_follow.py` | 状态机、颜色检测（形态学+ROI+去抖）、PD 控制巡线 |
| `mqtt_module.py` | paho-mqtt 客户端封装（MqttClientWrapper） |
| `stream_usb_max250804.py` | MI48 热成像传感器数据采集（80×62，依赖 senxor 库） |
| `start_line.sh` | 启动脚本（进程监控+自动重启+信号清理） |
| `auto_recharge_ros/scripts/line_auto_recharger_v2.py` | 回充流程启动器（启动 navigation + auto_recharger） |

## 已知问题

### 回充节点冲突
`line_auto_recharger_v2.py` 使用 `roslaunch` 启动 `navigation.launch`，会导致与常驻节点（`/wheeltec_robot`、`/base_to_link` 等）命名冲突。`mqtt_bridge.py` 中的 `stop_line_follower_launch()` 会在回充前尝试停止这些节点。

**解决方向**：
1. 复用常驻节点，回充脚本仅启动导航逻辑
2. 使用命名空间隔离回充相关节点

### Python 兼容性
代码需兼容 Python 2.7（ROS Melodic）和 Python 3.x，注意 `ConnectionError` 和字节处理的兼容写法。

## 硬件依赖

- 底盘：Wheeltec 差速轮机器人
- 相机：Astra RGB 相机 或 USB 摄像头
- 传感器：MI48 热成像模块（80×62，需 senxor 库）
- 继电器：4/8 路串口继电器板（/dev/ttyUSB0）
- 雷达：用于导航避障

## 开发注意事项

1. **部署路径**：代码需部署到 `/home/wheeltec/wheeltec_robot/src/simple_follower/` 目录
2. **热成像脚本路径**：`stream_usb_max250804.py` 硬编码在 `/home/wheeltec/wheeltec_robot/src/auto_recharge_ros/scripts/`
3. **PID 文件**：进程 PID 存储在 `/tmp/mqtt_bridge.pid` 和 `/tmp/line_follower.pid`
4. **MQTT 重连**：使用指数退避策略（5s-60s），避免频繁重连
