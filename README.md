# MQTT Robot - 基于 ROS 的 MQTT 远程控制巡线机器人

基于 ROS（Robot Operating System）的 Wheeltec 移动机器人自动巡线系统，通过 MQTT 协议实现远程控制，支持自动巡线、继电器控制、热成像温度采集和自动回充功能。

## 系统架构

系统采用 **MQTT Bridge 桥接架构**，核心由两个 ROS 节点组成：

- **mqtt_bridge** — 中枢控制节点，负责 MQTT 通信、继电器串口控制、温度采集和巡线生命周期管理
- **line_follow** — 巡线执行节点，基于摄像头图像颜色识别 + PD 控制器实现路径跟踪

```
MQTT Server (175.178.8.235:1883)
        ↕
  mqtt_bridge.py  ←→  继电器(串口) / 热成像传感器
        ↕ (ROS Topic)
  line_follow.py  ←→  摄像头 → 颜色识别 → PD控制 → 底盘驱动
```

## 核心功能

### 巡线跟踪
- 基于 HSV 色彩空间的多颜色识别（红/黄/绿线）
- PD 控制器驱动差速底盘跟线行驶
- 黄色线段触发暂停 + 继电器动作序列
- 绿色线段触发自动回充流程

### MQTT 远程控制
- 支持自动模式（巡线 + 停车检测 + 回充）和手动模式（仅巡线 + 回充）
- 十六进制指令协议，支持远程启停、模式切换
- 指数退避自动重连机制（5s - 60s）

### 继电器控制
- 8 路串口继电器（`/dev/ttyUSB0`，9600bps）
- 支持升降杆、云台、照明灯控制
- 巡线停车时自动执行继电器动作序列

### 热成像温度采集
- MI48 热成像模块（80×62 像素）
- 支持自动/手动触发温度采集
- 温度数据通过 MQTT 以 JSON 格式上报

### 自动回充
- 检测绿色线段后自动触发
- 基于 ROS Navigation 导航至充电桩
- 自动对接充电

## 项目结构

```
mqtt_robot/
├── main.py                        # 入口（示例）
├── docs/                          # 文档
├── src/
│   ├── line_follow.py             # 早期独立巡线脚本
│   ├── origin/                    # 原始版本存档
│   ├── mqtt_follow/               # V1：MQTT 控制 + 巡线
│   ├── mqtt_follow_v2/            # V2：Bridge 架构 + 回充 + 温度采集
│   │   ├── mqtt_bridge.py         # MQTT 桥接节点
│   │   ├── line_follow.py         # 巡线节点
│   │   ├── mqtt_module.py         # MQTT 客户端封装
│   │   ├── thermal_data.py        # 热成像数据采集
│   │   ├── auto_recharge_ros/     # 自动回充 ROS 包
│   │   └── *.launch / *.sh        # 启动配置
│   └── mqtt_follow_v3/            # V3（最新）：新增手动模式
│       ├── mqtt_bridge.py
│       ├── line_follow.py
│       ├── mqtt_module.py
│       └── *.launch / *.sh
```

## 版本演进

| 版本 | 目录 | 主要特性 |
|------|------|---------|
| 原始版 | `src/origin/` | 基础巡线，黑线/黄线跟踪 |
| V1 | `src/mqtt_follow/` | 增加 MQTT 远程启停控制 |
| V2 | `src/mqtt_follow_v2/` | Bridge 架构重构，红线跟踪（双HSV区间），黄/绿线检测（形态学+去抖），自动回充，温度采集 |
| **V3** | `src/mqtt_follow_v3/` | **最新版**，新增手动模式（忽略黄线停车），黄线 fallback 巡线，手动温度采集 |

## 环境要求

- **操作系统**: Ubuntu 18.04
- **ROS 版本**: ROS Melodic
- **Python**: 2.7（ROS Melodic 默认）
- **硬件平台**: Wheeltec 差速/阿克曼底盘

### Python 依赖

- `rospy` — ROS Python 客户端
- `paho-mqtt` — MQTT 客户端库
- `opencv-python` — 图像处理
- `cv_bridge` — ROS 图像桥接
- `numpy` — 数值计算
- `pyserial` — 串口通信
- `senxor` — MI48 热成像传感器驱动

### 硬件依赖

- Wheeltec 移动底盘
- Astra RGB 深度相机 / USB 摄像头
- MI48 热成像模块
- 串口继电器板（4/8 路）
- 激光雷达（导航避障）

## 部署与启动

### 部署路径

```
/home/wheeltec/wheeltec_robot/src/simple_follower/
```

### 启动方式

```bash
# 启动 MQTT Bridge + 巡线系统（V3）
cd src/mqtt_follow_v3
bash start_line.sh
```

启动脚本会自动完成：清理残留进程 → 启动 MQTT Bridge → 进程监控与自动重启。

## MQTT 通信协议

- **Broker**: `175.178.8.235:1883`
- **发布话题**: `Car/1`
- **订阅话题**: `Server1`

### 控制指令（十六进制）

| 指令 | 功能 |
|------|------|
| `01050002FF002DFA` | 自动模式启动（开灯 + 巡线） |
| `01050002000028CA` | 自动模式停止 |
| `01050002FF002ABA` | 手动模式启动（忽略黄线停车） |
| `01050002FF002BCB` | 手动模式停止 |

## 状态机（V3）

```
WAITING ──start──→ FOLLOWING ──黄线──→ STOPPED ──继电器完成──→ AVOIDING → FOLLOWING
                        │
                  manual_start
                        ↓
                 MANUAL_FOLLOWING（忽略黄线，绿线回充）
                        │
                      绿线
                        ↓
                    RECHARGING → 自动导航回充
```

## 许可证

私有项目，仅供内部使用。
