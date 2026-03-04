# V3 快速参考指南

## 故障排查流程

### 问题：回充触发后无响应

**步骤1**：检查回充节点
```bash
rosnode list | grep auto_recharger
# 应该看到: /auto_recharger_v2
```

**步骤2**：检查话题订阅
```bash
rostopic info /auto_recharge/trigger
# 应该看到 auto_recharger_v2 在订阅者列表中
```

**步骤3**：手动触发测试
```bash
rostopic pub /auto_recharge/trigger std_msgs/String "data: 'green_detected'"
# 观察日志输出
```

### 问题：底盘通信失败

**步骤1**：检查服务
```bash
rosservice list | grep set_charge
```

**步骤2**：测试服务调用
```bash
rosservice call /set_charge 0
# 预期: name: "true"
```

**步骤3**：检查底盘节点
```bash
rosnode info /wheeltec_robot
# 查看 Services 部分
```

**步骤4**：查看日志
```bash
cat ~/.ros/log/latest/wheeltec_robot*.log
```

### 问题：导航失败

**步骤1**：检查导航节点
```bash
rosnode list | grep move_base
# 应该看到: /move_base
```

**步骤2**：检查地图
```bash
rostopic echo /map -n 1
# 应该有地图数据
```

**步骤3**：检查定位
```bash
rostopic echo /amcl_pose -n 1
# 应该有位置数据
```

---
**版本**：V3.0  
**更新日期**：2025-11-01  
**作者**：AI智能编程助手
