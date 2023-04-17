# RM_VISION

机器人视觉算法仓库，用于RoboMaster机器人装甲板检测/车辆状态估计/弹道计算

## 项目依赖

- [rmoss_core](https://github.com/robomaster-oss/rmoss_core)：提供弹道解算工具包
- [rmoss_interfaces](https://github.com/robomaster-oss/rmoss_interfaces)：RM一般通用ROS消息接口
- [rm_hardware_interfaces](https://e.coding.net/itlkineticpanda/23chaoduiitlkineticpanda/rm_hardware_driver.git)：云台控制ROS消息接口

### 项目说明

- [auto_aim_interfaces](./auto_aim_interfaces/): 基于 @chenjunnn 开源项目 [rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim) 修改的自瞄自定义ROS消息接口
- [armor_processor](./armor_processor/): 基于 @chenjunnn 开源项目 [rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim) 修改而来的车辆估计功能包
- [openvino_armor_detector](./openvino_armor_detector/): OpenVINO装甲板检测功能包
- [projectie_motion](./projectile_motion/): 基于 [rmoss_core](https://github.com/robomaster-oss/rmoss_core) 的弹道计算功能包


## 使用说明

下载编译代码
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://e.coding.net/itlkineticpanda/23chaoduiitlkineticpanda/rm_hardware_driver.git
vcs import --recursive < rm_vision/deps.repos # clone依赖仓库

# 编译
cd ~/ros2_ws
colcon build --symlink-install
```

## 致谢

感谢 @RangerOnMars @chenjunnn @gezp @tup-robomaster (沈阳航空航天大学RoboMaster TUP战队) @robomaster-oss （RoboMaster开源软件栈项目），以上个人与组织对于本项目提供的大力支持

## Lisence

```
Copyright 2023 Yunlong Feng

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```