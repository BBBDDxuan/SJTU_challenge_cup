# SJTU Challenge Cup 仿真系统

本仓库用于“工业环境下物体感知识别与指令交互型智能体研发”项目的前期仿真开发。  
当前阶段目标是在 **Isaac Sim + ROS2** 仿真环境中，完成一个基于自然语言指令驱动的机器人作业闭环：

> 文本/语音指令理解 → 任务规划 → 机械臂端相机感知 → 梯形线圈识别与定位 → 机械臂抓取 → 四轮底盘跨区移动 → 卡槽识别 → 线圈放置 → 机械臂复位

---

## 1. 项目目标

本项目面向工业环境下的线圈上下料任务，构建一个仿真智能体系统。机器人由 **四轮移动底盘、机械臂、夹爪和机械臂端相机** 组成，需要根据自然语言指令自主完成指定尺寸梯形线圈的识别、抓取、搬运和放置。

示例指令：

```text
请把最左边的大号梯形线圈搬运到对应卡槽中。
```

系统应自动完成：

```text
解析指令
→ 移动机械臂到线圈观察位
→ 使用机械臂端相机识别梯形线圈
→ 计算抓取位姿
→ 机械臂抓取线圈
→ 四轮底盘移动到操作区
→ 移动机械臂到卡槽观察位
→ 识别对应尺寸卡槽
→ 机械臂放置线圈
→ 机械臂复位
```

---

## 2. 系统功能

- 自然语言指令解析
- 梯形线圈尺寸识别
- 梯形线圈三维定位
- 梯形线圈抓取位姿计算
- 机械臂端相机主动观察
- 四轮移动底盘跨区移动
- 梯形卡槽识别与匹配
- 机械臂抓取、放置与复位
- 任务规划与状态机控制
- 失败检测与重试机制

---

## 3. 技术路线

```text
Isaac Sim 仿真环境
        ↓
四轮移动机器人 + 机械臂 + 夹爪 + 机械臂端相机
        ↓
ROS2 Bridge
        ↓
自然语言指令理解
        ↓
任务规划 / 行为树 / 状态机
        ↓
机械臂端相机主动感知
        ↓
梯形线圈识别与抓取位姿估计
        ↓
机械臂抓取
        ↓
四轮底盘移动
        ↓
梯形卡槽识别与放置位姿估计
        ↓
机械臂放置与复位
```

---

## 4. 仓库结构

```text
SJTU_challenge_cup/
│
├── README.md
├── LICENSE
├── .gitignore
├── requirements.txt
├── environment.yml
│
├── docs/
│   ├── system_architecture.md
│   ├── module_interfaces.md
│   ├── setup_guide.md
│   ├── run_guide.md
│   ├── troubleshooting.md
│   └── weekly_progress.md
│
├── assets/
│   ├── usd/
│   │   ├── scenes/
│   │   ├── coils/
│   │   ├── slots/
│   │   └── robot/
│   ├── urdf/
│   │   └── meshes/
│   └── textures/
│
├── isaac_sim/
│   ├── launch_scene.py
│   ├── create_scene.py
│   ├── spawn_robot.py
│   ├── spawn_trapezoid_coils.py
│   ├── camera_config.py
│   └── ros2_bridge_config.py
│
├── ros2_ws/
│   └── src/
│       ├── coilbot_bringup/
│       ├── coilbot_description/
│       ├── coilbot_msgs/
│       ├── coilbot_nlu/
│       ├── coilbot_perception/
│       ├── coilbot_planner/
│       ├── coilbot_arm_control/
│       ├── coilbot_base_control/
│       └── coilbot_task_manager/
│
├── models/
│   └── yolo/
│
├── datasets/
│   ├── raw/
│   ├── images/
│   ├── labels/
│   └── synthetic_generation/
│
├── scripts/
├── tests/
├── logs/
├── docker/
└── .github/
    ├── ISSUE_TEMPLATE/
    └── workflows/
```

---

## 5. 核心模块说明

### 5.1 `isaac_sim/`

用于 Isaac Sim 仿真场景搭建和机器人加载，包括：

- 创建工件放置区
- 创建机器人操作区
- 生成大/中/小梯形线圈
- 生成大/中/小梯形卡槽
- 加载四轮底盘、机械臂和夹爪
- 配置机械臂端相机
- 配置 ROS2 Bridge

---

### 5.2 `coilbot_nlu/`

自然语言理解模块，用于将用户输入的中文指令解析为结构化任务。

输入示例：

```text
把最左边的大号梯形线圈放到对应卡槽。
```

输出示例：

```json
{
  "target_type": "trapezoid_coil",
  "target_size": "large",
  "position_constraint": "leftmost",
  "destination": "matched_slot",
  "action": "pick_move_place"
}
```

---

### 5.3 `coilbot_perception/`

感知模块，负责机械臂端相机的图像获取、线圈识别、卡槽识别和位姿估计。

主要功能：

- 机械臂端相机图像采集
- 大/中/小梯形线圈识别
- 大/中/小梯形卡槽识别
- 梯形轮廓和角点提取
- 2D 图像坐标到 3D 世界坐标转换
- 手眼坐标变换

坐标转换关系：

```text
camera_frame
    ↓
end_effector_frame
    ↓
arm_base_frame
    ↓
world_frame
```

---

### 5.4 `coilbot_planner/`

任务规划模块，负责将自然语言解析结果转换为机器人可执行的任务序列。

任务状态机示例：

```text
IDLE
 ↓
PARSE_INSTRUCTION
 ↓
MOVE_ARM_TO_COIL_VIEW
 ↓
DETECT_TRAPEZOID_COIL
 ↓
ESTIMATE_GRASP_POSE
 ↓
EXECUTE_GRASP
 ↓
CHECK_GRASP
 ↓
MOVE_BASE_TO_OPERATION_AREA
 ↓
MOVE_ARM_TO_SLOT_VIEW
 ↓
DETECT_TRAPEZOID_SLOT
 ↓
ESTIMATE_PLACE_POSE
 ↓
EXECUTE_PLACE
 ↓
CHECK_PLACE
 ↓
FINISH
```

---

### 5.5 `coilbot_arm_control/`

机械臂控制模块，负责观察、抓取、放置和复位动作。

主要动作包括：

```text
home_pose
coil_view_pose
pre_grasp_pose
grasp_pose
lift_pose
carry_pose
slot_view_pose
pre_place_pose
place_pose
retreat_pose
```

---

### 5.6 `coilbot_base_control/`

四轮移动底盘控制模块，负责机器人从工件区移动到操作区。

两周内优先采用 waypoint 控制：

```text
pick_area_pose → middle_pose → operation_area_pose
```

---

### 5.7 `coilbot_task_manager/`

系统主控模块，用于串联所有模块，执行完整任务闭环。

完整流程：

```text
语言理解
→ 任务规划
→ 机械臂观察线圈
→ 线圈识别与定位
→ 机械臂抓取
→ 四轮底盘移动
→ 机械臂观察卡槽
→ 卡槽识别与定位
→ 机械臂放置
→ 任务完成
```

---

## 6. 环境依赖

推荐环境：

- Ubuntu 22.04
- ROS2 Humble
- Isaac Sim
- Python 3.10
- OpenCV
- NumPy
- PyTorch
- Ultralytics YOLOv8
- transforms3d / scipy
- colcon

Python 依赖可以写入 `requirements.txt`：

```text
numpy
opencv-python
scipy
pyyaml
transforms3d
ultralytics
torch
torchvision
```

---

## 7. 快速开始

### 7.1 克隆仓库

```bash
git clone https://github.com/BBBDDxuan/SJTU_challenge_cup.git
cd SJTU_challenge_cup
```

### 7.2 安装 Python 依赖

```bash
pip install -r requirements.txt
```

### 7.3 编译 ROS2 工作空间

```bash
cd ros2_ws
colcon build
source install/setup.bash
cd ..
```

### 7.4 启动 Isaac Sim 场景

```bash
python isaac_sim/launch_scene.py
```

### 7.5 启动完整任务流程

```bash
bash scripts/run_full_pipeline.sh
```

或者：

```bash
ros2 launch coilbot_bringup sim_full.launch.py
```

---

## 8. 测试指令

可用于自然语言理解模块测试：

```text
把大号梯形线圈放到对应卡槽。
请抓取中号梯形线圈并放到正确位置。
把最左边的小号梯形线圈搬运到对应卡槽。
帮我把右边的大梯形线圈放入大卡槽。
我要将中间的中号梯形线圈放置到正确卡槽中。
```

---

## 9. 当前阶段开发重点

前两周仿真阶段的重点是先跑通完整闭环：

- [ ] Isaac Sim 场景搭建
- [ ] 梯形线圈和梯形卡槽建模
- [ ] 四轮底盘模型与移动控制
- [ ] 机械臂端相机配置
- [ ] 梯形线圈识别
- [ ] 梯形卡槽识别
- [ ] 线圈抓取位姿计算
- [ ] 自然语言指令解析
- [ ] 任务状态机设计
- [ ] 机械臂抓取与放置控制
- [ ] 全流程集成测试

---

## 10. 分工建议

| 成员 | 负责方向 | 主要目录 |
|---|---|---|
| 1号 | 系统架构与总集成 | `coilbot_task_manager/`, `coilbot_bringup/` |
| 2号 | Isaac Sim 场景与梯形工件 | `assets/`, `isaac_sim/` |
| 3号 | 四轮底盘控制 | `coilbot_base_control/` |
| 4号 | 机械臂端相机与标定 | `coilbot_perception/` |
| 5号 | 梯形线圈识别 | `coilbot_perception/`, `datasets/`, `models/` |
| 6号 | 抓取位姿估计 | `coilbot_perception/pose_estimator.py` |
| 7号 | 卡槽识别与放置位姿 | `coilbot_perception/` |
| 8号 | 自然语言理解 | `coilbot_nlu/` |
| 9号 | 任务规划与行为树 | `coilbot_planner/` |
| 10号 | 机械臂抓取与放置 | `coilbot_arm_control/` |

---

## 11. Git 协作规范

### 11.1 分支命名

建议每个人不要直接在 `main` 分支上开发，而是使用自己的功能分支：

```bash
git checkout -b feature/perception-coil
git checkout -b feature/arm-control
git checkout -b feature/task-planner
```

### 11.2 提交格式

建议 commit 信息清楚说明改了什么：

```bash
git commit -m "add trapezoid coil detector"
git commit -m "update arm grasp poses"
git commit -m "add task state machine"
```

### 11.3 合并流程

推荐流程：

```text
个人功能分支开发
→ push 到 GitHub
→ 提交 Pull Request
→ 组长检查
→ 合并到 main
```

---

## 12. 常见问题

### 12.1 GitHub 不显示空文件夹

GitHub 不会保存空文件夹。可以在空文件夹中加入 `.gitkeep` 文件。

### 12.2 机械臂相机看不到完整线圈

需要调整 `coil_view_pose`，保证梯形线圈完整出现在相机视野中。

### 12.3 抓取点偏移

优先检查：

```text
camera_frame → end_effector_frame → arm_base_frame → world_frame
```

这几个坐标变换是否正确。

### 12.4 四轮底盘移动后卡槽坐标不准

底盘移动后机器人基座位姿发生变化，应在到达操作区后重新用机械臂端相机识别卡槽，不要直接使用移动前的卡槽坐标。

---

## 13. 项目状态

当前项目处于前两周仿真搭建阶段，优先目标是完成一个可运行的最小闭环系统：

```text
输入指令 → 识别梯形线圈 → 抓取 → 四轮移动 → 识别卡槽 → 放置
```

后续可继续扩展：

- YOLOv8-seg 梯形线圈分割
- FoundationPose 位姿估计
- 更稳定的机械臂抓取策略
- 真实机械臂与相机部署
- Sim2Real 迁移
- 工业场景大模型微调
