# 模块接口文档 module_interfaces.md

## 1. 总体接口约定

本文档用于统一“工业环境下物体感知识别与指令交互型智能体研发”项目中各模块之间的通信接口，保证自然语言理解、任务规划、视觉检测、目标选择、位姿估计、机械臂控制、四轮底盘控制、夹爪控制和系统主控模块能够顺利集成。

本项目标准文字输入为：

```text
请把最左边的大号线圈搬运到对应卡槽中
```

该指令中的含义为：

```text
目标物体：线圈
目标属性：大号
目标选择规则：从候选大号线圈中选择最左边的一个
放置目标：与当前线圈尺寸对应的卡槽
任务动作：抓取 → 搬运 → 放置
```

系统整体流程如下：

```text
自然语言指令输入
→ 指令解析
→ 任务规划
→ 机械臂移动到线圈观察位
→ 相机检测候选线圈
→ 根据选择规则选出目标线圈
→ 估计目标线圈 6D 位姿
→ 估计抓取位姿
→ 机械臂抓取线圈
→ 四轮底盘移动到操作区
→ 机械臂移动到卡槽观察位
→ 相机检测对应卡槽
→ 估计放置位姿
→ 机械臂放置线圈
→ 夹爪松开
→ 机械臂复位
→ 任务结束
```

模块之间的数据传输优先采用 ROS2 Topic / Service / Action。  
在前期仿真和联调阶段，可以先使用 JSON 格式作为统一数据结构，后期再根据稳定字段转为 ROS2 自定义消息。

接口设计原则如下：

1. 自然语言理解模块负责将文字指令转换为结构化任务描述。
2. 视觉检测模块只负责检测候选目标，不负责判断“最左边”等任务语义。
3. 目标选择模块负责根据 `target.selector` 从候选目标中选择最终操作对象。
4. YOLOv8 主要负责二维目标检测，输出类别、检测框和置信度。
5. FoundationPose 或其他位姿估计算法负责输出目标物体的 6D 位姿。
6. 卡槽识别不接收 `leftmost`，只根据目标线圈属性匹配对应卡槽。
7. 所有涉及位姿的数据必须明确 `frame_id`。

---

## 2. 各模块 ROS2 节点名称

| 模块 | ROS2 包名 | 节点名称 | 主要功能 |
|---|---|---|---|
| 系统主控模块 | `coilbot_task_manager` | `task_manager_node` | 串联完整任务流程，控制状态机运行 |
| 自然语言理解模块 | `coilbot_nlu` | `instruction_parser_node` | 将文本/语音指令解析为结构化任务 |
| 任务规划模块 | `coilbot_planner` | `task_planner_node` | 将结构化任务转化为执行步骤 |
| 线圈检测模块 | `coilbot_perception` | `coil_detector_node` | 使用 YOLOv8 等方法检测指定类型和尺寸的候选线圈 |
| 目标选择模块 | `coilbot_planner` | `target_selector_node` | 根据选择规则从候选目标中选择最终操作对象 |
| 线圈位姿估计模块 | `coilbot_perception` | `coil_pose_estimator_node` | 使用 FoundationPose / RGB-D 几何 / 仿真真值估计目标线圈 6D 位姿 |
| 抓取位姿估计模块 | `coilbot_perception` | `grasp_pose_estimator_node` | 根据线圈 6D 位姿估计抓取位姿 |
| 卡槽检测模块 | `coilbot_perception` | `slot_detector_node` | 检测与线圈尺寸对应的目标卡槽 |
| 放置位姿估计模块 | `coilbot_perception` | `place_pose_estimator_node` | 根据卡槽位姿估计放置位姿 |
| 机械臂控制模块 | `coilbot_arm_control` | `arm_control_node` | 控制机械臂到观察位、抓取位、放置位和复位 |
| 四轮底盘控制模块 | `coilbot_base_control` | `base_control_node` | 控制四轮底盘从工件放置区移动到机器人操作区 |
| 夹爪控制模块 | `coilbot_arm_control` | `gripper_control_node` | 控制夹爪打开、闭合、保持和释放 |
| 状态监控模块 | `coilbot_task_manager` | `status_monitor_node` | 输出当前任务状态、错误信息和调试日志 |

---

## 3. 各模块输入输出

### 3.1 自然语言理解模块

输入：

| 输入来源 | 内容 |
|---|---|
| 裁判/用户 | 自然语言文本指令 |

示例输入：

```json
{
  "text": "请把最左边的大号线圈搬运到对应卡槽中"
}
```

输出：

```json
{
  "target": {
    "object_type": "coil",
    "attributes": {
      "size": "large"
    },
    "selector": {
      "type": "spatial",
      "relation": "leftmost",
      "reference_area": "workpiece_area"
    }
  },
  "destination": {
    "type": "matched_slot",
    "match_by": "size"
  },
  "action": "pick_move_place"
}
```

字段说明：

| 字段 | 含义 |
|---|---|
| `target.object_type` | 目标物体类型，本任务中为 `coil` |
| `target.attributes.size` | 目标尺寸，可选 `large` / `medium` / `small` |
| `target.selector.type` | 选择规则类型，例如空间规则 `spatial` |
| `target.selector.relation` | 选择关系，例如 `leftmost` |
| `target.selector.reference_area` | 选择规则作用区域，本任务中为工件放置区 `workpiece_area` |
| `destination.type` | 放置目标类型，本任务中为 `matched_slot` |
| `destination.match_by` | 匹配依据，本任务中按 `size` 匹配 |
| `action` | 任务动作类型，本任务中为 `pick_move_place` |

---

### 3.2 任务规划模块

输入：

```json
{
  "target": {
    "object_type": "coil",
    "attributes": {
      "size": "large"
    },
    "selector": {
      "type": "spatial",
      "relation": "leftmost",
      "reference_area": "workpiece_area"
    }
  },
  "destination": {
    "type": "matched_slot",
    "match_by": "size"
  },
  "action": "pick_move_place"
}
```

输出：

```json
{
  "task_id": "task_001",
  "steps": [
    "move_arm_to_coil_view_pose",
    "detect_coil_candidates",
    "select_target_object",
    "estimate_coil_pose",
    "estimate_grasp_pose",
    "execute_grasp",
    "move_base_to_operation_area",
    "move_arm_to_slot_view_pose",
    "detect_matched_slot",
    "estimate_place_pose",
    "execute_place",
    "reset_arm",
    "finish"
  ]
}
```

---

### 3.3 线圈检测模块

线圈检测模块主要由 YOLOv8 算法实现。该模块只负责检测候选线圈，不负责输出完整 6D 位姿，也不负责根据“最左边”等规则选择最终目标。

输入：

```json
{
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "image_topic": "/camera/color/image_raw"
}
```

输出：

```json
{
  "detected": true,
  "object_type": "coil",
  "candidates": [
    {
      "object_id": "coil_large_01",
      "attributes": {
        "size": "large"
      },
      "bbox_2d": {
        "x_center": 320,
        "y_center": 240,
        "width": 120,
        "height": 80,
        "unit": "pixel"
      },
      "confidence": 0.91,
      "frame_id": "camera_color_frame"
    },
    {
      "object_id": "coil_large_02",
      "attributes": {
        "size": "large"
      },
      "bbox_2d": {
        "x_center": 450,
        "y_center": 245,
        "width": 118,
        "height": 82,
        "unit": "pixel"
      },
      "confidence": 0.88,
      "frame_id": "camera_color_frame"
    }
  ]
}
```

如果使用 YOLOv8-OBB，可增加二维旋转框字段：

```json
{
  "bbox_obb": {
    "cx": 320,
    "cy": 240,
    "width": 120,
    "height": 80,
    "angle_2d": 1.57,
    "unit": "rad"
  }
}
```

注意：`angle_2d` 只是图像平面内的二维旋转角，不等价于物体三维姿态中的 `roll`、`pitch`、`yaw`。

---

### 3.4 目标选择模块

目标选择模块负责根据自然语言解析得到的 `target.selector`，从检测模块输出的候选目标中选择最终操作对象。

输入：

```json
{
  "selector": {
    "type": "spatial",
    "relation": "leftmost",
    "reference_area": "workpiece_area"
  },
  "candidates": [
    {
      "object_id": "coil_large_01",
      "attributes": {
        "size": "large"
      },
      "bbox_2d": {
        "x_center": 320,
        "y_center": 240,
        "width": 120,
        "height": 80,
        "unit": "pixel"
      },
      "confidence": 0.91,
      "frame_id": "camera_color_frame"
    },
    {
      "object_id": "coil_large_02",
      "attributes": {
        "size": "large"
      },
      "bbox_2d": {
        "x_center": 450,
        "y_center": 245,
        "width": 118,
        "height": 82,
        "unit": "pixel"
      },
      "confidence": 0.88,
      "frame_id": "camera_color_frame"
    }
  ]
}
```

输出：

```json
{
  "selected": true,
  "object_id": "coil_large_01",
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "selection_reason": "leftmost_in_workpiece_area",
  "bbox_2d": {
    "x_center": 320,
    "y_center": 240,
    "width": 120,
    "height": 80,
    "unit": "pixel"
  },
  "confidence": 0.91,
  "frame_id": "camera_color_frame"
}
```

选择规则说明：

| `relation` | 含义 |
|---|---|
| `leftmost` | 选择候选集中最左侧目标 |
| `rightmost` | 选择候选集中最右侧目标 |
| `nearest` | 选择距离机器人或相机最近的目标 |
| `highest_confidence` | 选择置信度最高的目标 |
| `none` | 不使用额外选择规则 |

---

### 3.5 线圈位姿估计模块

线圈位姿估计模块负责估计目标线圈的 6D 位姿，即三维位置和三维姿态。该模块可由 FoundationPose、PnP、RGB-D 几何计算或仿真真值实现。

输入：

```json
{
  "object_id": "coil_large_01",
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "bbox_2d": {
    "x_center": 320,
    "y_center": 240,
    "width": 120,
    "height": 80,
    "unit": "pixel"
  },
  "rgb_image_topic": "/camera/color/image_raw",
  "depth_image_topic": "/camera/depth/image_raw",
  "camera_info_topic": "/camera/color/camera_info",
  "cad_model": "models/coil_large/coil_large.obj"
}
```

输出：

```json
{
  "object_id": "coil_large_01",
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "pose_6d": {
    "position": {
      "x": 1.20,
      "y": 0.40,
      "z": 0.15
    },
    "orientation": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 1.57
    },
    "frame_id": "world"
  },
  "confidence": 0.86,
  "pose_source": "foundationpose"
}
```

`pose_source` 可选值：

| 值 | 含义 |
|---|---|
| `foundationpose` | 使用 FoundationPose 估计 6D 位姿 |
| `rgbd_geometry` | 使用 RGB-D 几何和相机内参估计位姿 |
| `simulation_ground_truth` | 使用仿真环境中的真值位姿 |
| `assumed_plane_plus_obb` | 假设目标位于平面上，并结合二维旋转框估计部分姿态 |

---

### 3.6 抓取位姿估计模块

输入：

```json
{
  "object_id": "coil_large_01",
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "pose_6d": {
    "position": {
      "x": 1.20,
      "y": 0.40,
      "z": 0.15
    },
    "orientation": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 1.57
    },
    "frame_id": "world"
  }
}
```

输出：

```json
{
  "object_id": "coil_large_01",
  "grasp_pose": {
    "x": 1.20,
    "y": 0.40,
    "z": 0.25,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  },
  "pre_grasp_pose": {
    "x": 1.20,
    "y": 0.40,
    "z": 0.40,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  }
}
```

---

### 3.7 机械臂控制模块

输入：

```json
{
  "task": "pick",
  "target_pose": {
    "x": 1.20,
    "y": 0.40,
    "z": 0.25,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  },
  "gripper_action": "close"
}
```

输出：

```json
{
  "success": true,
  "current_state": "grasp_finished",
  "message": "机械臂已完成抓取动作"
}
```

---

### 3.8 四轮底盘控制模块

输入：

```json
{
  "task": "move_base",
  "target_area": "operation_area",
  "target_pose": {
    "x": 3.00,
    "y": 0.00,
    "yaw": 0.0,
    "frame_id": "world"
  }
}
```

输出：

```json
{
  "success": true,
  "current_area": "operation_area",
  "message": "四轮底盘已到达机器人操作区"
}
```

---

### 3.9 卡槽检测模块

卡槽检测模块负责根据当前夹持线圈的属性，检测与其对应的卡槽。该模块不接收 `leftmost` 等目标选择规则。

输入：

```json
{
  "slot_type": "matched_slot",
  "match_by": "size",
  "matched_attributes": {
    "size": "large"
  },
  "image_topic": "/camera/color/image_raw"
}
```

输出：

```json
{
  "detected": true,
  "slot_id": "slot_large_01",
  "slot_type": "coil_slot",
  "attributes": {
    "size": "large"
  },
  "position": {
    "x": 3.10,
    "y": 0.55,
    "z": 0.80
  },
  "orientation": {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "confidence": 0.88,
  "frame_id": "world"
}
```

---

### 3.10 放置位姿估计模块

输入：

```json
{
  "slot_id": "slot_large_01",
  "slot_type": "coil_slot",
  "attributes": {
    "size": "large"
  },
  "position": {
    "x": 3.10,
    "y": 0.55,
    "z": 0.80
  },
  "orientation": {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "frame_id": "world"
}
```

输出：

```json
{
  "slot_id": "slot_large_01",
  "place_pose": {
    "x": 3.10,
    "y": 0.55,
    "z": 0.85,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  },
  "pre_place_pose": {
    "x": 3.10,
    "y": 0.55,
    "z": 1.00,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  }
}
```

---

### 3.11 夹爪控制模块

输入：

```json
{
  "gripper_action": "open"
}
```

或：

```json
{
  "gripper_action": "close"
}
```

输出：

```json
{
  "success": true,
  "gripper_state": "open",
  "message": "夹爪已打开"
}
```

---

### 3.12 系统主控模块

输入：

```json
{
  "text": "请把最左边的大号线圈搬运到对应卡槽中"
}
```

输出：

```json
{
  "task_id": "task_001",
  "success": true,
  "final_state": "FINISH",
  "message": "任务完成"
}
```

---

## 4. JSON 字段格式

### 4.1 任务解析字段

| 字段 | 类型 | 可选值 | 含义 |
|---|---|---|---|
| `target.object_type` | string | `coil` / `part` / `workpiece` | 目标物体类型 |
| `target.attributes.size` | string | `large` / `medium` / `small` / `none` | 目标尺寸属性 |
| `target.selector.type` | string | `spatial` / `confidence` / `none` | 选择规则类型 |
| `target.selector.relation` | string | `leftmost` / `rightmost` / `nearest` / `highest_confidence` / `none` | 选择关系 |
| `target.selector.reference_area` | string | `workpiece_area` / `operation_area` / `none` | 选择规则作用区域 |
| `destination.type` | string | `matched_slot` / `fixed_area` | 放置目标类型 |
| `destination.match_by` | string | `size` / `color` / `shape` / `none` | 匹配依据 |
| `action` | string | `pick_move_place` | 任务动作类型 |

### 4.2 二维检测框字段

| 字段 | 类型 | 单位 | 含义 |
|---|---|---|---|
| `x_center` | int / float | pixel | 检测框中心点 x 坐标 |
| `y_center` | int / float | pixel | 检测框中心点 y 坐标 |
| `width` | int / float | pixel | 检测框宽度 |
| `height` | int / float | pixel | 检测框高度 |
| `angle_2d` | float | rad | 图像平面内旋转角，仅用于 OBB |

### 4.3 三维位姿字段

| 字段 | 类型 | 单位 | 含义 |
|---|---|---|---|
| `x` | float | m | X 方向位置 |
| `y` | float | m | Y 方向位置 |
| `z` | float | m | Z 方向位置 |
| `roll` | float | rad | 绕 X 轴旋转 |
| `pitch` | float | rad | 绕 Y 轴旋转 |
| `yaw` | float | rad | 绕 Z 轴旋转 |
| `frame_id` | string | - | 当前位姿所在坐标系 |

### 4.4 识别结果字段

| 字段 | 类型 | 含义 |
|---|---|---|
| `detected` | bool | 是否识别成功 |
| `object_type` | string | 识别到的物体类型 |
| `attributes` | object | 目标属性，例如尺寸、颜色、形状 |
| `confidence` | float | 识别置信度，范围 0 到 1 |
| `object_id` | string | 目标物体编号 |
| `slot_id` | string | 卡槽编号 |
| `candidates` | array | 候选目标列表 |

### 4.5 执行结果字段

| 字段 | 类型 | 含义 |
|---|---|---|
| `success` | bool | 当前动作是否执行成功 |
| `current_state` | string | 当前任务状态 |
| `message` | string | 状态说明或错误提示 |
| `error_code` | string | 错误码，成功时可为空 |

---

## 5. ROS2 Topic / Service 名称

### 5.1 Topic 接口

| Topic 名称 | 发布节点 | 订阅节点 | 消息内容 |
|---|---|---|---|
| `/task_instruction` | `task_manager_node` | `instruction_parser_node` | 原始自然语言指令 |
| `/parsed_task` | `instruction_parser_node` | `task_planner_node` / `task_manager_node` | 结构化任务 JSON |
| `/planned_steps` | `task_planner_node` | `task_manager_node` | 任务步骤序列 |
| `/coil_candidates` | `coil_detector_node` | `target_selector_node` / `task_manager_node` | 候选线圈检测结果 |
| `/selected_target` | `target_selector_node` | `task_manager_node` / `coil_pose_estimator_node` | 被选中的目标物体 |
| `/coil_pose` | `coil_pose_estimator_node` | `task_manager_node` / `grasp_pose_estimator_node` | 目标线圈 6D 位姿 |
| `/grasp_pose` | `grasp_pose_estimator_node` | `task_manager_node` / `arm_control_node` | 抓取位姿 |
| `/slot_detection_result` | `slot_detector_node` | `task_manager_node` / `place_pose_estimator_node` | 卡槽检测结果 |
| `/place_pose` | `place_pose_estimator_node` | `task_manager_node` / `arm_control_node` | 放置位姿 |
| `/arm_command` | `task_manager_node` | `arm_control_node` | 机械臂目标位姿与动作 |
| `/base_command` | `task_manager_node` | `base_control_node` | 四轮底盘目标点 |
| `/gripper_command` | `task_manager_node` | `gripper_control_node` | 夹爪开合命令 |
| `/system_state` | `task_manager_node` | `status_monitor_node` | 当前系统状态 |
| `/task_result` | `task_manager_node` | 外部显示/日志模块 | 最终任务结果 |

---

### 5.2 Service 接口

| Service 名称 | 服务端节点 | 客户端节点 | 功能 |
|---|---|---|---|
| `/parse_instruction` | `instruction_parser_node` | `task_manager_node` | 解析自然语言指令 |
| `/plan_task` | `task_planner_node` | `task_manager_node` | 生成任务步骤 |
| `/detect_coil_candidates` | `coil_detector_node` | `task_manager_node` | 检测候选线圈 |
| `/select_target_object` | `target_selector_node` | `task_manager_node` | 从候选目标中选择最终操作对象 |
| `/estimate_coil_pose` | `coil_pose_estimator_node` | `task_manager_node` | 估计目标线圈 6D 位姿 |
| `/estimate_grasp_pose` | `grasp_pose_estimator_node` | `task_manager_node` | 估计抓取位姿 |
| `/execute_arm_motion` | `arm_control_node` | `task_manager_node` | 执行机械臂动作 |
| `/move_base_to_target` | `base_control_node` | `task_manager_node` | 控制底盘移动 |
| `/detect_matched_slot` | `slot_detector_node` | `task_manager_node` | 检测对应卡槽 |
| `/estimate_place_pose` | `place_pose_estimator_node` | `task_manager_node` | 估计放置位姿 |
| `/control_gripper` | `gripper_control_node` | `task_manager_node` | 控制夹爪开合 |

---

## 6. 坐标系约定

本项目统一采用右手坐标系，所有模块在输出位姿时必须明确 `frame_id`。

| 坐标系 | 含义 |
|---|---|
| `world` | 仿真世界坐标系，用于描述场景中线圈、卡槽、机器人全局位置 |
| `map` | 导航地图坐标系，可用于底盘全局导航 |
| `odom` | 底盘里程计坐标系 |
| `base_link` | 四轮底盘本体坐标系 |
| `arm_base_link` | 机械臂基座坐标系 |
| `end_effector_link` | 机械臂末端执行器坐标系 |
| `gripper_link` | 夹爪坐标系 |
| `camera_link` | 机械臂端相机坐标系 |
| `camera_color_frame` | 彩色相机图像坐标系 |
| `camera_depth_frame` | 深度相机坐标系 |
| `coil_link` | 线圈物体坐标系 |
| `slot_link` | 卡槽坐标系 |

### 坐标转换要求

1. YOLOv8 检测模块通常输出图像坐标系下的 `bbox_2d`，其 `frame_id` 为 `camera_color_frame`。
2. 线圈位姿估计模块需要结合 RGB-D 图像、相机内参、CAD 模型或仿真真值，将二维检测结果转换为 6D 位姿。
3. 位姿估计模块输出的 `pose_6d` 可以位于 `camera_link`、`arm_base_link` 或 `world` 坐标系，但必须明确 `frame_id`。
4. 机械臂控制模块优先接收 `arm_base_link` 或 `world` 坐标系下的目标位姿。
5. 底盘控制模块优先接收 `map` 或 `world` 坐标系下的目标位姿。
6. 所有输出位姿必须包含 `frame_id`，禁止只输出坐标值而不说明坐标系。

---

## 7. 错误码和失败处理

| 错误码 | 含义 | 可能原因 | 处理方式 |
|---|---|---|---|
| `E_PARSE_FAIL` | 指令解析失败 | 指令格式复杂、关键词缺失 | 使用模板解析或请求重新输入 |
| `E_INVALID_TARGET` | 目标无效 | 未识别到尺寸或目标类型 | 默认选择裁判指定尺寸或返回错误 |
| `E_NOT_FOUND` | 未识别到目标 | 相机视角错误、遮挡、模型不清晰 | 调整机械臂观察位并重试 |
| `E_LOW_CONFIDENCE` | 识别置信度过低 | 光照、角度、算法不稳定 | 重新识别或使用仿真真值坐标 |
| `E_TARGET_SELECTION_FAIL` | 目标选择失败 | 候选目标为空或选择规则无法应用 | 使用置信度最高目标或返回错误 |
| `E_POSE_ESTIMATION_FAIL` | 6D 位姿估计失败 | FoundationPose 失败、深度缺失、CAD 模型不匹配 | 使用 RGB-D 简化估计或仿真真值坐标 |
| `E_GRASP_POSE_FAIL` | 抓取位姿估计失败 | 位姿不合理、超出机械臂工作空间 | 使用预设抓取位姿 |
| `E_ARM_PLAN_FAIL` | 机械臂路径规划失败 | 目标位姿不可达、碰撞约束失败 | 调整预抓取点或使用固定轨迹 |
| `E_GRASP_FAIL` | 抓取失败 | 夹爪摩擦力不足、对准误差 | 重新抓取一次，或执行趋势动作 |
| `E_BASE_MOVE_FAIL` | 底盘移动失败 | 路径被挡、控制误差大 | 使用预设路径或直接移动到固定目标点 |
| `E_SLOT_NOT_FOUND` | 未识别到卡槽 | 相机视角错误、卡槽遮挡 | 调整观察位并重试，或使用预设卡槽坐标 |
| `E_PLACE_POSE_FAIL` | 放置位姿估计失败 | 卡槽位姿不稳定 | 使用预设放置位姿 |
| `E_PLACE_FAIL` | 放置失败 | 对位误差、夹爪释放不稳定 | 至少执行向目标卡槽下放的趋势动作 |
| `E_TIMEOUT` | 任务超时 | 单步执行时间过长 | 终止当前任务并输出失败状态 |

### 失败处理原则

1. 每个关键步骤最多重试 3 次。
2. 如果视觉检测失败，可以调整观察位重新检测。
3. 如果目标选择失败，可以在同尺寸候选目标中选择置信度最高者作为降级方案。
4. 如果 6D 位姿估计失败，可以临时使用仿真环境中的真值坐标作为降级方案。
5. 如果抓取失败，应至少保证机械臂有明显对准和触碰目标线圈的动作趋势。
6. 如果卡槽识别失败，可以根据线圈尺寸匹配预设卡槽坐标。
7. 如果最终放置失败，应至少执行向对应卡槽下放的动作趋势。
8. 所有失败情况都必须通过 `/system_state` 输出当前状态和错误码。

---

## 8. 系统状态定义

| 状态名称 | 含义 |
|---|---|
| `IDLE` | 系统空闲，等待任务输入 |
| `PARSE_INSTRUCTION` | 正在解析自然语言指令 |
| `PLAN_TASK` | 正在生成任务步骤 |
| `MOVE_ARM_TO_COIL_VIEW_POSE` | 机械臂移动到线圈观察位 |
| `DETECT_COIL_CANDIDATES` | 正在检测候选线圈 |
| `SELECT_TARGET_OBJECT` | 正在根据选择规则选出目标线圈 |
| `ESTIMATE_COIL_POSE` | 正在估计目标线圈 6D 位姿 |
| `ESTIMATE_GRASP_POSE` | 正在估计抓取位姿 |
| `EXECUTE_GRASP` | 正在执行抓取动作 |
| `CHECK_GRASP` | 正在检查抓取是否成功 |
| `MOVE_BASE_TO_OPERATION_AREA` | 四轮底盘移动到机器人操作区 |
| `MOVE_ARM_TO_SLOT_VIEW_POSE` | 机械臂移动到卡槽观察位 |
| `DETECT_MATCHED_SLOT` | 正在检测对应尺寸卡槽 |
| `ESTIMATE_PLACE_POSE` | 正在估计放置位姿 |
| `EXECUTE_PLACE` | 正在执行放置动作 |
| `RESET_ARM` | 机械臂复位 |
| `FINISH` | 任务完成 |
| `ERROR` | 系统发生错误 |

---

## 9. 示例完整任务数据流

### 9.1 输入指令

```json
{
  "text": "请把最左边的大号线圈搬运到对应卡槽中"
}
```

### 9.2 指令解析结果

```json
{
  "target": {
    "object_type": "coil",
    "attributes": {
      "size": "large"
    },
    "selector": {
      "type": "spatial",
      "relation": "leftmost",
      "reference_area": "workpiece_area"
    }
  },
  "destination": {
    "type": "matched_slot",
    "match_by": "size"
  },
  "action": "pick_move_place"
}
```

### 9.3 线圈检测结果

```json
{
  "detected": true,
  "object_type": "coil",
  "candidates": [
    {
      "object_id": "coil_large_01",
      "attributes": {
        "size": "large"
      },
      "bbox_2d": {
        "x_center": 320,
        "y_center": 240,
        "width": 120,
        "height": 80,
        "unit": "pixel"
      },
      "confidence": 0.91,
      "frame_id": "camera_color_frame"
    },
    {
      "object_id": "coil_large_02",
      "attributes": {
        "size": "large"
      },
      "bbox_2d": {
        "x_center": 450,
        "y_center": 245,
        "width": 118,
        "height": 82,
        "unit": "pixel"
      },
      "confidence": 0.88,
      "frame_id": "camera_color_frame"
    }
  ]
}
```

### 9.4 目标选择结果

```json
{
  "selected": true,
  "object_id": "coil_large_01",
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "selection_reason": "leftmost_in_workpiece_area",
  "bbox_2d": {
    "x_center": 320,
    "y_center": 240,
    "width": 120,
    "height": 80,
    "unit": "pixel"
  },
  "confidence": 0.91,
  "frame_id": "camera_color_frame"
}
```

### 9.5 线圈位姿估计结果

```json
{
  "object_id": "coil_large_01",
  "object_type": "coil",
  "attributes": {
    "size": "large"
  },
  "pose_6d": {
    "position": {
      "x": 1.20,
      "y": 0.40,
      "z": 0.15
    },
    "orientation": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 1.57
    },
    "frame_id": "world"
  },
  "confidence": 0.86,
  "pose_source": "foundationpose"
}
```

### 9.6 抓取位姿估计结果

```json
{
  "object_id": "coil_large_01",
  "grasp_pose": {
    "x": 1.20,
    "y": 0.40,
    "z": 0.25,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  },
  "pre_grasp_pose": {
    "x": 1.20,
    "y": 0.40,
    "z": 0.40,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  }
}
```

### 9.7 底盘移动目标

```json
{
  "task": "move_base",
  "target_area": "operation_area",
  "target_pose": {
    "x": 3.00,
    "y": 0.00,
    "yaw": 0.0,
    "frame_id": "world"
  }
}
```

### 9.8 卡槽检测结果

```json
{
  "detected": true,
  "slot_id": "slot_large_01",
  "slot_type": "coil_slot",
  "attributes": {
    "size": "large"
  },
  "position": {
    "x": 3.10,
    "y": 0.55,
    "z": 0.80
  },
  "orientation": {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "confidence": 0.88,
  "frame_id": "world"
}
```

### 9.9 放置位姿估计结果

```json
{
  "slot_id": "slot_large_01",
  "place_pose": {
    "x": 3.10,
    "y": 0.55,
    "z": 0.85,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  },
  "pre_place_pose": {
    "x": 3.10,
    "y": 0.55,
    "z": 1.00,
    "roll": 0.0,
    "pitch": 1.57,
    "yaw": 0.0,
    "frame_id": "world"
  }
}
```

### 9.10 最终任务结果

```json
{
  "task_id": "task_001",
  "success": true,
  "final_state": "FINISH",
  "message": "大号线圈已成功放置到对应卡槽中"
}
```

