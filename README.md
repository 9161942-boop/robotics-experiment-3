# 实验三：桌面物体自动分类整理

本仓库保存第17组实验三的 ROS 2 + Gazebo 仿真工程、分类整理证据、实验报告和提交清单。项目目标是完成桌面物体的识别、网格定位、类别决策、机械臂抓取、分类垫放置以及异常状态记录。

## 仓库目录

- `code/`：ROS 2 工程、Gazebo 世界、URDF、控制器参数、手动控制台、自动播放程序和航点文件。
- `evidence/`：场景图、网格映射图、抓取与释放过程图、状态流程图、CSV 执行记录和场景配置。
- `report/`：小组实验报告、个人实验报告及 PDF、报告截图和渲染预览。
- `videos/`：两段实验三 Gazebo 仿真演示视频，分别对应完整自动整理运行过程。
- `SUBMISSION_CHECKLIST.md`：提交材料与实验要求的对应关系。

## 构建与运行

```bash
cd code
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mycobot_description mecharm_fixed_pick_sim
source install/setup.bash
ros2 launch mecharm_fixed_pick_sim sorting_gazebo.launch.py gui:=true execute_motion:=false
```

执行自动整理流程：

```bash
ros2 launch mecharm_fixed_pick_sim sorting_gazebo.launch.py gui:=true execute_motion:=true
```

手动示教与航点检查：

```bash
ros2 launch mecharm_fixed_pick_sim official_assembled_gazebo.launch.py gui:=true run_task:=false contact_grasp:=true
ros2 run mecharm_fixed_pick_sim manual_control_panel
```

## 结果边界

- 红蓝两类基础实验采用已归档的对象级 CSV，6 个目标的识别、抓取和放置结果为 6/6。
- 红蓝绿三分类内容包含场景、分类垫、航点和自动播放序列证据；本仓库不虚构未归档的三分类对象级成功率。
- 真机部分保留部署、标定和安全验收计划，不把仿真结果写成真机结果。

## GitHub 提交

仓库地址：<https://github.com/9161942-boop/robotics-experiment-3>

提交记录按“工程与证据同步 → 报告和 GitHub 证据补齐 → 仿真视频归档”的顺序保留，报告中同时放置仓库链接和提交历史截图。两段 Gazebo 演示视频位于 `videos/experiment3_gazebo_sorting_run_01.webm` 和 `videos/experiment3_gazebo_sorting_run_02.webm`。
