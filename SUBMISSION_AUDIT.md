# 实验三提交材料审计

审计日期：2026-09-23  
审计范围：任务书要求、实验三提交仓库、最终压缩包和 GitHub `main` 分支。

## 1. 当前已整理的材料

| 类别 | 位置 | 审计结论 |
|---|---|---|
| ROS 2 工程 | `code/` | 已包含，17 个 Python 文件通过静态语法检查 |
| 单一启动入口 | `code/src/mecharm_fixed_pick_sim/launch/sorting_gazebo.launch.py` | 已包含，统一启动 Gazebo、场景监视、任务规划和执行节点 |
| 场景与机械臂模型 | `code/src/.../worlds/`、`config/`、`mycobot_ros2/` | 已包含 |
| 检测消息接口 | `sorting_scene_monitor.py`、`sorting_task.py` | 已包含 `vision_msgs/Detection2DArray`，类别、框中心和置信度字段齐全 |
| 任务控制接口 | `sorting_task.py`、`sorting_executor.py` | 已提供状态与结果反馈主题，属于状态机式等价接口；没有独立 ROS 2 Action Server |
| 自动整理证据 | `evidence/mecharm_sorting_execution.csv` | 红蓝六目标对象级结果为 6/6 |
| 三类检测模型 | `models/best_yolov8n_eraser_lock_stapler.pt`、`models/README.md` | 已包含橡皮、锁、订书机 YOLOv8n 权重、训练配置和哈希 |
| 外接摄像头检测画面 | `evidence/model_detection_realtime.png` | 已包含三类目标同框检测、置信度和 FPS 证据 |
| 异常处理证据 | `evidence/ERROR_LOG.md`、流程图、过程图 | 已包含 E01-E06 分支摘要；原始异常运行视频仍未归档 |
| Gazebo 演示视频 | `videos/experiment3_gazebo_sorting_run_01.webm`、`run_02.webm` | 已包含，两段均为仿真视频 |
| 实验报告 | `report/` | 小组/个人 DOCX、PDF 和 GitHub 截图均已包含 |
| GitHub 同步 | GitHub `main` | 已同步，当前 `main` 保留 20 次提交记录；报告内附提交历史截图 |

## 2. 与任务书逐项对照

| 任务书要求 | 当前证据 | 状态 |
|---|---|---|
| 不少于两类物体 | 红、蓝基线；红、蓝、绿扩展场景 | 已满足仿真要求 |
| 至少六个物体且正确整理至少五个 | 红蓝对象级 CSV：6 个目标全部 `passed` | 已满足仿真证据要求 |
| 任务开始后自动选择网格和分类区 | `sorting_task.py`、`sorting_executor.py`、自动播放序列 | 已满足仿真设计要求 |
| 空网格、未识别、不可达、抓取失败等异常 | `ERROR_LOG.md`、E01-E06、状态流程图 | 已实现并有摘要；若要求原始异常录像需补充 |
| 一个 Launch 启动完整仿真系统 | `sorting_gazebo.launch.py` | 已满足 |
| 目标检测模型、类别说明和网格配置 | `models/` 中的 YOLOv8n 权重、README、类别说明和网格配置 | 已满足模型材料提交要求；若老师要求部署脚本，需另附推理命令或脚本 |
| BehaviorTree XML 或状态机配置 | Python 状态机、状态/结果主题和配置文件 | 已有等价实现；如老师指定 XML，需另导出 XML |
| 仿真与真机演示视频 | 两段 Gazebo 视频已包含；没有真机视频 | **真机部分待补（若课程要求）** |
| 分类结果、异常记录、任务日志和实验报告 | CSV、`ERROR_LOG.md`、证据图、DOCX/PDF | 已包含 |

## 3. 仍需补齐的材料

1. **模型权重和推理说明**：已补充实验三实际使用的三类 YOLOv8n 权重、类别配置、训练摘要和 SHA-256。当前仿真检测节点仍从 Gazebo `ModelStates` 生成确定性 `Detection2DArray`，训练权重用于外接摄像头和后续真机感知接入；若老师要求完整推理脚本，再补充脚本和运行命令即可。
2. **真机验收证据**：需要 Jetson/mechArm 的现场视频、至少六个目标的对象级结果、抓取/放置结果和异常日志。当前报告已明确写成部署计划，不能作为真机实测结果。
3. **可选增强**：若教师要求“异常演示视频”或“原始终端日志”，还需从录屏/终端导出独立文件；`ERROR_LOG.md` 目前是规则和调试摘要。

## 4. 当前提交结论

当前材料可以作为“实验三 Gazebo 仿真版”提交：代码、仿真证据、两份报告、两段视频和 GitHub 多次提交均已整理。若课程验收包含任务书中的真机阶段，则补齐模型/推理说明和真机证据后，才可称为完整提交。
