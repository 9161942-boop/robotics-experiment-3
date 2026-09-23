# 实验三提交材料清单

| 要求 | 仓库位置 | 状态 |
|---|---|---|
| ROS 2 工程 | `code/src/` | 已包含 |
| Launch、控制器与场景参数 | `code/src/mecharm_fixed_pick_sim/launch/`、`config/`、`worlds/` | 已包含 |
| 机械臂与夹爪模型 | `code/src/mycobot_ros2/mycobot_description/` | 已包含 |
| 手动示教与自动播放 | `code/manual_animation.json`、`code/src/mecharm_fixed_pick_sim/mecharm_fixed_pick_sim/` | 已包含 |
| 场景与网格证据 | `evidence/` | 已包含 |
| 红蓝基线对象级记录 | `evidence/mecharm_sorting_execution.csv` | 6/6，已归档 |
| 异常与调试证据 | `evidence/ERROR_LOG.md`、流程图、过程图、日志和配置 | 已包含摘要 |
| 三类目标检测模型 | `models/best_yolov8n_eraser_lock_stapler.pt`、`models/README.md` | 已包含权重、类别、训练配置和哈希 |
| 外接摄像头检测结果 | `evidence/model_detection_realtime.png` | 已包含橡皮、锁、订书机同框检测截图 |
| Gazebo 仿真演示视频 | `videos/experiment3_gazebo_sorting_run_01.webm`、`videos/experiment3_gazebo_sorting_run_02.webm` | 已包含 |
| 小组实验报告 | `report/experiment3_lab_report_group17.docx/pdf` | 已包含 |
| 个人实验报告 | `report/experiment3_personal_report_yangmuqing.docx/pdf` | 已包含 |
| GitHub 链接与提交截图 | 两份报告及 `report/figures/github_commit_history_exp3.png` | 已包含 |

三分类扩展以场景、航点和自动播放证据为准；真机内容只记录部署和验收计划。

详细逐项审计见 [`SUBMISSION_AUDIT.md`](SUBMISSION_AUDIT.md)。

## 提交前仍需确认的材料

| 项目 | 当前状态 | 提交前动作 |
|---|---|---|
| 前序目标检测模型与推理说明 | `models/best_yolov8n_eraser_lock_stapler.pt`、`models/README.md` | 已补充三类权重、类别、训练配置、指标和 SHA-256；若老师还要求推理脚本，可再补充部署命令 |
| 真机运行结果 | 报告中为部署、标定和安全验收计划 | 若教师要求真机实测证据，补充现场视频、运行日志和对象级结果；当前两段视频均为 Gazebo 仿真 |
