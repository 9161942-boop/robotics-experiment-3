# 实验三异常记录与处理摘要

本文件整理实验三仿真调试中出现过的异常现象、程序分支和处理方法。它与 `mecharm_sorting_execution.csv` 配套使用：CSV 保存红蓝基线对象级执行结果，本文件保存异常处理规则和调试摘要。除 CSV 中明确归档的红蓝基线外，本文件不把三分类扩展或真机内容写成独立成功率。

| 编号 | 异常场景 | 触发条件或现象 | 程序处理 | 对应证据 |
|---|---|---|---|---|
| E01 | 未识别或低置信度 | 类别为空、类别不在红蓝绿集合，或置信度低于 `min_confidence=0.80` | 写入 `detect/skipped`，不进入抓取队列 | `sorting_task.py`、`state_flow.png` |
| E02 | 网格外或重复网格 | 检测框中心距离所有网格超过阈值，或两个目标映射到同一网格 | 写入 `outside grid` 或 `duplicate grid assignment`，跳过该目标 | `sorting_task.py`、`grid_mapping.png` |
| E03 | 抓取未附着 | 夹爪闭合后目标没有随末端抬升，或附着状态未确认 | 记录 `failed`/`exception`，停止当前目标并回到安全处理路径 | `sorting_executor.py`、`grasp_stages.png`、`blue_cylinder_grasp_verified.png` |
| E04 | 释放位姿异常 | 释放后目标位姿超出分类垫 XY 容差，或高度超过安全阈值 | 记录 `release pose outside destination`，结束当前目标并发布失败状态 | `sorting_executor.py`、`release_stages.png` |
| E05 | 节点启动或重复实例问题 | 旧 Gazebo、控制器或播放器实例未清理，或节点日志调用参数不兼容 | 清理旧实例、修正日志调用、重新启动并检查状态主题 | 个人报告第4.5、4.6节，`system_flow.png` |
| E06 | 空计划或服务不可用 | 任务计划为空，或 `/gazebo/set_entity_state` 服务不可用 | 发布 `failed:empty_plan` 或记录服务异常，停止继续执行 | `sorting_executor.py`、`evidence_chain.png` |

## 验收边界

- E01、E02、E03、E04、E06 是当前状态机和执行器中已经实现的异常分支；日志字符串与代码保持一致，便于复现和检索。
- E05 是本次仿真调试中记录的启动与日志问题，修复后已通过干净环境启动检查。
- 若课程要求提交独立的异常运行视频或逐次原始终端日志，还需在最终提交前补充对应文件；本摘要不能替代未归档的原始视频。
