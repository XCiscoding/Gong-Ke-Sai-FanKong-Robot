# 机械臂模块

| 入口 | 内容 |
| --- | --- |
| [调参](Core/Src/arm_tune.c) | 关节参数与调参接口。 |
| [抓取阶段面板](Core/Src/arm_grasp_stage_panel.c) | 抓取阶段的选择和动作请求。 |
| [抓取状态](Core/Src/arm_grasp_capture.c) | 调参关节与保持状态。 |
| [姿态基线](Core/Inc/arm_grasp_frozen_baseline.h) | 已记录的抓取参数。 |
| [测试](tests) | 机械臂静态测试。 |

本分支提供机械臂模块源码。构建和测试在[完整 F407 工程](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/105652712c3fa6cf648085afd292aeac009f8864)中进行，使用同版本的应用入口、CubeIDE 配置和 HAL。该工程默认选中底盘档，机械臂档的单独构建和上板表现待验收。
