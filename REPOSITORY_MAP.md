# 仓库模块与历史

| 分支 | 用途 |
| --- | --- |
| main | 仓库首页。底盘和视觉分别从各自模块分支接手。 |
| [chassis](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/chassis) | F407、X42S、麦轮运动、IMU 航向、厘米路线和停车。初始提交 1056527 已验证 Debug 构建与 ELF 调用链。 |
| [vision](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/vision) | F407 与 K230 的 TEXT TRACK 传输、脚本和协议。测试入口在该分支 README。 |

机械臂源码随 F407 固件保留。独立的机械臂探针上位机在 [ZM-HostComputer](https://github.com/XCiscoding/ZM-HostComputer)。

旧快照和拆出的复用模块保留原名与提交，可从[标签页](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tags)查找。

- ZiMing/Motor、ZiMing/Vision 和 archive/YuXuan 保存历史协作快照。
- archive/pbrobot-f6662ea 和 archive/pbrobot-worktree-20261121 保存两次 F407 归档。
- reuse/ 下的六个标签保存距离换算、路线停车、IMU 航向、麦轮混控、STEP/DIR 计步和 X42S UART 的独立模块快照。

协作从模块分支拉取；新改动可从对应模块另开分支。构建结果、烧录和实物表现分别记录。
