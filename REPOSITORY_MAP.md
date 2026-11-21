# 仓库模块与历史

| 分支 | 用途 |
| --- | --- |
| main | 仓库首页和模块导航。 |
| [chassis](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/chassis) | 完整 F407 工程，含 X42S、麦轮运动、IMU 航向、厘米路线和停车。初始提交 1056527 已验证 Debug 构建与 ELF 调用链。 |
| [vision](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/vision) | F407 与 K230 的 TEXT TRACK 传输、K230 脚本以及协议和测试入口。 |
| [arm](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/arm) | 机械臂调参、抓取阶段、姿态基线和测试。构建与测试使用 [共享 F407 工程](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/105652712c3fa6cf648085afd292aeac009f8864)。 |

机械臂探针上位机见 [ZM-HostComputer](https://github.com/XCiscoding/ZM-HostComputer)。

旧快照保留原名和提交，可从[标签页](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tags)查找。ZiMing/Motor、ZiMing/Vision 和 archive/YuXuan 保存历史协作快照，archive/ 保存 F407 归档，reuse/ 保存六个独立复用模块的快照。
