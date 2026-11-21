# 视觉模块

这个分支保存 F407↔K230 的 TEXT TRACK 通讯闭包：F407 文本传输层、K230 接入脚本、协议合同和静态检查器。
它不包含电机路线、机械臂、相机算法调参或固件映像。

## 当前模块

- `f407-vision-transport/`：`MODE,TRACK` / `ACK,TRACK`、`TRACK` / `LOST` 文本协议的 F407 适配代码与 C 主机回归测试。
- `f407-vision-transport/EVIDENCE.md`：原候选的构建、协议测试和物理验收边界。
- `k230/line_track_uart.py`：当前现场使用的 K230 TRACK 脚本。
- `k230/uart_link_test.py`：仅用于运输层定位的旧基线；不能替代视觉脚本。
- `F407-K230-TEXT-TRACK-协议合同.md`：引脚、串口参数、握手、允许帧与禁止项的唯一通讯合同。
- `tools/check_k230_track_protocol.py`：新 K230 TRACK 脚本上传前必须通过的静态合同检查。
- `VISION_TEST_STATUS.md`：本次提交的测试身份回执；项目实时状态仍只读项目任务看板。

## 边界

该快照证明协议实现、脚本身份和静态检查，不替代当前 K230 画面、目标识别稳定性、物理 UART 或视觉驱动底盘的实车验收。固件映像是可再生成物，故意不提交。

## 测试前检查

```powershell
python tools/check_k230_track_protocol.py --source k230/line_track_uart.py
```

该命令通过后，仍须按 `VISION_TEST_STATUS.md` 的物理边界执行被动观测；不能把静态 PASS 当成 F407↔K230 已联通。

## 其他模块

底盘的完整 CubeIDE 工程在 [chassis](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/tree/chassis)。仓库的模块入口和旧快照见[仓库地图](https://github.com/XCiscoding/Gong-Ke-Sai-FanKong-Robot/blob/main/REPOSITORY_MAP.md)。
