# PBRoBot F407 固件

本目录是当前唯一活动固件工程。工程使用 STM32CubeIDE，目标芯片为
STM32F407VETx。Git中的源码提交、ARM构建、烧录映像和实车结果是四层不同
证据，不能互相替代。

## 从任务到电机的调用层次

```text
任务编排层
  main.c / mission.c / chassis_route_runner.c
  选择运行档；按任务表推进路线段，完成或故障时统一停车
        ↓ ChassisRouteSegment（方向、cm、RPM、目标航向）

路线与底盘控制层
  chassis_route.c / chassis_distance.c / chassis_heading.c
  解析物理方向、换算/投影距离、产生航向修正
        ↓ 内部 vx / vy / heading_w

麦轮混控层
  chassis.c
  把车体平移和旋转命令换成 M1—M4 四路有符号 RPM
        ↓ ChassisDirectWheelRpm

电机执行层
  motor.c
  把 RPM 换成 STEP 周期，管理DIR/EN、脉冲计数和安全停车
        ↓ TIM / GPIO

板级硬件配置层
  motor_board_config.h / STM32 HAL
  保存引脚、定时器、GPIO极性和中断入口
```

上层只描述任务意图，下层只保证执行。`motor.c`不知道路线、厘米、视觉或
任务编号；路线层不知道GPIO和定时器寄存器。

常见命名只保留三个核心词：

- `Chassis`：底盘/车体层；C语言没有命名空间，因此保留模块前缀防止重名；
- `Step`：驱动器收到的一步计数，也是距离统计的基础单位；
- `CAL`：`calibration`，距离校准倍率，`1.0`表示不修正。

## 注释约定

所有新增或修改的函数（包括`static`辅助函数和测试函数）都必须有函数级注释，
说明职责、关键参数、返回值或安全边界。状态机循环还要在每个阶段解释“本轮
为什么做这件事”，不能只复述代码语法。

## main.c 为什么只运行一条循环分支

`Core/Inc/main.h`中的`APP_PROFILE_*`是编译期开关，并有“只能启用一个”的
编译检查。预处理器在构建时已经删除未选择的分支，所以`while (1)`不是在
四种模式间动态切换，而是重复服务唯一被编入固件的状态机。

厘米路线画像已经把单段运行逻辑收敛到`chassis_route_runner.c`；`main.c`只负责
初始化并每轮调用它。历史脉冲/HOLD测试仍保留在`main.c`，后续只有在它们重新
成为活动目标时才做机械搬迁，避免为“看起来统一”改动已通过的回归入口。

厘米路线状态固定为：

```text
BOOT_SAFE → STABILIZE → START_RAMP → RUN_ONCE → STOP_LATCHED
    全停       等IMU建零      起步斜坡      距离/航向控制     全停且不自动重启
```

## 厘米距离参数

常用参数集中在`Core/Inc/chassis_config.h`：

- 当前轮径候选：70 mm；
- X42S到车轮：1:1；
- 电机细分：3200 STEP/rev，唯一常量在`motor_units.h`；
- 理论初值：约145.51 STEP/cm；
- 默认路线速度：50 RPM；
- X、Y方向分别使用独立距离修正系数。

理论换算只用于生成首版候选。麦轮在不同地面、载荷和方向上的滑移不同，
最终必须分别测量并修正X/Y系数。

## 当前方向证据边界

`chassis_route.c` 接收四个有效方向，分别使用同轴的内部 X/Y 命令与步数投影。
当前实物记录确认了 Y+ 对应内部 `vy > 0`，并完成 Y+ 的定距复验。X+ 校准值、
其他方向与整条路线的实车表现仍待复验。

当前厘米路线由 `Core/Src/mission.c` 的七段任务表驱动。归档保留工作区中的
命令值，任务表与主机测试的参数一致只证明软件合同成立。里程依据是发出的
STEP 脉冲，地面实际里程仍需独立测量。

独立视觉距离对齐档中，移动时丢帧会停车，上一段路线运行器仍保留活动状态。
下一次申请移动可能被拒绝，该档的丢帧后重新启动尚未验证。

## 验证入口

离线C行为合同：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

测试会在系统临时目录用本机GCC编译纯换算模块，不打开串口、不连接调试器，
也不会产生电机动作。STM32全量构建仍应使用CubeIDE的Debug配置，并单独记录
0 error/0 warning、ELF/MAP身份以及是否真正进入运行调用链。
