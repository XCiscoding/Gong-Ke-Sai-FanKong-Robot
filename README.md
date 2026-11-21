# PBRoBot F407 固件

本目录是当前唯一活动固件工程。工程使用 STM32CubeIDE，目标芯片为
STM32F407VETx。Git中的源码提交、ARM构建、烧录映像和实车结果是四层不同
证据，不能互相替代。

## 从任务到电机的调用层次

```text
任务编排层
  main.c / chassis_route_runner.c
  选择当前运行档；推进单段状态机，决定继续或统一停车
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
BOOT_SAFE → STABILIZE → RUN_ONCE → STOP_LATCHED
    全停       等IMU建零      距离/航向控制       全停且不自动重启
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

新直脉冲后端目前只确认：内部`vy > 0`时，车辆按用户坐标沿物理X+移动。
因此`chassis_route.c`当前只放行物理X+；X−、Y+、Y−会返回
`CHASSIS_ROUTE_RESOLVE_DIRECTION_UNVERIFIED`。每获得一个新的实物方向证据，
才在`chassis_config.h`开放对应方向。

## 验证入口

离线C行为合同：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

测试会在系统临时目录用本机GCC编译纯换算模块，不打开串口、不连接调试器，
也不会产生电机动作。STM32全量构建仍应使用CubeIDE的Debug配置，并单独记录
0 error/0 warning、ELF/MAP身份以及是否真正进入运行调用链。
