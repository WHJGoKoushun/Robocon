# 大疆电机驱动框架（STM32H723ZET6）

基于老师给的电机结构体 / PID / 控制代码整理出的最小驱动框架。
目标：让大疆 RoboMaster 电机（M3508 + M2006）安全转起来。

## 一、目录结构

```
DJ_motor/
├── Common/
│   └── macro.h         通用宏(ABS / GetSign / Clamp / ClampPeak / MIN / MAX)
├── Drivers/
│   ├── dj_motor.h      电机结构体 + 参数宏 + 函数声明
│   ├── dj_motor.c      电机初始化 / 接收解析 / 模式控制 / 发送
│   ├── pid.h           PID 结构体 + 声明
│   └── pid.c           PID 初始化 / 复位 / 计算
└── (CubeMX 生成的 Core / Drivers / MDK-ARM / *.ioc 等)
```

> 说明：原来的 `Common/common.c` 已删除（空文件，无用）；工具函数 `EncodeS16Data / ChangeDataByte` 已并入 `dj_motor.c`。

## 二、代码相对老师原版的改动（尽量少改多增）

- 补全悬空定义：`DJmotorStatus / DJmotorArgum / DJmotorError / PIDType`、宏、以及 `PID_Init / PID_Reset / DJmotor_SetZero / DJmotor_GetCanHandle / DJmotor_PID_Reload / EncodeS16Data / ChangeDataByte`。
- `DJmotor_Init` 里两个 for 循环对调：M3508 占 ID1~4、M2006 占 ID5~8（原版顺序反了，8 台会地址全反不转）。
- `DJ_Current` 分支补一行限幅写回（原版限幅结果没赋值回去）。
- `DJmotor_Monitor` 堵转判断加 `ABS`（脉冲差有正负）。
- `DJmotor_SwitchMode` 切到位置模式时先 `DJmotor_SetZero`（跑完速度再跑位置必须设零点）。
- 启用了 `DJmotor_Monitor`（超时 / 堵转自动断电流，安全）。
- 其余函数体（AngleCalculate / Receive / CurrentTransmit / SpeedMode / PositionMode / ZeroMode / PID_Caculate）原样保留。

## 三、CubeMX 6.18.1 配置步骤（MDK-ARM V5）

1. **New Project** → 搜索 `STM32H723ZET6` → Start Project。
2. **System Core → SYS**：`Debug` 选 `Serial Wire`；`Timebase Source` 选 `TIM6`。
3. **RCC**：`HSE` 选 `Crystal/Ceramic Resonator`（先确认板子晶振，H723 常用 25MHz，若板子是 8MHz 就在此处改）。
4. **Clock Configuration**：
   - 输入 HSE 频率（25MHz）。
   - 把 `SYSCLK` 配到 **550MHz**（H723 上限），CubeMX 会自动解 PLL。
   - 记住时钟树里 **FDCAN** 那一栏的频率（一般 40MHz），下面算波特率用。
5. **Connectivity → FDCAN1** → 勾选 **Activate**：
   - 引脚：推荐 `PD0 = FDCAN1_RX`、`PD1 = FDCAN1_TX`（占用就换 `PA11/PA12` 或 `PB8/PB9`，AF 选 FDCAN1）。
   - **Parameter Settings**（Nominal，目标 1Mbps）：
     - `Nominal Prescaler = 1`
     - `Nominal Sync Jump Width = 8`
     - `Nominal Time Seg1 = 31`
     - `Nominal Time Seg2 = 8`
     - 看下方 **Nominal Bit Time 应约为 1000ns（1Mbit/s）**。若不对，按 `(1+Seg1+Seg2)*Prescaler / FDCAN时钟 = 1us` 微调。
   - 再设三个"数量"参数（决定 FDCAN 预留空间）：`Standard Filters Nbr = 1`、`Rx FIFO 0 Elements Nbr = 3`、`Tx FIFO Queue Elmts Nbr = 3`。
     - filter 的**具体内容**（0x201~0x208）不用在这里配，已在 `main.c` 里用代码配好。
   - **NVIC Settings**：勾选 `FDCAN1_IT0`（Rx FIFO0）与 `FDCAN1_IT1`，并确保 Enabled 打勾。
6. **Timers → TIM2**（电机控制周期 1kHz 中断）：
   - Clock Source 选 `Internal Clock`。
   - `Prescaler = 274`、`Counter Period = 999`（275MHz ÷ 275 ÷ 1000 = 1kHz）。
   - NVIC Settings 勾选 `TIM2 全局中断`。
7. **Project Manager**：
   - Project Name 填 `H7_DJ_motor`；Location 填 `d:\RC\DJ_motor`。
   - Toolchain / IDE 选 **MDK-ARM V5**。
   - Code Generator 勾选：`Copy only the necessary library files`、`Generate peripheral initialization as a pair of .c/.h files`。
8. 点 **GENERATE CODE**。

## 四、Keil 工程接入

1. CubeMX 生成后用 Keil 打开 `d:\RC\DJ_motor\H7_DJ_motor\MDK-ARM\H7_DJ_motor.uvprojx`。
2. 把驱动源码加入工程：
   - 在工程里新建 Group（如 `Motor`），右键 → Add Existing Files，加入：
     - `d:\RC\DJ_motor\Drivers\dj_motor.c`
     - `d:\RC\DJ_motor\Drivers\pid.c`
3. 加 include 路径（Options for Target → C/C++ → Include Paths），加上：
   - `d:\RC\DJ_motor\Common`
   - `d:\RC\DJ_motor\Drivers`
   - （CubeMX 已自动加的 Core/Inc、Drivers/STM32H7xx_HAL_Driver/Inc 等保持不变）
4. 按第五节改 main.c，然后编译。

## 五、main.c 修改（中断回调 + 测试框架）

在生成的 `main.c` 顶部用户代码区加：

```c
/* USER CODE BEGIN Includes */
#include "dj_motor.h"
/* USER CODE END Includes */
```

在 `main()` 里，`MX_FDCAN1_Init();` 之后加：

```c
/* USER CODE BEGIN 2 */
/* 只接收大疆反馈帧 0x201~0x208 进 FIFO0 */
FDCAN_FilterTypeDef sFilterConfig = {0};
sFilterConfig.IdType       = FDCAN_STANDARD_ID;
sFilterConfig.FilterIndex  = 0;
sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
sFilterConfig.FilterID1    = 0x201;
sFilterConfig.FilterID2    = 0x208;
if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
{
  Error_Handler();
}

if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
{
  Error_Handler();
}

/* 打开 RX FIFO0 新消息中断,否则收不到电机反馈 */
if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
{
  Error_Handler();
}

DJmotor_Init();

/* 启动 TIM2 1kHz 中断(解包与状态机周期) */
HAL_TIM_Base_Start_IT(&htim2);
/* USER CODE END 2 */
```

接收回调（放在 main.c 用户代码区，覆盖 HAL 的 weak 回调）：

```c
/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (hfdcan == &hfdcan1)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            DJmotor_RxPush(rx_header, rx_data);
        }
    }
}
/* USER CODE END 4 */
```

TIM2 中断回调（1kHz：先解包再跑状态机，加在 `HAL_TIM_PeriodElapsedCallback` 里）：

```c
/* USER CODE BEGIN Callback 1 */
if (htim->Instance == TIM2)
{
    DJmotor_RxProcess();   // 从软件队列取包并解包
    DJmotor_Func();        // 状态机 + 发送
}
/* USER CODE END Callback 1 */
```

主循环留空即可（控制已移到 TIM2 中断）：

```c
/* USER CODE BEGIN WHILE */
while (1)
{
/* USER CODE END WHILE */
}
```

> 架构：CAN 接收中断只塞软件队列（`DJmotor_RxPush`），TIM2 中断里取队列解包（`DJmotor_RxProcess` → `DJmotor_Receive`）再跑状态机（`DJmotor_Func`）。

## 六、电机接线

- M3508 配 **C620 电调**，M2006 配 **C610 电调**。
- 所有电调 `CAN_H` 并一起、`CAN_L` 并一起，接到板子 CAN 收发器（TJA1050 / SN65HVD230），收发器 `RXD/TXD` 接 STM32 的 `FDCAN1_RX/FDCAN1_TX`（与 CubeMX 选的引脚一致）。
- 总线两端各并一个 120Ω 终端电阻（板子自带就不加）。
- 电调 ID 拨码（满配 8 台）：
  - C620（M3508）拨 **ID 1~4**；
  - C610（M2006）拨 **ID 5~8**。
- 代码里索引对应：`DJmotor[0..3]` = M3508（ID1~4），`DJmotor[4..7]` = M2006（ID5~8）。

## 七、安全测试步骤（务必按顺序，防止弄坏电机）

> 上电默认 `Begin=false`、`MODE_Set=DJ_Disable`，发 0 电流，电机不会动。

1. **先看反馈**：上电后不要使能，用手慢慢转电机，观察 `DJmotor[i].valNow.angle_deg` 是否变化、`temperature_C` 是否正常，确认 CAN 收发正常。
2. **电流模式（最安全）**：给 1 号电机（M3508）很小电流，看是否慢转：
   ```c
   DJmotor[0].Begin = true;
   DJmotor[0].MODE_Set = DJ_Current;
   DJmotor[0].valSet.current_raw = 500;   /* 很小,先确认转向 */
   ```
3. **速度模式**：
   ```c
   DJmotor[0].MODE_Set = DJ_RPM;
   DJmotor[0].valSet.speed_rpm = 500;     /* 低转速 */
   ```
4. **位置模式**（切过去会自动设零点，从当前位置平滑起步）：
   ```c
   DJmotor[0].MODE_Set = DJ_Position;
   DJmotor[0].valSet.angle_deg = 90.0f;   /* 相对设零点后的 90 度 */
   ```
5. 任何时候要停：`DJmotor[0].MODE_Set = DJ_Disable;` 或 `DJmotor[0].Begin = false;`

## 八、注意事项

- **发送触发条件依赖满配**：老师原版 `DJmotor_CurrentTransmit` 是「ID4 发 0x200 整帧（4 台 M3508）、ID8 发 0x1FF 整帧（4 台 M2006）」。只有 8 台满配时才在 ID4/ID8 触发发送。**如果实际没接满 8 台，需要把里面的 `if (motor->ID == 4U || motor->ID == 8U)` 发送条件去掉（改成每台直接发），否则部分接法下不发控制帧、电机不转。**
- **有正负含义的变量用有符号类型**：脉冲/角度/转速/电流/误差均为 `int16_t / int32_t / float`，仅纯计数与 ID 用无符号。
- **跑完速度再跑位置必须设零点**：已由 `DJmotor_SwitchMode` 在切到 `DJ_Position` 时自动调用 `DJmotor_SetZero` 完成。
- 电流限幅：M3508 限 10000、M2006 限 4500（`param.CurrentLimit_raw`），超出自动截断。
- 超时/堵转保护：`DJmotor_Monitor` 已启用，通信丢失或堵转会置 `DJ_Disable`。


