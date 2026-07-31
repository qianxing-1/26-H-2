# H题钢球平衡控制工程

STM32F103C8T6 通过 USART2 接收摄像头返回的钢球坐标，使用
TIM1_CH4 在 PA11 输出步进脉冲，并通过 PA10/PA12 控制 DIR/ENA。

- PC14：切换要求3、要求4/5两个模式；运行中按下会先停止。
- PC15：启动当前模式。
- 模式1（要求3）：从任意有效小球位置出发，先定点控制到 +5 cm，进入到达范围后立即切换目标并稳定在 -5 cm。
- 模式2（要求4/5）：持续稳定在中心坐标333。

当前标定为中心333、+5 cm=496、-5 cm=170。控制、滤波和步进电机
参数集中在 `User/main.c` 与 `Hardware/EMM_Gimbal.c` 文件顶部。
如果实际DIR接线与当前约定相反，只需把 `BALANCE_MOTOR_SIGN` 改为 `-1.0f`。

## 模式1调参

按下开始键后不检查小球是否位于中心，第一目标固定为 `BALL_PLUS_5CM_X`。
小球进入 +5 判定窗口，或向右运动时高速跨过整个窗口后，立即把目标切换为 `BALL_MINUS_5CM_X`，
两个阶段使用不同的模式3制动参数；加速、位置和速度环参数暂时仍与模式2共用。

- `MODE3_PLUS_REACH_PIXELS`：+5 cm 的到达窗口；向右高速跨过整个窗口也会切换，避免漏判。
- `MODE3_PLUS_BRAKE_ENABLE_PIXELS`：去 +5 时只在距离目标小于该值后允许反向制动，防止半路停下。
- `MODE3_PLUS_DRIVE_MIN_TARGET_STEPS`：进入去 +5 制动区之前的最小驱动倾角。
- `MODE3_PLUS_BRAKE_*`：去 +5 使用的短预测、弱制动参数，只要求可靠进入 +5 判定范围。
- `MODE3_MINUS_BRAKE_*`：去 -5 使用的提前制动、强制动参数，负责最终停稳。
- `MODE3_MINUS_BRAKE_TAPER_PIXELS`：去 -5 时最低强制动步数开始渐变减小的末端范围。
- `MODE3_MINUS_BRAKE_MIN_TARGET_NEAR_STEPS`：到达 -5 附近时的最低强制动步数；减小可降低末端振荡。
- `MODE3_FINAL_ERROR_PIXELS`：最终位于 -5 cm 附近的位置判定窗口。
- `MODE3_FINAL_VELOCITY`：最终完成判定允许的小球速度。
- `MODE3_FINAL_STABLE_FRAMES`：位置和速度连续满足条件的视觉帧数。

## 串口协议

波特率115200，摄像头发送十进制X坐标和带符号速度，并以换行结束：

```text
320,12.50\n
```

其中速度单位为像素/秒。有效坐标范围为1~639。收到0、640或更大的值时，主控立即停止STEP脉冲；
摄像头不再发送数据时，主控会在120 ms超时后执行相同的停机保护。

## 位置-速度串级平衡

模式1的两个移动阶段、最终 -5 cm 保持阶段以及模式2都使用同一套位置-速度串级反馈。定义：

```text
e = 小球坐标 - 目标坐标
e_predict = e + 小球速度 * 制动预测时间
v_ref = -Kpos(e_predict) * e_predict
v_error = v_ref - 小球实测速度
电机目标步数 = -Kaccel/Kbrake * v_error + 中心附近积分补偿
```

`Kpos(e)` 会随误差大小在近端、远端增益之间变化。小球在目标右侧且静止时，
`e > 0`、`v_ref < 0`，电机目标步数为正，管道顺时针升高并让小球向中心加速。
控制器先用速度预测计算未来误差。当小球向中心的负速度超过 `v_ref` 时，`v_error` 会在小球到达中心前变为正，
目标步数随即变为负，电机快速反向到水平位置另一侧进行制动。左侧过程完全对称。
每次过中心后的误差和速度都会继续进入同一关系，逐次减小振幅，最后进入中心保持区。

电机使用固定高速追踪目标步数，普通目标变化经过轻微滤波；目标步数一旦反号，
滤波会被旁路以保证制动换向及时。模式2直接使用已经实测有效的 `MODE3_MINUS_BRAKE_*` 制动参数，
包括速度预测、制动滞回和末端最低制动力渐变。积分只在中心附近启用，用于补偿管道零位和摩擦造成的静态偏差。

## 串级控制调参

建议先把 `BALANCE_POSITION_INTEGRAL_KI` 临时设为0，调好位置/速度反馈后再加积分：

- `BALANCE_MOTOR_SIGN`：方向不符时只把 `1.0f` 改为 `-1.0f`。
- `BALANCE_POSITION_VEL_KP_FAR`：远离目标时允许的回中心速度斜率；增大后加速更积极、制动更晚。
- `BALANCE_POSITION_VEL_KP_NEAR`：接近目标时的速度斜率；减小会更早制动，过小则靠近中心很慢。
- `BALANCE_GAIN_SCHEDULE_PIXELS`：模式2位置增益达到远端最大值所需的预测误差；当前为 `160 px`，适配约半段行程。
- `MODE3_GAIN_SCHEDULE_PIXELS`：模式3长行程的增益调度距离；当前保持 `320 px`。
- `BALANCE_ACCEL_STEP_KP`：加速阶段速度误差对应的倾角；只影响正常回中力度。
- `MODE3_MINUS_BRAKE_STEP_KP`：模式3去 -5 和模式2共同使用的制动增益；增大可增强反向制动力。
- `MODE3_MINUS_BRAKE_PREDICT_TIME_S`：共同使用的速度预测时间；增大会更早制动，建议每次调整 `0.05 s`。
- `MODE3_MINUS_BRAKE_MIN_TARGET_STEPS`：远离目标时的最低强制动步数。
- `MODE3_MINUS_BRAKE_TAPER_PIXELS`：进入目标附近该范围后，最低强制动步数开始渐变减小。
- `MODE3_MINUS_BRAKE_MIN_TARGET_NEAR_STEPS`：目标附近的最低强制动步数；越小越不易抖动，过小会刹不住。
- `MODE3_MINUS_BRAKE_MIN_VELOCITY`、`MODE3_MINUS_BRAKE_RELEASE_VELOCITY`：进入和退出强制动的速度滞回阈值。
- `MODE3_MINUS_BRAKE_HOLD_FRAMES`：制动保持帧数；当前2帧在50 FPS下约40 ms。
- `BALANCE_DESIRED_VELOCITY_MAX`：期望速度上限；过冲严重时先降低，响应不足时再提高。
- `BALANCE_TARGET_LIMIT_STEPS`：最大倾斜步数，限制最强加速和制动角度。
- `BALANCE_TARGET_FILTER_ALPHA`：同方向目标步数的响应系数，越接近1越敏捷；反号换向始终立即执行。
- `BALANCE_MOTOR_FIXED_SPEED_HZ`：追踪目标步数的统一高速；失步时降低，换向和到位太慢时提高。
- `BALANCE_MOTOR_STOP_WINDOW_STEPS`：目标步数停止窗口；太大会动作不足，太小会在目标位置附近来回补脉冲。
- `BALANCE_POSITION_INTEGRAL_KI`：最后消除静态偏差；只能小幅增加，持续低频摆动时应减小。

模式2建议按以下顺序调节：

1. 回中加速不足：减小 `BALANCE_GAIN_SCHEDULE_PIXELS`，或增大 `BALANCE_ACCEL_STEP_KP`。
2. 到中心时速度仍大：增大 `MODE3_MINUS_BRAKE_PREDICT_TIME_S`，每次增加 `0.05 s`。
3. 反向及时但刹不住：增大 `MODE3_MINUS_BRAKE_STEP_KP`；仍不足再增大 `MODE3_MINUS_BRAKE_MIN_TARGET_STEPS`。
4. 中心附近抖动：减小 `MODE3_MINUS_BRAKE_MIN_TARGET_NEAR_STEPS`，或增大 `MODE3_MINUS_BRAKE_TAPER_PIXELS`。
5. 加速与制动频繁切换：增大进入/退出制动速度的差值，或把 `MODE3_MINUS_BRAKE_HOLD_FRAMES` 从2增至3。
6. 最后长期偏离中心：其他参数稳定后再小幅增加 `BALANCE_POSITION_INTEGRAL_KI`。

模式2与模式3去 -5 直接共用 `MODE3_MINUS_BRAKE_*`。修改这些参数会同时影响两种模式；
`MODE3_PLUS_BRAKE_*` 仍只影响去 +5 的过程。
