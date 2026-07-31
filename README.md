# H题钢球平衡控制工程

STM32F103C8T6 通过 USART2 接收摄像头返回的钢球坐标，使用
TIM1_CH4 在 PA11 输出步进脉冲，并通过 PA10/PA12 控制 DIR/ENA。

- PC14：依次切换要求3、要求4/5、首次坐标保持三个模式；运行中按下会先停止。
- PC15：启动当前模式。
- 模式1（要求3）：从任意有效小球位置出发，先定点控制到 +5 cm，进入到达范围后立即切换目标并稳定在 -5 cm。
- 模式2（要求4/5）：持续稳定在中心坐标322。
- 模式3（首次坐标保持）：按下PC15后，将下一帧有效小球X坐标锁定为目标，使用与模式2完全相同的回中控制器保持该坐标。

当前标定为中心322、+5 cm=450、-5 cm=185。控制、滤波和步进电机
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
滤波会被旁路以保证制动换向及时。制动阶段使用独立的较大增益和最小制动倾角，并保持若干视觉帧，
避免速度噪声使控制器在加速、制动之间高速切换。积分只在中心附近启用，用于补偿管道零位和摩擦造成的静态偏差。

## 串级控制调参

建议先把 `BALANCE_POSITION_INTEGRAL_KI` 临时设为0，调好位置/速度反馈后再加积分：

- `BALANCE_MOTOR_SIGN`：方向不符时只把 `1.0f` 改为 `-1.0f`。
- `BALANCE_POSITION_VEL_KP_FAR`：远离目标时允许的回中心速度斜率；增大后加速更积极、制动更晚。
- `BALANCE_POSITION_VEL_KP_NEAR`：接近目标时的速度斜率；减小会更早制动，过小则靠近中心很慢。
- `BALANCE_ACCEL_STEP_KP`：加速阶段速度误差对应的倾角；只影响正常回中力度。
- `BALANCE_BRAKE_STEP_KP`：制动阶段独立增益；增大后反向制动力更强，不会同时放大初始加速。
- `BALANCE_BRAKE_PREDICT_TIME_S`：速度前视时间；增大会更早制动，建议以 `0.05 s` 为步长调整。
- `BALANCE_BRAKE_MIN_TARGET_STEPS`：高速制动时的最小反向倾角，防止速度误差刚过阈值时制动力太弱。
- `BALANCE_BRAKE_HOLD_FRAMES`：制动状态保持帧数；50 FPS下4帧约80 ms，用于抑制增益切换抖动。
- `BALANCE_BRAKE_MIN_VELOCITY`、`BALANCE_BRAKE_RELEASE_VELOCITY`：进入和退出强制动的速度滞回阈值。
- `BALANCE_DESIRED_VELOCITY_MAX`：期望速度上限；过冲严重时先降低，响应不足时再提高。
- `BALANCE_TARGET_LIMIT_STEPS`：最大倾斜步数，限制最强加速和制动角度。
- `BALANCE_TARGET_FILTER_ALPHA`：同方向目标步数的响应系数，越接近1越敏捷；反号换向始终立即执行。
- `BALANCE_MOTOR_FIXED_SPEED_HZ`：追踪目标步数的统一高速；失步时降低，换向和到位太慢时提高。
- `BALANCE_MOTOR_STOP_WINDOW_STEPS`：目标步数停止窗口；太大会动作不足，太小会在目标位置附近来回补脉冲。
- `BALANCE_POSITION_INTEGRAL_KI`：最后消除静态偏差；只能小幅增加，持续低频摆动时应减小。

现场判断方法：还没接近中心就制动且响应慢，增大近端 `Kpos`；到中心速度仍很大，
减小近端 `Kpos` 或增大预测时间；电机反向了但制动力不足，优先增大 `BRAKE_STEP_KP` 或最小制动步数；
中心附近快速抖动则提高制动速度阈值、增加保持帧数、增大停止窗口或加强速度滤波。

`BALANCE_BRAKE_*` 参数只用于模式2；模式3分别使用 `MODE3_PLUS_BRAKE_*` 和
`MODE3_MINUS_BRAKE_*`，两段行程可以独立调节。

## 高灵敏机械结构

当前机构不到四分之一圈即可获得足够的加速和制动力。按1.8度步进电机、8细分估算，
一圈约1600个脉冲，45度约200个脉冲。控制器留出约20步余量，驱动层在正负45度处停止继续向外运动：

- `BALANCE_TARGET_LIMIT_STEPS = 180`：正常控制目标限制在正负40.5度以内。
- `TRACK_POSITION_LIMIT = 200`：从上电水平位置计算的正负45度软件保护限位。
- `BALANCE_MOTOR_STOP_WINDOW_STEPS = 4`：目标与估算位置相差超过4步才动作。
- `BALANCE_MOTOR_FIXED_SPEED_HZ = 450`：目标位置追踪速度，完整走完45度约需0.45秒。
- `TRACK_START_SPEED_HZ = 120`：降低启动冲击。
- `TRACK_MAX_SPEED_HZ = 600`：驱动层绝对速度上限。
- `TRACK_ACCEL_HZ_PER_S = 20000`、`TRACK_DECEL_HZ_PER_S = 40000`：减缓启动并保留较快停止能力。

该限位依赖软件累计脉冲，不是绝对位置检测。每次上电前必须先把管道放在机械水平中位；若在偏斜位置上电，软件中的零点也会随之偏移，不能保证真实角度不超过45度。

原来电机不转的直接原因是目标最大值只有30步，而停止窗口仍为160步，所有目标都会被判定为
“已经到位”；同时固定速度和驱动最大速度被调成90 Hz，响应也会非常慢。

新结构建议按以下顺序调节：

1. 先确认安全行程：让 `BALANCE_TARGET_LIMIT_STEPS` 保持小于 `TRACK_POSITION_LIMIT`，不要同时随意增大。
2. 电机仍不动作：先检查有效坐标和使能，再把 `BALANCE_MOTOR_STOP_WINDOW_STEPS` 从4减到3，不要先增大全部增益。
3. 电机会动但倾角不足：中心模式增大 `BALANCE_ACCEL_STEP_KP`；去+5增大 `MODE3_PLUS_DRIVE_MIN_TARGET_STEPS`。
4. 加速过猛：减小 `BALANCE_ACCEL_STEP_KP`，或减小 `BALANCE_TARGET_LIMIT_STEPS`。
5. 制动太晚：增大对应的 `*_BRAKE_PREDICT_TIME_S`，每次增加 `0.03~0.05 s`。
6. 换向及时但刹不住：增大对应的 `*_BRAKE_STEP_KP`，再小幅增大 `*_BRAKE_MIN_TARGET_STEPS`。
7. 目标附近抖动：减小近端最低制动步数，或把停止窗口从4增加到6~8步。
8. 电机到位太慢：提高 `BALANCE_MOTOR_FIXED_SPEED_HZ`，但不得超过 `TRACK_MAX_SPEED_HZ`。
9. 电机失步或机械冲击明显：降低 `TRACK_START_SPEED_HZ` 和 `BALANCE_MOTOR_FIXED_SPEED_HZ`。

所有带 `STEPS` 的参数现在都以步进脉冲数为单位；如果驱动器细分发生变化，需要按细分比例一起缩放。
