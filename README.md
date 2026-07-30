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
小球进入 `MODE3_PLUS_REACH_PIXELS` 范围后立即把目标切换为 `BALL_MINUS_5CM_X`，
两个阶段共用模式3独立的预测制动参数；加速、位置和速度环参数暂时仍与模式2共用。

- `MODE3_PLUS_REACH_PIXELS`：认为已经到达 +5 cm 的位置窗口；增大会更早切换到 -5 cm。
- `MODE3_BRAKE_STEP_KP`：模式3独立制动增益，不影响模式2中心平衡。
- `MODE3_BRAKE_PREDICT_TIME_S`：模式3独立预测时间；中途过早刹停时减小，到端点速度仍大时增大。
- `MODE3_BRAKE_MIN_VELOCITY`、`MODE3_BRAKE_RELEASE_VELOCITY`：模式3进入和退出强制动的速度滞回阈值。
- `MODE3_BRAKE_HOLD_FRAMES`：模式3制动保持帧数；当前2帧在50 FPS下约40 ms。
- `MODE3_BRAKE_MIN_TARGET_STEPS`：模式3高速制动的最小反向倾角；过大容易在途中被刹停。
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

`BALANCE_BRAKE_*` 参数只用于模式2；模式3使用独立的 `MODE3_BRAKE_*` 参数。
当前模式3去 +5 和去 -5 共用同一组参数。
