# H题钢球平衡控制工程

STM32F103C8T6 通过 USART2 接收摄像头返回的钢球坐标，使用
TIM1_CH4 在 PA11 输出步进脉冲，并通过 PA10/PA12 控制 DIR/ENA。

- PC14：切换要求3、要求4/5两个模式；运行中按下会先停止。
- PC15：启动当前模式。
- 模式1（要求3）：从水平中心出发，依次执行升高加速、反向倾斜制动、回到水平滑行，在 -5 cm 附近停止。
- 模式2（要求4/5）：持续稳定在中心坐标333。

当前标定为中心333、+5 cm=496、-5 cm=170。控制、滤波和步进电机
参数集中在 `User/main.c` 与 `Hardware/EMM_Gimbal.c` 文件顶部。
如果实际DIR接线与当前约定相反，只需把 `BALANCE_MOTOR_SIGN` 改为 `-1.0f`。

## 模式1调参

上电前必须把管道调到水平并将小球放在中心。模式1把启动时的位置作为零点，
按目标脉冲位置运行；400脉冲约等于丝杆1 mm，1745脉冲约等于管道1度。

- `MODE1_HEIGHT_SIGN`：整套轨迹方向相反时改为 `-1.0f`。
- `MODE1_ACCEL_HEIGHT_STEPS`：第一段升高量，越大则向 +5 cm 加速越强。
- `MODE1_BRAKE_HEIGHT_STEPS`：第二段反向倾斜量，绝对值越大则制动和回程越强。
- `MODE1_FIXED_SPEED_HZ`：模式1所有阶段统一使用的高速；各阶段只改变目标步数。
- `MODE1_STOP_WINDOW_STEPS`：接近目标位置时停止高速脉冲的步数窗口；最终微调也使用同一高速，只改变目标步数。
- `MODE1_SWITCH_TO_BRAKE_X`：第一次提前反转坐标；调小会更早制动，调大则更靠近 +5 cm。
- `MODE1_SWITCH_TO_LEVEL_X`：第二次提前回水平坐标；调大会更早回平，调小则更靠近 -5 cm。
- `MODE1_FINAL_ERROR_PIXELS`、`MODE1_FINAL_VELOCITY`：判断最终停止的坐标和速度范围。

## 串口协议

波特率115200，摄像头发送十进制X坐标和带符号速度，并以换行结束：

```text
320,12.50\n
```

其中速度单位为像素/秒。有效坐标范围为1~639。收到0、640或更大的值时，主控立即停止STEP脉冲；
摄像头不再发送数据时，主控会在120 ms超时后执行相同的停机保护。

## 位置-速度串级平衡

模式2以及模式1到达 -5 cm 后的稳定阶段使用同一套位置-速度串级反馈。定义：

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

模式1前三个轨迹阶段仍通过 `MODE1_BRAKE_PREDICT_TIME_S`、
`MODE1_LEVEL_PREDICT_TIME_S`、两个 `MODE1_*_BRAKE_MARGIN_PIXELS` 和切换坐标提前换向；
进入 -5 cm 保持阶段后自动切换到上述串级反馈。
