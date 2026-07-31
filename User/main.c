#include "stm32f10x.h"
#include "Timer.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "step_pwm.h"
#include "EMM_Gimbal.h"

/* 相机坐标以及±5cm任务像素标定 */
#define BALL_CENTER_X                322        // 小球处于中心位置对应的相机X像素
#define BALL_PLUS_5CM_X              450  // 小球向右偏移+5cm目标像素
#define BALL_MINUS_5CM_X             180  // 小球向左偏移?5cm目标像素
#define BALL_COORDINATE_MAX          640        // 相机X轴像素最大值

#define CONTROL_PERIOD_MS              5        // 控制循环周期 5ms
#define TARGET_STALE_MS              120        // 视觉数据超时时间，超过该时间认为目标丢失
#define DISPLAY_PERIOD_MS             80        // OLED屏幕刷新周期

/* New linkage: keep commanded travel inside 45 degrees (200 pulses at 8 microsteps). */
/* Position/velocity cascade tuning; positive target steps command CW/up. */
#define BALANCE_POSITION_VEL_KP_FAR     6.0f
#define BALANCE_POSITION_VEL_KP_NEAR    0.80f
#define BALANCE_DESIRED_VELOCITY_MAX  650.0f
#define BALANCE_ACCEL_STEP_KP            0.35f
#define BALANCE_BRAKE_STEP_KP            0.70f
#define BALANCE_BRAKE_PREDICT_TIME_S     0.30f
#define BALANCE_BRAKE_MIN_VELOCITY      80.0f
#define BALANCE_BRAKE_RELEASE_VELOCITY  50.0f
#define BALANCE_BRAKE_HOLD_FRAMES           2
#define BALANCE_BRAKE_MIN_TARGET_STEPS  35.0f
#define BALANCE_BRAKE_TARGET_ALPHA       1.00f
#define BALANCE_POSITION_INTEGRAL_KI     0.06f
#define BALANCE_GAIN_SCHEDULE_PIXELS   240.0f
#define BALANCE_INTEGRAL_ZONE_PIXELS    32.0f
#define BALANCE_INTEGRAL_MAX_STEPS      16.0f
#define BALANCE_TARGET_LIMIT_STEPS     180.0f
#define BALANCE_TARGET_FILTER_ALPHA      0.90f
#define BALANCE_MOTOR_FIXED_SPEED_HZ   450.0f
#define BALANCE_MOTOR_STOP_WINDOW_STEPS   4.0f
#define BALANCE_CENTER_HOLD_PIXELS        6.0f
#define BALANCE_CENTER_HOLD_VELOCITY     50.0f
#define BALANCE_MOTOR_SIGN                1.0f
/* Requirement 3 point-to-point tuning for the high-sensitivity linkage. */
#define MODE3_PLUS_REACH_PIXELS                    12.0f
#define MODE3_PLUS_BRAKE_ENABLE_PIXELS             50.0f
#define MODE3_PLUS_DRIVE_MIN_TARGET_STEPS          40.0f
#define MODE3_PLUS_BRAKE_STEP_KP                     0.35f
#define MODE3_PLUS_BRAKE_PREDICT_TIME_S              0.08f
#define MODE3_PLUS_BRAKE_MIN_VELOCITY              110.0f
#define MODE3_PLUS_BRAKE_RELEASE_VELOCITY           75.0f
#define MODE3_PLUS_BRAKE_HOLD_FRAMES                    1
#define MODE3_PLUS_BRAKE_MIN_TARGET_STEPS           22.0f
#define MODE3_MINUS_BRAKE_STEP_KP                    0.50f
#define MODE3_MINUS_BRAKE_PREDICT_TIME_S             0.30f
#define MODE3_MINUS_BRAKE_MIN_VELOCITY              80.0f
#define MODE3_MINUS_BRAKE_RELEASE_VELOCITY          50.0f
#define MODE3_MINUS_BRAKE_HOLD_FRAMES                   2
#define MODE3_MINUS_BRAKE_MIN_TARGET_STEPS          38.0f
#define MODE3_MINUS_BRAKE_TAPER_PIXELS              45.0f
#define MODE3_MINUS_BRAKE_MIN_TARGET_NEAR_STEPS     12.0f
#define MODE3_FINAL_ERROR_PIXELS                     8.0f
#define MODE3_FINAL_VELOCITY                        35.0f
#define MODE3_FINAL_STABLE_FRAMES                       8
/* STM32端滤波，抑制相机单像素抖动；大小跳变使用不同滤波系数 */
#define POSITION_FILTER_ALPHA_NEAR    0.48f    // 小球位置变化小时，滤波系数，数值越小滤波越强
#define POSITION_FILTER_ALPHA_FAR     0.78f    // 小球位置变化大时，滤波系数，响应更快
#define POSITION_FILTER_FAR_PIXELS   12.0f     // 像素变化超过该阈值切换快速滤波
#define VELOCITY_FILTER_ALPHA         0.30f    // 小球速度一阶低通滤波系数
#define MAX_MEASURED_VELOCITY       2500.0f     // 小球测量速度最大限幅，抑制异常跳变

/* 任务模式3要求：先到达+5cm，再反向稳定到?5cm */
#define MODE_REQUIREMENT_3               1     // 模式3：+5cm再到?5cm往复任务
#define MODE_REQUIREMENT_4_5             2     // 模式4/5：小球保持在中心

#define MODE3_PHASE_TO_PLUS              0     // 模式3状态：运动到+5cm
#define MODE3_PHASE_TO_MINUS             1     // 模式3状态：运动到?5cm
#define MODE3_PHASE_HOLD_MINUS           2     // 模式3状态：稳定保持?5cm

uint8_t KeyNum = 0;                              // 按键返回值
volatile uint16_t x = 0;                         // 小球原始X像素坐标
volatile uint16_t y = 0;                         // 小球原始Y像素坐标，本程序未使用
volatile int16_t Gimbal_Target_Offset_X = 0;     // 云台X轴偏移量，本程序未使用
volatile uint32_t SystemTickMs = 0;              // 系统毫秒时钟，TIM2中断累加

static volatile uint8_t CurrentMode = MODE_REQUIREMENT_3;    // 当前工作模式
static volatile uint8_t ControlEnabled = 0;                   // 控制器使能标志 0关闭 1开启
static volatile uint8_t VisionValid = 0;                     // 视觉数据有效标志
static volatile uint8_t Mode3Phase = MODE3_PHASE_TO_PLUS;     // Mode3状态机阶段
static volatile uint8_t Mode3ReachCount = 0;                  // 到达目标稳定帧数计数器
static volatile uint16_t BalanceTargetX = BALL_PLUS_5CM_X;    // 当前控制目标X像素
static volatile float BallFilteredX = 0.0f;                   // 滤波之后小球X坐标
static volatile float BallVelocityX = 0.0f;                   // 小球X方向速度，像素每秒
static volatile float CameraVelocityX = 0.0f;
static volatile float BalanceErrorX = 0.0f;                   // 位置误差=滤波小球坐标?目标坐标
static volatile float BalanceSpeedCommand = 0.0f;              // Final signed motor frequency command in Hz
static volatile float BalanceTargetSteps = 0.0f;
static volatile float BalanceIntegralSteps = 0.0f;
static volatile uint8_t BalanceBrakeHoldFrames = 0;
static volatile int8_t BalanceBrakeDirection = 0;
static volatile uint32_t Mode3CompleteMs = 0;

static uint32_t LastControlFrameId = 0;        // 上一次已经处理过的视觉帧ID，防止重复处理同一帧
static uint32_t LastSampleMs = 0;              // 上一次采样小球位置时间戳ms
static float LastFilteredX = 0.0f;             // 上一时刻滤波后的小球X，用于微分求速度

EMM_Motor BalanceMotor;                        // 云台电机结构体实例

/**
 * @brief 浮点数求绝对值
 */
static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 浮点数限幅函数，将value限制在low~high之间
 */
static float ClampFloat(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

/**
 * @brief float四舍五入转为int32_t整数
 */
static int32_t RoundFloat(float value)
{
    if (value >= 0.0f)
        return (int32_t)(value + 0.5f);
    return (int32_t)(value - 0.5f);
}

/**
 * @brief 判断视觉数据是否新鲜有效
 * @param target_x 小球X像素
 * @param last_rx_ms 接收该帧的时间戳
 * @retval 1有效，0过期或者无效
 */
static uint8_t TargetIsFresh(uint16_t target_x, uint32_t last_rx_ms)
{
    return target_x > 0 && target_x < BALL_COORDINATE_MAX &&
           (uint32_t)(SystemTickMs - last_rx_ms) <= TARGET_STALE_MS;
}

/**
 * @brief Reset vision estimates and cascade-control state.
 */
static void Balance_ResetEstimator(void)
{
    VisionValid = 0;
    LastControlFrameId = Serial_RxFrameCount;
    LastSampleMs = 0;
    LastFilteredX = 0.0f;
    BallFilteredX = 0.0f;
    BallVelocityX = 0.0f;
    CameraVelocityX = 0.0f;
    BalanceErrorX = 0.0f;
    BalanceSpeedCommand = 0.0f;
    BalanceTargetSteps = 0.0f;
    BalanceIntegralSteps = 0.0f;
    BalanceBrakeHoldFrames = 0;
    BalanceBrakeDirection = 0;
}

/**
 * @brief 启动平衡控制，初始化状态机，打开控制使能
 */
static void Balance_Start(void)
{
    __disable_irq();
    ControlEnabled = 0;
    Mode3CompleteMs = 0;
    Mode3ReachCount = 0;
    Mode3Phase = MODE3_PHASE_TO_PLUS;
    BalanceTargetX = (CurrentMode == MODE_REQUIREMENT_3) ?
                     BALL_PLUS_5CM_X : BALL_CENTER_X;
    Balance_ResetEstimator();
    EMM_Hold(&BalanceMotor);        // Keep the current pipe position while starting.
    ControlEnabled = 1;
    __enable_irq();
}

/**
 * @brief 关闭平衡控制，电机锁止，清空全部状态
 */
static void Balance_Stop(void)
{
    __disable_irq();
    ControlEnabled = 0;
    Balance_ResetEstimator();
    EMM_Hold(&BalanceMotor);
    __enable_irq();
}

static float PositionServo_CalculateSpeed(float target_steps,
                                          float fixed_speed,
                                          float stop_window)
{
    float position_error = target_steps - BalanceMotor.Position_Estimate;

    if (AbsFloat(position_error) <= stop_window)
        return 0.0f;
    return (position_error > 0.0f) ? fixed_speed : -fixed_speed;
}

static void Balance_UpdateTargetPosition(float dt_s)
{
    float abs_error = AbsFloat(BalanceErrorX);
    float predicted_error;
    float predicted_abs_error;
    float gain_ratio;
    float position_velocity_kp;
    float desired_velocity;
    float velocity_error;
    float control_effort;
    float step_gain;
    float raw_target;
    float brake_step_kp;
    float brake_predict_time_s;
    float brake_min_velocity;
    float brake_release_velocity;
    float brake_min_target_steps;
    uint8_t brake_hold_frames;
    float alpha = BALANCE_TARGET_FILTER_ALPHA;
    uint8_t brake_request;
    uint8_t braking = 0;
    int8_t motion_direction = 0;

    if (CurrentMode == MODE_REQUIREMENT_3)
    {
        if (Mode3Phase == MODE3_PHASE_TO_PLUS)
        {
            brake_step_kp = MODE3_PLUS_BRAKE_STEP_KP;
            brake_predict_time_s = MODE3_PLUS_BRAKE_PREDICT_TIME_S;
            brake_min_velocity = MODE3_PLUS_BRAKE_MIN_VELOCITY;
            brake_release_velocity = MODE3_PLUS_BRAKE_RELEASE_VELOCITY;
            brake_hold_frames = MODE3_PLUS_BRAKE_HOLD_FRAMES;
            brake_min_target_steps = MODE3_PLUS_BRAKE_MIN_TARGET_STEPS;
        }
        else
        {
            float taper_ratio;

            brake_step_kp = MODE3_MINUS_BRAKE_STEP_KP;
            brake_predict_time_s = MODE3_MINUS_BRAKE_PREDICT_TIME_S;
            brake_min_velocity = MODE3_MINUS_BRAKE_MIN_VELOCITY;
            brake_release_velocity = MODE3_MINUS_BRAKE_RELEASE_VELOCITY;
            brake_hold_frames = MODE3_MINUS_BRAKE_HOLD_FRAMES;

            /* Reduce only the forced minimum near -5; calculated high-speed braking remains available. */
            taper_ratio = ClampFloat(abs_error /
                                     MODE3_MINUS_BRAKE_TAPER_PIXELS,
                                     0.0f, 1.0f);
            brake_min_target_steps =
                MODE3_MINUS_BRAKE_MIN_TARGET_NEAR_STEPS +
                (MODE3_MINUS_BRAKE_MIN_TARGET_STEPS -
                 MODE3_MINUS_BRAKE_MIN_TARGET_NEAR_STEPS) * taper_ratio;
        }
    }
    else
    {
        brake_step_kp = BALANCE_BRAKE_STEP_KP;
        brake_predict_time_s = BALANCE_BRAKE_PREDICT_TIME_S;
        brake_min_velocity = BALANCE_BRAKE_MIN_VELOCITY;
        brake_release_velocity = BALANCE_BRAKE_RELEASE_VELOCITY;
        brake_hold_frames = BALANCE_BRAKE_HOLD_FRAMES;
        brake_min_target_steps = BALANCE_BRAKE_MIN_TARGET_STEPS;
    }

    /* Predict the ball position after camera, motor and linkage delay. */
    predicted_error = BalanceErrorX +
        BallVelocityX * brake_predict_time_s;
    predicted_error = ClampFloat(predicted_error,
                                 -(float)BALL_COORDINATE_MAX,
                                 (float)BALL_COORDINATE_MAX);
    predicted_abs_error = AbsFloat(predicted_error);

    gain_ratio = ClampFloat(predicted_abs_error /
                            BALANCE_GAIN_SCHEDULE_PIXELS,
                            0.0f, 1.0f);
    position_velocity_kp = BALANCE_POSITION_VEL_KP_NEAR +
        (BALANCE_POSITION_VEL_KP_FAR -
         BALANCE_POSITION_VEL_KP_NEAR) * gain_ratio;

    /* Prediction lowers and reverses the speed reference before arrival. */
    desired_velocity = -position_velocity_kp * predicted_error;
    desired_velocity = ClampFloat(desired_velocity,
                                  -BALANCE_DESIRED_VELOCITY_MAX,
                                  BALANCE_DESIRED_VELOCITY_MAX);
    velocity_error = desired_velocity - BallVelocityX;
    control_effort = -velocity_error;

    if (BallVelocityX > 0.0f)
        motion_direction = 1;
    else if (BallVelocityX < 0.0f)
        motion_direction = -1;

    /* Control effort with the same sign as velocity produces braking tilt. */
    brake_request =
        (AbsFloat(BallVelocityX) >= brake_min_velocity &&
         control_effort * BallVelocityX > 0.0f);
    if (CurrentMode == MODE_REQUIREMENT_3 &&
        Mode3Phase == MODE3_PHASE_TO_PLUS &&
        abs_error > MODE3_PLUS_BRAKE_ENABLE_PIXELS)
    {
        brake_request = 0;
        BalanceBrakeHoldFrames = 0;
        BalanceBrakeDirection = 0;
    }

    if (brake_request)
    {
        BalanceBrakeHoldFrames = brake_hold_frames;
        BalanceBrakeDirection = motion_direction;
        braking = 1;
    }
    else if (BalanceBrakeHoldFrames > 0 &&
             AbsFloat(BallVelocityX) >= brake_release_velocity &&
             motion_direction == BalanceBrakeDirection)
    {
        BalanceBrakeHoldFrames--;
        braking = 1;
    }
    else
    {
        BalanceBrakeHoldFrames = 0;
        BalanceBrakeDirection = 0;
    }

    step_gain = braking ? brake_step_kp :
                           BALANCE_ACCEL_STEP_KP;

    if (abs_error <= BALANCE_INTEGRAL_ZONE_PIXELS)
    {
        BalanceIntegralSteps +=
            BALANCE_POSITION_INTEGRAL_KI * BalanceErrorX * dt_s;
        BalanceIntegralSteps = ClampFloat(BalanceIntegralSteps,
                                          -BALANCE_INTEGRAL_MAX_STEPS,
                                          BALANCE_INTEGRAL_MAX_STEPS);
    }
    else
    {
        BalanceIntegralSteps *= 0.85f;
    }

    if (!braking &&
        abs_error <= BALANCE_CENTER_HOLD_PIXELS &&
        predicted_abs_error <= BALANCE_CENTER_HOLD_PIXELS &&
        AbsFloat(BallVelocityX) <= BALANCE_CENTER_HOLD_VELOCITY)
    {
        /* Keep the learned level offset instead of discarding static bias. */
        raw_target = BalanceIntegralSteps;
    }
    else
    {
        if (braking && control_effort * (float)BalanceBrakeDirection <= 0.0f)
            control_effort = (float)BalanceBrakeDirection *
                             AbsFloat(control_effort);
        raw_target = step_gain * control_effort + BalanceIntegralSteps;

        if (braking &&
            AbsFloat(raw_target) < brake_min_target_steps)
        {
            raw_target = (float)BalanceBrakeDirection *
                         brake_min_target_steps;
        }
    }

    if (CurrentMode == MODE_REQUIREMENT_3 &&
        Mode3Phase == MODE3_PHASE_TO_PLUS &&
        abs_error > MODE3_PLUS_BRAKE_ENABLE_PIXELS)
    {
        /* Keep driving toward +5 until the dedicated braking zone begins. */
        if (BalanceErrorX > 0.0f &&
            raw_target < MODE3_PLUS_DRIVE_MIN_TARGET_STEPS)
        {
            raw_target = MODE3_PLUS_DRIVE_MIN_TARGET_STEPS;
        }
        else if (BalanceErrorX < 0.0f &&
                 raw_target > -MODE3_PLUS_DRIVE_MIN_TARGET_STEPS)
        {
            raw_target = -MODE3_PLUS_DRIVE_MIN_TARGET_STEPS;
        }
    }

    raw_target *= BALANCE_MOTOR_SIGN;
    raw_target = ClampFloat(raw_target,
                            -BALANCE_TARGET_LIMIT_STEPS,
                            BALANCE_TARGET_LIMIT_STEPS);

    /* Braking target changes and direction reversals must take effect now. */
    if (braking)
        alpha = BALANCE_BRAKE_TARGET_ALPHA;
    if (raw_target * BalanceTargetSteps < 0.0f)
        alpha = 1.0f;

    BalanceTargetSteps +=
        (raw_target - BalanceTargetSteps) * alpha;
}

static void Mode3_UpdatePhase(void)
{
    if (Mode3Phase == MODE3_PHASE_TO_PLUS)
    {
        float plus_error = BallFilteredX - (float)BALL_PLUS_5CM_X;

        if (AbsFloat(plus_error) <= MODE3_PLUS_REACH_PIXELS ||
            (plus_error > MODE3_PLUS_REACH_PIXELS && BallVelocityX > 0.0f))
        {
            Mode3Phase = MODE3_PHASE_TO_MINUS;
            BalanceTargetX = BALL_MINUS_5CM_X;
            Mode3ReachCount = 0;
            BalanceIntegralSteps = 0.0f;
            BalanceBrakeHoldFrames = 0;
            BalanceBrakeDirection = 0;
        }
        return;
    }

    if (Mode3Phase == MODE3_PHASE_TO_MINUS)
    {
        if (AbsFloat(BallFilteredX - (float)BALL_MINUS_5CM_X) <=
                MODE3_FINAL_ERROR_PIXELS &&
            AbsFloat(BallVelocityX) <= MODE3_FINAL_VELOCITY)
        {
            if (Mode3ReachCount < MODE3_FINAL_STABLE_FRAMES)
                Mode3ReachCount++;
        }
        else
        {
            Mode3ReachCount = 0;
        }

        if (Mode3ReachCount >= MODE3_FINAL_STABLE_FRAMES)
        {
            Mode3Phase = MODE3_PHASE_HOLD_MINUS;
            Mode3CompleteMs = SystemTickMs;
        }
    }
}

/**
 * @brief Process a new vision frame and update position/velocity feedback.
 * @param target_x 原始小球X像素
 * @param frame_id 视觉帧序号
 * @param sample_ms 接收该帧时间戳ms
 */
static void Balance_ProcessNewFrame(uint16_t target_x,
                                     int32_t velocity_centi,
                                     uint32_t frame_id, uint32_t sample_ms)
{
    float raw_x = (float)target_x;
    float dt_s = 0.01f;
    float delta;
    float alpha;
    float measured_velocity = 0.0f;
    float camera_velocity;

    LastControlFrameId = frame_id;

    if (!VisionValid)
    {
        // 第一次收到有效视觉数据，初始化滤波器
        VisionValid = 1;
        BallFilteredX = raw_x;
        LastFilteredX = raw_x;
        CameraVelocityX = ClampFloat((float)velocity_centi * 0.01f,
                                         -MAX_MEASURED_VELOCITY,
                                         MAX_MEASURED_VELOCITY);
        BallVelocityX = CameraVelocityX;
        LastSampleMs = sample_ms;
    }
    else
    {
        uint32_t dt_ms = (uint32_t)(sample_ms - LastSampleMs);

        // 时间间隔合法换算成秒；异常间隔使用默认0.01s
        if (dt_ms >= 2 && dt_ms <= 100)
            dt_s = (float)dt_ms / 1000.0f;

        delta = raw_x - BallFilteredX;
        // 根据像素跳动大小选择滤波系数
        alpha = (AbsFloat(delta) >= POSITION_FILTER_FAR_PIXELS) ?
                POSITION_FILTER_ALPHA_FAR : POSITION_FILTER_ALPHA_NEAR;
        BallFilteredX += delta * alpha;

        // 差分求小球瞬时速度
        measured_velocity = (BallFilteredX - LastFilteredX) / dt_s;
        measured_velocity = ClampFloat(measured_velocity,
                                       -MAX_MEASURED_VELOCITY,
                                       MAX_MEASURED_VELOCITY);
        // 速度做一阶低通滤波
        camera_velocity = ClampFloat((float)velocity_centi * 0.01f,
                                      -MAX_MEASURED_VELOCITY,
                                      MAX_MEASURED_VELOCITY);
        CameraVelocityX += (camera_velocity - CameraVelocityX) *
                          VELOCITY_FILTER_ALPHA;
        BallVelocityX += (((CameraVelocityX * 0.75f) +
                          (measured_velocity * 0.25f)) - BallVelocityX) *
                         VELOCITY_FILTER_ALPHA;

        LastFilteredX = BallFilteredX;
        LastSampleMs = sample_ms;
    }

    if (CurrentMode == MODE_REQUIREMENT_3)
        Mode3_UpdatePhase();

    BalanceErrorX = BallFilteredX - (float)BalanceTargetX;
    Balance_UpdateTargetPosition(dt_s);
}

/**
 * @brief 5ms控制周期执行函数，读取视觉，执行控制输出
 */
static void Balance_ControlTick(void)
{
    uint16_t target_x;
    uint16_t target_y;
    int32_t velocity_centi;
    uint32_t last_rx_ms;
    uint32_t frame_id;

    Serial_ReadTarget(&target_x, &target_y, &velocity_centi,
                       &last_rx_ms, &frame_id);
    (void)target_y; // Y coordinate is not used.

    if (TargetIsFresh(target_x, last_rx_ms))
    {
        if (frame_id != LastControlFrameId)
            Balance_ProcessNewFrame(target_x, velocity_centi,
                                    frame_id, last_rx_ms);
    }
    else
    {
        Balance_ResetEstimator();
        EMM_Hold(&BalanceMotor);
        return;
    }

    /* All modes use the same fixed-speed motor position servo. */
    BalanceSpeedCommand =
        PositionServo_CalculateSpeed(BalanceTargetSteps,
                                     BALANCE_MOTOR_FIXED_SPEED_HZ,
                                     BALANCE_MOTOR_STOP_WINDOW_STEPS);
    EMM_Apply_Speed(&BalanceMotor, BalanceSpeedCommand,
                    (float)CONTROL_PERIOD_MS / 1000.0f);
}

/**
 * @brief Initialize the balance stepper motor.
 */
static void BalanceMotor_Init(void)
{
    // 步进电机引脚：PA11脉冲，PA10方向，PA12使能
    BalanceMotor.STEP_Port = GPIOA;
    BalanceMotor.STEP_Pin = GPIO_Pin_11;
    BalanceMotor.DIR_Port = GPIOA;
    BalanceMotor.DIR_Pin = GPIO_Pin_10;
    BalanceMotor.ENA_Port = GPIOA;
    BalanceMotor.ENA_Pin = GPIO_Pin_12;
    EMM_Motor_Init(&BalanceMotor);
}

/**
 * @brief OLED第一行打印当前工作模式
 */
static void OLED_ShowModeLine(void)
{
    if (CurrentMode == MODE_REQUIREMENT_3)
        OLED_ShowString(1, 1, "M1 Req3 +-5cm  ");
    else
        OLED_ShowString(1, 1, "M2 Req4/5 O    ");
}

/**
 * @brief OLED刷新全部状态，坐标、目标、误差、速度、运行状态
 */
static void OLED_ShowStatus(void)
{
    uint16_t shown_x = x;
    int32_t shown_error = VisionValid ? RoundFloat(BalanceErrorX) : 0;
    int32_t shown_velocity = VisionValid ? RoundFloat(BallVelocityX) : 0;

    // 数值截断，防止超出OLED显示位数
    if (shown_velocity > 9999)
        shown_velocity = 9999;
    if (shown_velocity < -9999)
        shown_velocity = -9999;

    OLED_ShowModeLine();
    OLED_ShowString(2, 1, "X:000 T:000     ");
    OLED_ShowNum(2, 3, shown_x, 3);
    OLED_ShowNum(2, 9, BalanceTargetX, 3);
    OLED_ShowString(3, 1, "E:+000 V:+0000  ");
    OLED_ShowSignedNum(3, 3, shown_error, 3);
    OLED_ShowSignedNum(3, 10, shown_velocity, 4);

    if (!ControlEnabled)
    {
        OLED_ShowString(4, 1, "STOP PC15=Start ");
    }
    else if (!VisionValid)
    {
        OLED_ShowString(4, 1, "RUN No Ball     ");
    }
    else if (CurrentMode == MODE_REQUIREMENT_4_5)
    {
        OLED_ShowString(4, 1, "RUN Hold Center ");
    }
    else if (Mode3Phase == MODE3_PHASE_TO_PLUS)
    {
        OLED_ShowString(4, 1, "RUN To +5cm     ");
    }
    else if (Mode3Phase == MODE3_PHASE_TO_MINUS)
    {
        OLED_ShowString(4, 1, "RUN To -5cm     ");
    }
    else if (Mode3CompleteMs == 0)
    {
        OLED_ShowString(4, 1, "RUN Hold -5cm   ");
    }
    else if ((uint32_t)(SystemTickMs - Mode3CompleteMs) <= 5000)
    {
        OLED_ShowString(4, 1, "DONE At -5cm    ");
    }
    else
    {
        OLED_ShowString(4, 1, "HOLD At -5cm    ");
    }
}

int main(void)
{
    uint32_t last_display_ms = 0;

    Serial_Init();      // 串口初始化，接收OpenMV视觉数据
    Key_Init();         // 按键初始化
    OLED_Init();        // OLED屏幕初始化
    STEP_PWM_Init();    // 步进电机PWM时钟初始化
    BalanceMotor_Init();// Initialize the balance stepper motor
    Timer_Init();       // TIM2定时器初始化，提供系统节拍与控制节拍
    OLED_ShowStatus();

    while (1)
    {
        KeyNum = Key_GetNum();

        if (KeyNum == 1)
        {   // 按键1切换模式，运行中先停止控制器
            if (ControlEnabled)
                Balance_Stop();
            CurrentMode = (CurrentMode == MODE_REQUIREMENT_3) ?
                          MODE_REQUIREMENT_4_5 : MODE_REQUIREMENT_3;
            BalanceTargetX = (CurrentMode == MODE_REQUIREMENT_3) ?
                             BALL_PLUS_5CM_X : BALL_CENTER_X;
            OLED_ShowStatus();
        }
        else if (KeyNum == 2 && !ControlEnabled)
        {   // 按键2启动控制，只有停止状态才能启动
            Balance_Start();
            OLED_ShowStatus();
        }

        // 到达显示周期刷新OLED屏幕
        if ((uint32_t)(SystemTickMs - last_display_ms) >= DISPLAY_PERIOD_MS)
        {
            last_display_ms = SystemTickMs;
            OLED_ShowStatus();
        }
    }
}

/**
 * @brief TIM2更新中断，系统毫秒节拍，产生控制任务调度
 */
void TIM2_IRQHandler(void)
{
    static uint8_t control_divider = 0;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        SystemTickMs++;        // 系统毫秒计时器自增
        Key_Tick();            // 按键扫描节拍

        control_divider++;
        if (control_divider >= CONTROL_PERIOD_MS)
        {
            control_divider = 0;
            if (ControlEnabled)
                Balance_ControlTick(); // 到达5ms执行一次控制逻辑
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
