#include "stm32f10x.h"
#include "Timer.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "step_pwm.h"
#include "EMM_Gimbal.h"

/* 相机坐标以及±5cm任务像素标定 */
#define BALL_CENTER_X                286        // 小球处于中心位置对应的相机X像素
#define BALL_PLUS_5CM_X              425  // 小球向右偏移+5cm目标像素
#define BALL_MINUS_5CM_X             138  // 小球向左偏移?5cm目标像素
#define BALL_COORDINATE_MAX          640        // 相机X轴像素最大值

#define CONTROL_PERIOD_MS              5        // 控制循环周期 5ms
#define TARGET_STALE_MS              120        // 视觉数据超时时间，超过该时间认为目标丢失
#define DISPLAY_PERIOD_MS             80        // OLED屏幕刷新周期

/* 平衡控制PID参数；输出正数代表电机顺时针转动 */
#define BALANCE_KP                   6.0f       // PID比例系数
#define BALANCE_KI                   0.05f      // PID积分系数
#define BALANCE_KD                   2.20f      // PID微分系数
#define BALANCE_INTEGRAL_LIMIT     300.0f       // 积分限幅，防止积分饱和
#define BALANCE_MAX_SPEED_HZ        1500.0f       // 电机最大脉冲输出频率，限制最高转速
#define BALANCE_MIN_COMMAND_HZ       24.0f      // 电机最小动作频率，低于该值不输出，消除微弱抖动
#define BALANCE_POSITION_DEADBAND      4.0f     // 位置死区，误差小于该值停止控制输出
#define BALANCE_VELOCITY_DEADBAND     35.0f     // 速度死区，小球速度小于该值停止控制输出
#define BALANCE_MOTOR_SIGN             1.0f     // 电机方向符号，可±1用来反转转向
#define BALANCE_PREDICT_TIME_S        0.38f
#define BALANCE_BRAKE_MIN_VELOCITY    50.0f
/* Mode 1 trajectory tuning: 400 steps = 1 mm, about 1745 steps = 1 degree. */
#define MODE1_HEIGHT_SIGN                1.0f
#define MODE1_ACCEL_HEIGHT_STEPS      1800.0f
#define MODE1_BRAKE_HEIGHT_STEPS     -2200.0f
#define MODE1_LEVEL_HEIGHT_STEPS         0.0f
#define MODE1_ACCEL_SPEED_HZ          2800.0f
#define MODE1_BRAKE_SPEED_HZ          3200.0f
#define MODE1_LEVEL_SPEED_HZ          2600.0f
#define MODE1_MOTOR_POSITION_KP          8.0f
#define MODE1_MOTOR_DEADBAND_STEPS       6.0f
/* Switch early so inertia carries the ball to the requested endpoints. */
#define MODE1_START_ERROR_PIXELS       12.0f
#define MODE1_START_VELOCITY           35.0f
#define MODE1_SWITCH_TO_BRAKE_X       385.0f
#define MODE1_SWITCH_TO_LEVEL_X       185.0f
#define MODE1_SWITCH_MIN_VELOCITY      20.0f
#define MODE1_FINAL_ERROR_PIXELS        8.0f
#define MODE1_FINAL_VELOCITY           35.0f
#define MODE1_FINAL_STABLE_FRAMES         8
#define MODE1_ACCEL_TIMEOUT_MS         3000
#define MODE1_BRAKE_TIMEOUT_MS         3000
/* STM32端滤波，抑制相机单像素抖动；大小跳变使用不同滤波系数 */
#define POSITION_FILTER_ALPHA_NEAR    0.48f    // 小球位置变化小时，滤波系数，数值越小滤波越强
#define POSITION_FILTER_ALPHA_FAR     0.78f    // 小球位置变化大时，滤波系数，响应更快
#define POSITION_FILTER_FAR_PIXELS   12.0f     // 像素变化超过该阈值切换快速滤波
#define VELOCITY_FILTER_ALPHA         0.60f    // 小球速度一阶低通滤波系数
#define MAX_MEASURED_VELOCITY       2500.0f     // 小球测量速度最大限幅，抑制异常跳变

/* 任务模式3要求：先到达+5cm，再反向稳定到?5cm */
#define MODE_REQUIREMENT_3               1     // 模式3：+5cm再到?5cm往复任务
#define MODE_REQUIREMENT_4_5             2     // 模式4/5：小球保持在中心

#define MODE1_PHASE_WAIT_START            3
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
static volatile float BalanceErrorX = 0.0f;                   // 位置误差=滤波小球坐标?目标坐标
static volatile float BalanceSpeedCommand = 0.0f;              // PID输出电机转速指令Hz
static volatile uint32_t Mode3CompleteMs = 0;
static volatile uint32_t Mode1PhaseStartMs = 0;                 // Mode3整套任务完成时间戳

static uint32_t LastControlFrameId = 0;        // 上一次已经处理过的视觉帧ID，防止重复处理同一帧
static uint32_t LastSampleMs = 0;              // 上一次采样小球位置时间戳ms
static float LastFilteredX = 0.0f;             // 上一时刻滤波后的小球X，用于微分求速度

EMM_Motor BalanceMotor;                        // 云台电机结构体实例
PID_Controller BalancePID;                     // PID控制器实例

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
 * @brief 重置观测器、滤波器、PID；丢失小球/停止控制调用
 */
static void Balance_ResetEstimator(void)
{
    VisionValid = 0;
    LastControlFrameId = Serial_RxFrameCount;
    LastSampleMs = 0;
    LastFilteredX = 0.0f;
    BallFilteredX = 0.0f;
    BallVelocityX = 0.0f;
    BalanceErrorX = 0.0f;
    BalanceSpeedCommand = 0.0f;
    PID_Reset(&BalancePID);   // 清空PID积分、微分项
}

/**
 * @brief 启动平衡控制，初始化状态机，打开控制使能
 */
static void Balance_Start(void)
{
    __disable_irq();
    ControlEnabled = 0;
    Mode3CompleteMs = 0;
    Mode1PhaseStartMs = SystemTickMs;
    Mode3ReachCount = 0;
    Mode3Phase = MODE1_PHASE_WAIT_START;
    BalanceTargetX = BALL_CENTER_X;
    if (CurrentMode == MODE_REQUIREMENT_3)
        BalanceMotor.Position_Estimate = 0.0f;
    Balance_ResetEstimator();
    EMM_Hold(&BalanceMotor);        // 电机锁止保持
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

/**
 * @brief Mode3状态机跳转判断，满足条件切换下一阶段
 */
static float Mode1_TargetHeightSteps(void)
{
    float target;
    if (Mode3Phase == MODE3_PHASE_TO_PLUS)
        target = MODE1_ACCEL_HEIGHT_STEPS;
    else if (Mode3Phase == MODE3_PHASE_TO_MINUS)
        target = MODE1_BRAKE_HEIGHT_STEPS;
    else
        target = MODE1_LEVEL_HEIGHT_STEPS;
    return target * MODE1_HEIGHT_SIGN;
}

static float Mode1_CalculateMotorSpeed(void)
{
    float position_error;
    float max_speed;
    float output;
    position_error = Mode1_TargetHeightSteps() - BalanceMotor.Position_Estimate;
    if (AbsFloat(position_error) <= MODE1_MOTOR_DEADBAND_STEPS)
        return 0.0f;
    if (Mode3Phase == MODE3_PHASE_TO_PLUS)
        max_speed = MODE1_ACCEL_SPEED_HZ;
    else if (Mode3Phase == MODE3_PHASE_TO_MINUS)
        max_speed = MODE1_BRAKE_SPEED_HZ;
    else
        max_speed = MODE1_LEVEL_SPEED_HZ;
    output = position_error * MODE1_MOTOR_POSITION_KP;
    return ClampFloat(output, -max_speed, max_speed);
}

static void Mode1_UpdatePhase(void)
{
    uint32_t phase_elapsed_ms = SystemTickMs - Mode1PhaseStartMs;

    if (Mode3Phase == MODE1_PHASE_WAIT_START)
    {
        if (AbsFloat(BallFilteredX - (float)BALL_CENTER_X) <=
                MODE1_START_ERROR_PIXELS &&
            AbsFloat(BallVelocityX) <= MODE1_START_VELOCITY)
        {
            Mode3Phase = MODE3_PHASE_TO_PLUS;
            BalanceTargetX = BALL_PLUS_5CM_X;
            Mode1PhaseStartMs = SystemTickMs;
        }
        return;
    }

    if (Mode3Phase == MODE3_PHASE_TO_PLUS)
    {
        if ((BallFilteredX >= MODE1_SWITCH_TO_BRAKE_X &&
             BallVelocityX >= MODE1_SWITCH_MIN_VELOCITY) ||
            phase_elapsed_ms >= MODE1_ACCEL_TIMEOUT_MS)
        {
            Mode3Phase = MODE3_PHASE_TO_MINUS;
            BalanceTargetX = BALL_MINUS_5CM_X;
            Mode1PhaseStartMs = SystemTickMs;
            Mode3ReachCount = 0;
        }
        return;
    }
    if (Mode3Phase == MODE3_PHASE_TO_MINUS)
    {
        if ((BallFilteredX <= MODE1_SWITCH_TO_LEVEL_X &&
             BallVelocityX <= -MODE1_SWITCH_MIN_VELOCITY) ||
            phase_elapsed_ms >= MODE1_BRAKE_TIMEOUT_MS)
        {
            Mode3Phase = MODE3_PHASE_HOLD_MINUS;
            BalanceTargetX = BALL_MINUS_5CM_X;
            Mode1PhaseStartMs = SystemTickMs;
            Mode3ReachCount = 0;
        }
        return;
    }
    if (AbsFloat(BallFilteredX - (float)BALL_MINUS_5CM_X) <=
            MODE1_FINAL_ERROR_PIXELS &&
        AbsFloat(BallVelocityX) <= MODE1_FINAL_VELOCITY)
    {
        if (Mode3ReachCount < MODE1_FINAL_STABLE_FRAMES)
            Mode3ReachCount++;
    }
    else
    {
        Mode3ReachCount = 0;
    }
    if (Mode3ReachCount >= MODE1_FINAL_STABLE_FRAMES && Mode3CompleteMs == 0)
        Mode3CompleteMs = SystemTickMs;
}

/**
 * @brief 处理一帧新视觉数据：滤波、速度估算、状态机、PID运算
 * @param target_x 原始小球X像素
 * @param frame_id 视觉帧序号
 * @param sample_ms 接收该帧时间戳ms
 */
static void Balance_ProcessNewFrame(uint16_t target_x,
                                    uint32_t frame_id, uint32_t sample_ms)
{
    float raw_x = (float)target_x;
    float dt_s = 0.01f;
    float delta;
    float alpha;
    float measured_velocity = 0.0f;
    float output;
    float control_error;

    LastControlFrameId = frame_id;

    if (!VisionValid)
    {
        if (CurrentMode == MODE_REQUIREMENT_3)
            Mode1PhaseStartMs = sample_ms;

        // 第一次收到有效视觉数据，初始化滤波器
        VisionValid = 1;
        BallFilteredX = raw_x;
        LastFilteredX = raw_x;
        BallVelocityX = 0.0f;
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
        BallVelocityX += (measured_velocity - BallVelocityX) *
                         VELOCITY_FILTER_ALPHA;

        LastFilteredX = BallFilteredX;
        LastSampleMs = sample_ms;
    }

    if (CurrentMode == MODE_REQUIREMENT_3)
        Mode1_UpdatePhase();

    BalanceErrorX = BallFilteredX - (float)BalanceTargetX;

    /* Mode 1 uses its own three-stage motor position trajectory. */
    if (CurrentMode == MODE_REQUIREMENT_3)
        return;
    control_error = BalanceErrorX;

    /* Predict crossing only while the ball is moving toward the target. */
    if (BalanceErrorX * BallVelocityX < 0.0f &&
        AbsFloat(BallVelocityX) >= BALANCE_BRAKE_MIN_VELOCITY)
    {
        control_error += BallVelocityX * BALANCE_PREDICT_TIME_S;
    }

    // 进入死区：位置、速度都很小，停止输出，积分做衰减抑制饱和
    if (AbsFloat(BalanceErrorX) <= BALANCE_POSITION_DEADBAND &&
        AbsFloat(BallVelocityX) <= BALANCE_VELOCITY_DEADBAND)
    {
        BalancePID.Integral *= 0.85f;
        BalanceSpeedCommand = 0.0f;
        return;
    }

    /* D直接使用小球测量速度，切换目标不会产生微分冲击 */
    output = PID_Calculate(&BalancePID, control_error,
                           BallVelocityX, dt_s);
    output *= BALANCE_MOTOR_SIGN;

    // 小于最小输出阈值直接清零，消除电机微弱抖动
    if (AbsFloat(output) < BALANCE_MIN_COMMAND_HZ)
        output = 0.0f;

    BalanceSpeedCommand = output;
}

/**
 * @brief 5ms控制周期执行函数，读取视觉，执行控制输出
 */
static void Balance_ControlTick(void)
{
    uint16_t target_x;
    uint16_t target_y;
    uint32_t last_rx_ms;
    uint32_t frame_id;

    Serial_ReadTarget(&target_x, &target_y, &last_rx_ms, &frame_id);
    (void)target_y; // Y坐标本程序没有使用

    if (TargetIsFresh(target_x, last_rx_ms))
    {
        if (frame_id != LastControlFrameId)   // 只处理没有处理过的新帧
            Balance_ProcessNewFrame(target_x, frame_id, last_rx_ms);
    }
    else
    {
        // 视觉数据失效，重置控制器，电机锁止
        Balance_ResetEstimator();
        EMM_Hold(&BalanceMotor);
        return;
    }

    // 将速度指令下发电机驱动
    /* The Mode 1 motor position loop runs every 5 ms. */
    if (CurrentMode == MODE_REQUIREMENT_3)
        BalanceSpeedCommand = Mode1_CalculateMotorSpeed();

    EMM_Apply_Speed(&BalanceMotor, BalanceSpeedCommand,
                    (float)CONTROL_PERIOD_MS / 1000.0f);
}

/**
 * @brief 云台电机与PID控制器初始化
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

    // PID初始化，配置参数与输出限幅
    PID_Init(&BalancePID, BALANCE_KP, BALANCE_KI, BALANCE_KD,
             BALANCE_INTEGRAL_LIMIT,
             BALANCE_MAX_SPEED_HZ, -BALANCE_MAX_SPEED_HZ);
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
    else if (Mode3Phase == MODE1_PHASE_WAIT_START)
    {
        OLED_ShowString(4, 1, "WAIT Ball Center");
    }
    else if (Mode3Phase == MODE3_PHASE_TO_PLUS)
    {
        OLED_ShowString(4, 1, "RUN Raise/Accel ");
    }
    else if (Mode3Phase == MODE3_PHASE_TO_MINUS)
    {
        OLED_ShowString(4, 1, "RUN Reverse/Brk ");
    }
    else if (Mode3CompleteMs == 0)
    {
        OLED_ShowString(4, 1, "RUN Level/Coast ");
    }
    else if ((uint32_t)(SystemTickMs - Mode3CompleteMs) <= 5000)
    {
        OLED_ShowString(4, 1, "DONE At -5cm    ");
    }
    else
    {
        OLED_ShowString(4, 1, "HOLD Level      ");
    }
}

int main(void)
{
    uint32_t last_display_ms = 0;

    Serial_Init();      // 串口初始化，接收OpenMV视觉数据
    Key_Init();         // 按键初始化
    OLED_Init();        // OLED屏幕初始化
    STEP_PWM_Init();    // 步进电机PWM时钟初始化
    BalanceMotor_Init();// 云台电机、PID初始化
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
