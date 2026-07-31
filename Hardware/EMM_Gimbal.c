#include "step_pwm.h"
#include "EMM_Gimbal.h"

/* Motor tuning: one control tick is normally 5 ms. */
#define TRACK_MIN_SPEED_HZ         100.0f
#define TRACK_START_SPEED_HZ      500.0f
#define TRACK_MAX_SPEED_HZ       1800.0f
#define TRACK_ACCEL_HZ_PER_S    100000.0f
#define TRACK_DECEL_HZ_PER_S    300000.0f

/* Relative pulse limit from the level power-up position; tune to the linkage. */
#define TRACK_POSITION_LIMIT       420.0f

#define MOTOR_DIR_CW_UP             0
#define MOTOR_DIR_CCW_DOWN          1

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float ClampFloat(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

void EMM_Motor_Init(EMM_Motor *motor)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = motor->DIR_Pin;
    GPIO_Init(motor->DIR_Port, &gpio);

    gpio.GPIO_Pin = motor->ENA_Pin;
    GPIO_Init(motor->ENA_Port, &gpio);

    motor->Direction = MOTOR_DIR_CW_UP;
    motor->Step_Frequency = 0.0f;
    motor->Signed_Frequency = 0.0f;
    motor->Position_Estimate = 0.0f;
    motor->Reverse_Pending = 0;
    EMM_Set_Direction(motor, MOTOR_DIR_CW_UP);
    EMM_Disable(motor);
}

void EMM_Enable(EMM_Motor *motor)
{
    GPIO_ResetBits(motor->ENA_Port, motor->ENA_Pin);
}

void EMM_Disable(EMM_Motor *motor)
{
    STEP_PWM_SetFreq(0);
    motor->Step_Frequency = 0.0f;
    motor->Signed_Frequency = 0.0f;
    motor->Reverse_Pending = 0;
    GPIO_SetBits(motor->ENA_Port, motor->ENA_Pin);
}

void EMM_Hold(EMM_Motor *motor)
{
    STEP_PWM_SetFreq(0);
    motor->Step_Frequency = 0.0f;
    motor->Signed_Frequency = 0.0f;
    motor->Reverse_Pending = 0;
    EMM_Enable(motor);
}

void EMM_Set_Direction(EMM_Motor *motor, uint8_t dir)
{
    motor->Direction = dir;
    if (dir == MOTOR_DIR_CCW_DOWN)
        GPIO_ResetBits(motor->DIR_Port, motor->DIR_Pin);
    else
        GPIO_SetBits(motor->DIR_Port, motor->DIR_Pin);
}

void EMM_Apply_Speed(EMM_Motor *motor, float signed_frequency, float dt_s)
{
    float current;
    float target;
    float next;
    float accel_step;
    float decel_step;
    uint8_t desired_direction;

    if (dt_s <= 0.0f || dt_s > 0.1f)
        dt_s = 0.005f;

    /* Count commanded pulses so a lost/stuck ball cannot drive the slider forever. */
    motor->Position_Estimate += motor->Signed_Frequency * dt_s;
    motor->Position_Estimate = ClampFloat(motor->Position_Estimate,
                                           -TRACK_POSITION_LIMIT,
                                           TRACK_POSITION_LIMIT);

    target = ClampFloat(signed_frequency,
                        -TRACK_MAX_SPEED_HZ, TRACK_MAX_SPEED_HZ);

    if ((motor->Position_Estimate >= TRACK_POSITION_LIMIT && target > 0.0f) ||
        (motor->Position_Estimate <= -TRACK_POSITION_LIMIT && target < 0.0f))
    {
        target = 0.0f;
    }

    if (AbsFloat(target) < TRACK_MIN_SPEED_HZ)
        target = 0.0f;

    current = motor->Signed_Frequency;
    accel_step = TRACK_ACCEL_HZ_PER_S * dt_s;
    decel_step = TRACK_DECEL_HZ_PER_S * dt_s;

    /* Stop STEP and change DIR; the next tick restarts directly at target speed. */
    if (current != 0.0f && target != 0.0f &&
        ((current > 0.0f) != (target > 0.0f)))
    {
        EMM_Hold(motor);
        desired_direction = (target > 0.0f) ?
                            MOTOR_DIR_CW_UP : MOTOR_DIR_CCW_DOWN;
        EMM_Set_Direction(motor, desired_direction);
        motor->Reverse_Pending = 1;
        return;
    }

    if (target == 0.0f)
    {
        if (AbsFloat(current) <= decel_step)
        {
            EMM_Hold(motor);
        }
        else
        {
            next = (current > 0.0f) ?
                   (current - decel_step) : (current + decel_step);
            motor->Signed_Frequency = next;
            motor->Step_Frequency = AbsFloat(next);
            STEP_PWM_SetFreq((uint32_t)motor->Step_Frequency);
        }
        return;
    }

    desired_direction = (target > 0.0f) ?
                        MOTOR_DIR_CW_UP : MOTOR_DIR_CCW_DOWN;
    if (current == 0.0f && desired_direction != motor->Direction)
    {
        EMM_Set_Direction(motor, desired_direction);
        motor->Reverse_Pending = 1;
        return;
    }

    if (current == 0.0f)
    {
        float start_speed = AbsFloat(target);

        if (!motor->Reverse_Pending && start_speed > TRACK_START_SPEED_HZ)
            start_speed = TRACK_START_SPEED_HZ;
        if (start_speed < TRACK_MIN_SPEED_HZ)
            start_speed = TRACK_MIN_SPEED_HZ;
        next = (target > 0.0f) ? start_speed : -start_speed;
        motor->Reverse_Pending = 0;
    }
    else if (AbsFloat(target) > AbsFloat(current))
    {
        float delta = target - current;

        if (delta > accel_step)
            delta = accel_step;
        if (delta < -accel_step)
            delta = -accel_step;
        next = current + delta;
    }
    else
    {
        float delta = target - current;

        if (delta > decel_step)
            delta = decel_step;
        if (delta < -decel_step)
            delta = -decel_step;
        next = current + delta;
    }

    if (AbsFloat(next) < TRACK_MIN_SPEED_HZ)
    {
        EMM_Hold(motor);
        return;
    }

    EMM_Enable(motor);
    motor->Signed_Frequency = next;
    motor->Step_Frequency = AbsFloat(next);
    STEP_PWM_SetFreq((uint32_t)motor->Step_Frequency);
}

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float max, float min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Integral_Limit = integral_limit;
    pid->Max_Output = max;
    pid->Min_Output = min;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    pid->Integral = 0.0f;
    pid->Last_Error = 0.0f;
}

float PID_Calculate(PID_Controller *pid, float error,
                    float error_velocity, float dt_s)
{
    float candidate_integral;
    float output;

    if (dt_s <= 0.0f || dt_s > 0.25f)
        dt_s = 0.01f;

    candidate_integral = pid->Integral + error * dt_s;
    candidate_integral = ClampFloat(candidate_integral,
                                    -pid->Integral_Limit,
                                    pid->Integral_Limit);

    output = pid->Kp * error +
             pid->Ki * candidate_integral +
             pid->Kd * error_velocity;

    /* Do not wind the integral farther into an output limit. */
    if (!((output > pid->Max_Output && error > 0.0f) ||
          (output < pid->Min_Output && error < 0.0f)))
    {
        pid->Integral = candidate_integral;
    }

    output = ClampFloat(output, pid->Min_Output, pid->Max_Output);
    pid->Last_Error = error;
    return output;
}
