#ifndef __EMM_GIMBAL_H
#define __EMM_GIMBAL_H

#include "stm32f10x.h"

/* Positive speed is clockwise/up; negative speed is counterclockwise/down. */
typedef struct
{
    GPIO_TypeDef *STEP_Port;
    uint16_t STEP_Pin;

    GPIO_TypeDef *DIR_Port;
    uint16_t DIR_Pin;

    GPIO_TypeDef *ENA_Port;
    uint16_t ENA_Pin;

    float Step_Frequency;
    float Signed_Frequency;
    float Position_Estimate;
    uint8_t Direction;
    uint8_t Reverse_Pending;
} EMM_Motor;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Integral;
    float Last_Error;
    float Integral_Limit;
    float Max_Output;
    float Min_Output;
} PID_Controller;

void EMM_Motor_Init(EMM_Motor *motor);
void EMM_Enable(EMM_Motor *motor);
void EMM_Disable(EMM_Motor *motor);
void EMM_Hold(EMM_Motor *motor);
void EMM_Set_Direction(EMM_Motor *motor, uint8_t dir);
void EMM_Apply_Speed(EMM_Motor *motor, float signed_frequency, float dt_s);

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float max, float min);
void PID_Reset(PID_Controller *pid);
float PID_Calculate(PID_Controller *pid, float error,
                    float error_velocity, float dt_s);

#endif
