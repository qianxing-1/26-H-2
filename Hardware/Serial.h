#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

extern volatile uint32_t Serial_LastRxMs;
extern volatile uint32_t Serial_RxFrameCount;

void Serial_Init(void);
void Serial_SendByte(uint8_t byte);
void Serial_SendArray(uint8_t *array, uint16_t length);
void Serial_SendString(char *string);
void Serial_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);
void Serial_ReadTarget(uint16_t *target_x, uint16_t *target_y,
                       int32_t *velocity_centi,
                       uint32_t *last_rx_ms, uint32_t *frame_id);

#endif
