/*
 * frame_uart.h
 *
 *  Created on: Apr 22, 2023
 *      Author: DELL
 */

#ifndef FRAME_UART_H_
#define FRAME_UART_H_
#include "stdio.h"
#include "main.h"
typedef enum
{
    FRAME_OK = 0,
    FRAME_MISS,
    FRAME_ERROR

} frame_uart_t;

#define START_BYTE 69
#define CHECK_BYTE 96
#define STOP_BYTE 196

#define FRAME_DATA_TX 8
#define FRAME_DATA_RX 9
#define FRAME_DATA_RX_HANDLE FRAME_DATA_RX * 4 // STOP + 2CRC + STOP
#define FRAME_DATA_TX_HANDLE FRAME_DATA_TX * 2 + 4

// #define FRAME_DATA_RX       FRAME_DATA_TX + 4

void SendFrameData(uint8_t *pu8Src, uint16_t u16Src_len, uint8_t *pu8Dest, uint16_t *pu16Dest_len);
frame_uart_t GetFrameData(uint8_t *pu8Src, uint16_t u16Src_len, uint8_t *pu8Dest);
#endif /* FRAME_UART_H_ */
