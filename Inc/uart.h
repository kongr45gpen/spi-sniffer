/*
 * uart.h
 *
 *  Created on: Oct 17, 2018
 *      Author: kongr45gpen
 */

#ifndef UART_H_
#define UART_H_

#include <stdarg.h>

extern UART_HandleTypeDef huart2;

void UART_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[256];

    vsnprintf(buffer, 256, format, args);

    HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);

    va_end(args);
}

#endif /* UART_H_ */
