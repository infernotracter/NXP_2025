/*
 * uart.h
 *
 *  Created on: 2024Äê8ÔÂ28ÈÕ
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_UART_H_
#define CODE_UART_H_

void  send_wave              (uart_index_enum uartn);
float parse_float            (uint8* str);
void  uart_adjust_parameter  (void);


#endif /* CODE_UART_H_ */
