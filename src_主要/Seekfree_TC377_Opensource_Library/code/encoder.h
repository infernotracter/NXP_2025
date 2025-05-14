/*
 * encoder.h
 *
 *  Created on: 2024Äê8ÔÂ29ÈÕ
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#define ENCODER_1                   (TIM5_ENCODER)
#define ENCODER_1_A                 (TIM5_ENCODER_CH1_P10_3)
#define ENCODER_1_B                 (TIM5_ENCODER_CH2_P10_1)

#define ENCODER_2                   (TIM2_ENCODER)
#define ENCODER_2_A                 (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_2_B                 (TIM2_ENCODER_CH2_P33_6)

extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern int16 encoder_data_temp;

void encoder_init    (void);
void encoder_data_get(void);

#endif /* CODE_ENCODER_H_ */
