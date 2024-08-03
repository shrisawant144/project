/*
 * adc.h
 *
 *  Created on: Jul 29, 2024
 *      Author: shrikrishna
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#ifndef ADC_H
#define ADC_H

#include "stm32f4xx.h"

// Function prototypes
void ADC_Init(void);
uint32_t ADC_Read(uint32_t channel);

#endif // ADC_H


#endif /* INC_ADC_H_ */
