/*
 * adc.c
 *
 *  Created on: Jul 29, 2024
 *      Author: shrikrishna
 */


#include "adc.h"

// Initialize ADC
void ADC_Init(void)
{
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->CR1 = ADC_CR1_RES_0; // 12-bit resolution
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT; // Enable ADC and continuous mode
    ADC1->SQR1 = 0; // Single channel
    ADC1->SQR3 = ADC_CHANNEL_0; // Default channel
}

// Read from a specified ADC channel
uint32_t ADC_Read(uint32_t channel)
{
    // Configure ADC to read from the specified channel
    ADC1->SQR3 = channel;

    // Start ADC Conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Return the ADC value
    return ADC1->DR;
}
