/*
 * adc_driver.c
 *
 *  Created on: May 6, 2025
 *      Author: Bowen
 */

#include "adc_driver.h"


extern ADC_HandleTypeDef hadc1;
extern osSemaphoreId_t adcSemHandle;
uint16_t adc_buffer[ADC_CHANNEL_COUNT];
uint16_t filtered_adc_buffer[ADC_CHANNEL_COUNT];

bool ADC_ReadAverage(uint16_t*, TickType_t);

/**
  * @brief  Get ADC reading with moving average filter
  * @param  *avg_buffer Pointer to filtered buffer
  * @param  timeout_ms Timeout to capture each reading in millisecond
  * @retval Is success
  */
bool ADC_ReadAverage(uint16_t *avg_buffer, TickType_t timeout_ms)
{
  uint32_t sum_buffer[ADC_CHANNEL_COUNT] = {0};

  for (uint32_t i = 0; i < ADC_FILTER_SIZE; ++i) {
		(void)osSemaphoreAcquire(adcSemHandle, 0);

		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNEL_COUNT) != HAL_OK) {
			return false;
		}
		// wait until ISR signals
		if (osSemaphoreAcquire(adcSemHandle, pdMS_TO_TICKS(timeout_ms)) != osOK) {
			HAL_ADC_Stop_DMA(&hadc1);
			return false;
		}
		HAL_ADC_Stop_DMA(&hadc1);

		for (uint32_t ch = 0; ch < ADC_CHANNEL_COUNT; ++ch) {
			sum_buffer[ch] += adc_buffer[ch];
		}
  }

  for (uint32_t ch = 0; ch < ADC_CHANNEL_COUNT; ++ch) {
  	avg_buffer[ch] = (uint16_t)(sum_buffer[ch] / ADC_FILTER_SIZE);
  }
  return true;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && adcSemHandle != NULL)
    {
    	osSemaphoreRelease(adcSemHandle);
    }
}
