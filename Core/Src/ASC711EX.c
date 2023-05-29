/*
 * ASC711EX.c
 *
 *  Created on: May 17, 2023
 *      Author: dawid
 */

#include "ACS711EX.h"
#include "adc.h"

void ACS711EX_Convert_To_mA( uint16_t * bufforTemp,float* buffor)
{
	static float tempBufforADC[3];

	for ( int i =0; i <3; i ++)
	{
		tempBufforADC[i] = ((float)bufforTemp[i]/4095)*3.3;
		tempBufforADC[i]=  (73.3*(tempBufforADC[i]/3.3)-36.7);
		buffor[i] = (float)tempBufforADC[i];


	}

}
float* ACS711EX_Calib(const uint16_t *const bufforTemp)
{
	static float tempBufforADC[3];

	for ( int i =0; i <3; i ++)
	{
		tempBufforADC[i] = ((float)bufforTemp[i]/4095)*3.3;
		tempBufforADC[i]=  (73.3*(tempBufforADC[i]/3.3)-36.7);



	}
	return tempBufforADC;
}

/*
static inline void ACS711EX_Set_Active_Channel(const ADC_HandleTypeDef *hadc, const uint32_t channel)
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = channel;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_ADC_Start(hadc);
}

uint16_t  ACS711EX_Avrg_From_Reading( ADC_HandleTypeDef *hadc,uint16_t loops,const uint32_t Channel)
{
	uint32_t temp=0;
	ACS711EX_Set_Active_Channel(hadc, Channel);
	for (uint16_t i = 0; i < loops; i++)
	{
		//HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, 1);
		temp += HAL_ADC_GetValue(hadc);


	}
	HAL_ADC_Stop(hadc);
	return (temp/loops);
}
*/


