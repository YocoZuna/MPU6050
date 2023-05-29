/*
 * ACS711EX.h
 *
 *  Created on: May 17, 2023
 *      Author: dawid
 */

#ifndef INC_ACS711EX_H_
#define INC_ACS711EX_H_

#include "main.h"


void ACS711EX_Convert_To_mA( uint16_t * bufforTemp,float* buffor);

float* ACS711EX_Calib(const uint16_t *const bufforTemp);
//int16_t ACS711EX_Avrg_From_Reading( ADC_HandleTypeDef *hadc,uint16_t loops,const uint32_t Channel);
//float* ACS711EX_Calibrate_ADC(const uint16_t *const bufforTemp);


#endif /* INC_ACS711EX_H_ */
