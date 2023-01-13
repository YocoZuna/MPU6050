/*
 * MPU6050.c
 *
 *  Created on: Jan 10, 2023
 *      Author: Dawid Zadlo
 */

#include "i2c.h"
#include <stdint.h>
#include "MPU6050.h"




static void MPU6050_Get_Gyro_RAW(I2C_HandleTypeDef* I2C,float* gyroBuff);
static void MPU6050_Get_Acc_RAW(I2C_HandleTypeDef* I2C,float* acc);

void MPU6050_Get_Temp(I2C_HandleTypeDef* I2C,float * tempr);
void MPU6050_Init(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050);
void MPU6050_Get_Acc_Value(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050,float* accvalue);
void MPU6050_Get_Gyro_Value(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050,float* gyrovalue);
void MPU6050_Get_Temp_Value(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050,float* tempr);
/** @ MPU6050_Init
  * @{
  */

/**
  * @brief  Initialization of MPU6050 register
  *
  *
  * @param
  * I2C_HandleTypeDef I2C -> handler to I2C hardware
  * MPU6050_Config_TypeDef mpu6050 - > handler to MPU6050 configuration structure
  * @retval None
  *
	  * typedef struct
	{
		uint8_t PIN_PULL;
		uint8_t PIN_SIGNAL;
		uint8_t ISR_CLEAR_ON;


	}MPU6050_Interrupt_Config_Typdef;


	typedef struct
	{
		uint8_t CLOCK;
		uint8_t FILTER;
		uint8_t TEMP_ON_OFF;
		MPU6050_Interrupt_Config_Typdef Interrupt_Config;

	}MPU6050_Config_TypeDef;
	  *
  */
void MPU6050_Init(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050)
{
	uint8_t temp = 0;

	/* Check if MPU6050 is present under 0x68 slave address */
	HAL_I2C_Mem_Read(I2C, MPU6050_DEV_ADDRESS, MPU6050_WHOAMI, 1, &temp, 1, 1000);
	if (temp == 0x68)
	{
		/* Restart of the device */
		HAL_I2C_Mem_Write(I2C, MPU6050_DEV_ADDRESS, MPU6050_POWER_MANAGMENT_1, 1,(uint8_t*) MPU6050_DEVICE_RESET, 1, 1000);
		HAL_Delay(100);
		/* Initialization of clock and tempr sensor */
		if (mpu6050->TEMP_ON_OFF == DISABLE)
		{
			temp  |= mpu6050->CLOCK + MPU6050_TEMP_DIS;
		}
		else
		{
			temp  = mpu6050->CLOCK;
		}
		/* Restart all sensors */
		HAL_I2C_Mem_Write(I2C, MPU6050_DEV_ADDRESS, MPU6050_USER_CONTROL, 1,(uint8_t*) MPU6050_USER_RESET_ALL_SENS, 1, 1000);
		/* Set lowpass filter ad dpfl */
		temp = mpu6050->FILTER;
		HAL_I2C_Mem_Write(I2C, MPU6050_DEV_ADDRESS, MPU6050_LOW_PASS_FILTER, 1,&temp, 1, 1000);

		/* Setting range for accelerometer and gyroscope */
		temp = mpu6050->ACC_RANGE;
		HAL_I2C_Mem_Write(I2C, MPU6050_DEV_ADDRESS, MPU6050_ACC_CONFIG, 1,&temp, 1, 1000);
		temp = mpu6050->GYRO_RANGE;
		HAL_I2C_Mem_Write(I2C, MPU6050_DEV_ADDRESS, MPU6050_GYRO_CONFIG, 1,&temp, 1, 1000);



		/*
		 * TODO Configuration of Interrupts
		 */
	}
}

/** @ MPU6050_Get_Gyro_RAW
  * @{
  */

/**
  * @brief  Getting raw data from gyroscope measurment data register
  *
  *
  * @param
  * I2C_HandleTypeDef I2C -> handler to I2C hardware
  * int16_t* gyroBuff -> pointer to gyroscope data array
  * where [0] = X axis, [1] = Y axis, [2] = Z axis
  * @retval None
  */
static void MPU6050_Get_Gyro_RAW(I2C_HandleTypeDef* I2C,float* gyroBuff)
{

	uint8_t temp[6];
	HAL_I2C_Mem_Read(I2C, MPU6050_DEV_ADDRESS, MPU6050_GYRO_MEAS, 1, temp, 6, 1000);

	gyroBuff[0] = (int16_t) (temp[1]<<8) | temp[0];
	gyroBuff[1] = (int16_t) (temp[3]<<8) | temp[1];
	gyroBuff[2] = (int16_t) (temp[5]<<8) | temp[4];

}

/** @ MPU6050_Get_Acc_RAW
  * @{
  */

/**
  * @brief  Getting raw data from gyroscope measurment data register
  *
  *
  * @param
  * I2C_HandleTypeDef I2C -> handler to I2C hardware
  * int16_t* accBuff -> pointer to acceleroscope data array
  * where [0] = X axis, [1] = Y axis, [2] = Z axis
  * @retval None
  */
static void MPU6050_Get_Acc_RAW(I2C_HandleTypeDef* I2C,float* accBuff)
{

	uint8_t temp[6];
	HAL_I2C_Mem_Read(I2C, MPU6050_DEV_ADDRESS, MPU6050_ACC_MEAS, 1, temp, 6, 1000);

	accBuff[0] = (int16_t) (temp[1]<<8) | temp[0];
	accBuff[1] = (int16_t) (temp[3]<<8) | temp[1];
	accBuff[2] = (int16_t) (temp[5]<<8) | temp[4];

}

/** @ MPU6050_Get_Temp
  * @{
  */

/**
  * @brief  Getting  data from temperature measurment data register
  *
  *
  * @param
  * I2C_HandleTypeDef I2C -> handler to I2C hardware
  * int16_t* tempr -> pointer to temperature variable
  * @retval None
  */
void MPU6050_Get_Temp(I2C_HandleTypeDef* I2C,float * tempr)
{

	uint8_t temp[2];
	HAL_I2C_Mem_Read(I2C, MPU6050_DEV_ADDRESS, MPU6050_TEMP, 1, temp, 2, 1000);

	*tempr = (int16_t) (temp[1]<<8) | temp[0];

}

void MPU6050_Get_Acc_Value(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050,float* accvalue)
{
	float accBuff[3];
	assert_param(sizeof(accvalue)==12);
	MPU6050_Get_Acc_RAW(I2C, accBuff);
	if (mpu6050->ACC_RANGE == MPU6050_ACC_AFS_2G)
		for ( int i=0;i<3;i++)
		{
			accvalue[i]  = accBuff[i]/16384;
		}
	if (mpu6050->ACC_RANGE == MPU6050_ACC_AFS_4G)
		for ( int i=0;i<3;i++)
		{
			accvalue[i]  = accBuff[i]/8192;
		}
	if (mpu6050->ACC_RANGE == MPU6050_ACC_AFS_8G)
		for ( int i=0;i<3;i++)
		{
			accvalue[i]  = accBuff[i]/4096;
		}
	if (mpu6050->ACC_RANGE == MPU6050_ACC_AFS_16G)
		for ( int i=0;i<3;i++)
		{
			accvalue[i]  = accBuff[i]/2048;
		}

}
void MPU6050_Get_Gyro_Value(I2C_HandleTypeDef* I2C,MPU6050_Config_TypeDef* mpu6050,float* gyrovalue)
{
	float gyroBuff[3];
	assert_param(sizeof(gyrovalue)==12);
	MPU6050_Get_Gyro_RAW(I2C, gyroBuff);
	if (mpu6050->GYRO_RANGE == MPU6050_GYRO_FS_250)
		for ( int i=0;i<3;i++)
		{
			gyrovalue[i]  = gyroBuff[i]/131;
		}
	if (mpu6050->GYRO_RANGE == MPU6050_GYRO_FS_500)
		for ( int i=0;i<3;i++)
		{
			gyrovalue[i]  = gyroBuff[i]/65.5;
		}
	if (mpu6050->GYRO_RANGE == MPU6050_GYRO_FS_1000)
		for ( int i=0;i<3;i++)
		{
			gyrovalue[i]  = gyroBuff[i]/32.8;
		}
	if (mpu6050->GYRO_RANGE == MPU6050_GYRO_FS_2000)
		for ( int i=0;i<3;i++)
		{
			gyrovalue[i]  = gyroBuff[i]/16.4;
		}

}


