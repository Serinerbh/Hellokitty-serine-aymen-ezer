/*
 * ADXL343_driver.C
 *
 *  Created on: Nov 18, 2025
 *      Author: maram
 */

#include "ADXL343_driver.h"
#include <math.h>
#include <stdio.h>

static HAL_StatusTypeDef adxl_write(I2C_HandleTypeDef *hi2c,uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(hi2c,
			ADXL343_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			HAL_MAX_DELAY);
}

static HAL_StatusTypeDef adxl_read(I2C_HandleTypeDef *hi2c,uint8_t reg, uint8_t *value)
{
	return HAL_I2C_Mem_Read(hi2c,
			ADXL343_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			value,
			1,
			HAL_MAX_DELAY);
}

//initialisation

uint8_t ADXL343_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t devid = 0;

	// Vérifier l'ID du capteur
	if (adxl_read(hi2c, ADXL_DEVID, &devid) != HAL_OK || devid != 0xE5)
		return 0;
	// 100 Hz
	if (adxl_write(hi2c, ADXL_BW_RATE, ADXL_DATA_RATE_100HZ) != HAL_OK)
		return 0;
	// FULL RES + ±2g
	uint8_t data_format = ADXL_FULL_RES | ADXL_RANGE_2G;
	if (adxl_write(hi2c, ADXL_DATA_FORMAT, data_format) != HAL_OK)
		return 0;
	// Mode mesure
	if (adxl_write(hi2c, ADXL_POWER_CTL, ADXL_POWER_MEASURE) != HAL_OK)
		return 0;

	return 1;
}

//lecture des axes, de lacceleration et de la difference d'acceleration

uint8_t ADXL343_ReadAxes(I2C_HandleTypeDef *hi2c, adxl343_axes_t *axes)
{
	uint8_t buf[6];

	if (HAL_I2C_Mem_Read(hi2c, ADXL343_ADDR, ADXL_DATAX0,I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY) != HAL_OK)
		return 0;

	axes->x = (int16_t)((buf[1] << 8) | buf[0]);
	axes->y = (int16_t)((buf[3] << 8) | buf[2]);
	axes->z = (int16_t)((buf[5] << 8) | buf[4]);

	return 1;
}

//detection de choc

HAL_StatusTypeDef ADXL343_ConfigShock(I2C_HandleTypeDef *hi2c,float thresh_g,float dur_ms)
{
	HAL_StatusTypeDef ret;
	//choc
	const float lsb_g = 0.0625f;
	uint8_t thresh = (uint8_t)(thresh_g / lsb_g);
	if (thresh == 0) thresh = 1;

	ret = adxl_write(hi2c, ADXL_THRESH_TAP, thresh);
	if (ret != HAL_OK) return ret;
	//durée
	const float lsb_ms = 0.625f;
	uint8_t dur = (uint8_t)(dur_ms / lsb_ms);
	if (dur == 0) dur = 1;

	ret = adxl_write(hi2c,  ADXL_DUR, dur);
	if (ret != HAL_OK) return ret;

	//activation des 3 axes pour la detection
	ret = adxl_write(hi2c,  ADXL_TAP_AXES, 0x07); // X,Y,Z
	if (ret != HAL_OK) return ret;

	//enable l'interruption de l'adx
	uint8_t int_enable = ADXL_INT_DATA_READY | ADXL_INT_SINGLE_TAP;
	ret = adxl_write(hi2c, ADXL_INT_ENABLE, int_enable);
	if (ret != HAL_OK) return ret;
	uint8_t int_map = 0x40;
	ret = adxl_write(hi2c, ADXL_INT_MAP, int_map);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}

//vérifier le choc

uint8_t ADXL343_CheckShock(I2C_HandleTypeDef *hi2c)
{
	uint8_t src = 0;
	adxl_read(hi2c, ADXL_INT_SOURCE, &src);

	return (src & ADXL_INT_SINGLE_TAP) ? 1 : 0;
}

//calculer l'acceleration
float ADXL343_ComputeTotalG(float xg, float yg, float zg)
{
	return sqrtf((xg * xg) + (yg * yg) + (zg * zg));
}

float ADXL343_RawTo_g(int16_t raw_value)
{
    // Pour une plage de ±2g, la sensibilité est de 256 LSB/g
    return (float)raw_value / 256.0f;
}

//afficher les resultats sur teraterm
void ADXL343_PrintAxes(I2C_HandleTypeDef *hi2c)
{
	static float prevA = 1.0f;  // mémorise l'accélération précédente

	adxl343_axes_t axes;

	if (!ADXL343_ReadAxes(hi2c, &axes)) {
		printf("Erreur lecture XYZ\r\n");
		return;
	}

	float xg = ADXL343_RawTo_g(axes.x);
	float yg = ADXL343_RawTo_g(axes.y);
	float zg = ADXL343_RawTo_g(axes.z);

	float Atot = ADXL343_ComputeTotalG(xg, yg, zg);
	float deltaA = fabsf(Atot - prevA);

	prevA = Atot;  // mise à jour pour prochaine itération

	printf("X=%.3f g  Y=%.3f g  Z=%.3f g  |  A=%.3f g  |  dA=%.3f g\r\n",
			xg, yg, zg, Atot, deltaA);
}





