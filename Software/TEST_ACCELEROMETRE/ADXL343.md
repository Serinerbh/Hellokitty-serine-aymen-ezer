# ADXL343 Driver for STM32

Ce document contient le code source du pilote pour l'accéléromètre ADXL343, basé sur la librairie STM32 HAL.

## 1. Explication du fonctionnement

L'accéléromètre ADXL343 communique avec le microcontrôleur via le bus **I2C**.

### Initialisation (`ADXL343_Init`)
1.  **Vérification de l'ID** : Le pilote lit le registre `ADXL_DEVID` (0x00) pour s'assurer que le capteur est bien connecté. La valeur attendue est `0xE5`.
2.  **Configuration du débit** : Le registre `ADXL_BW_RATE` est configuré pour un débit de données de 100 Hz (`0x0A`).
3.  **Format des données** : Le registre `ADXL_DATA_FORMAT` est configuré pour une résolution complète (Full Resolution) et une plage de mesure de ±2g.
4.  **Activation** : Le bit de mesure est activé dans le registre `ADXL_POWER_CTL` pour démarrer les mesures.

### Lecture des axes (`ADXL343_ReadAxes`)
Le pilote lit 6 octets consécutifs à partir du registre `ADXL_DATAX0` (0x32).
-   `DATAX0` & `DATAX1` : Axe X
-   `DATAY0` & `DATAY1` : Axe Y
-   `DATAZ0` & `DATAZ1` : Axe Z
Ces valeurs brutes sont ensuite combinées pour former des entiers signés de 16 bits (`int16_t`).

### Détection de Choc (Tap Detection)
-   **Configuration (`ADXL343_ConfigShock`)** : Définit le seuil (`THRESH_TAP`) et la durée (`DUR`) pour qu'un mouvement soit considéré comme un "tap" (choc). Active également les interruptions pour "Single Tap".
-   **Vérification (`ADXL343_CheckShock`)** : Lit le registre `ADXL_INT_SOURCE` pour voir si le bit `SINGLE_TAP` est levé, indiquant qu'un choc a été détecté.

---

## 2. Fichiers Sources

### `ADXL343_driver.h`

```c
/*
 * ADXL343_driver.h
 *
 *  Created on: Nov 18, 2025
 *      Author: maram
 */

#ifndef ADXL343_H
#define ADXL343_H

#include "main.h"
#include <stdint.h>

// Adresse I2C de l'ADXL343 (ALT ADDRESS pin à la masse)
#define ADXL343_ADDR            (0x53 << 1)

// Registres ADXL343
#define ADXL_DEVID              0x00
#define ADXL_THRESH_TAP         0x1D
#define ADXL_DUR                0x21
#define ADXL_LATENT             0x22
#define ADXL_WINDOW             0x23
#define ADXL_TAP_AXES           0x2A
#define ADXL_BW_RATE            0x2C
#define ADXL_POWER_CTL          0x2D
#define ADXL_INT_ENABLE         0x2E
#define ADXL_INT_MAP            0x2F
#define ADXL_INT_SOURCE         0x30
#define ADXL_DATA_FORMAT        0x31
#define ADXL_DATAX0             0x32

#define ADXL_POWER_MEASURE      (1 << 3)

#define ADXL_FULL_RES           (1 << 3)
#define ADXL_RANGE_2G           0x00
#define ADXL_RANGE_4G           0x01
#define ADXL_RANGE_8G           0x02
#define ADXL_RANGE_16G          0x03

#define ADXL_DATA_RATE_100HZ    0x0A

#define ADXL_INT_DATA_READY     (1 << 7)
#define ADXL_INT_SINGLE_TAP     (1 << 6)

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} adxl343_axes_t;

// Prototypes des fonctions
uint8_t ADXL343_Init(I2C_HandleTypeDef *hi2c);
uint8_t ADXL343_ReadAxes(I2C_HandleTypeDef *hi2c, adxl343_axes_t *axes);
uint8_t ADXL343_EnableSingleShock(I2C_HandleTypeDef *hi2c, float threshold_g, float duration_ms); // Note: Non implémenté dans le .c fourni, voir ConfigShock
HAL_StatusTypeDef ADXL343_ConfigShock(I2C_HandleTypeDef *hi2c, float thresh_g, float dur_ms);
uint8_t ADXL343_CheckShock(I2C_HandleTypeDef *hi2c);
void ADXL343_PrintAxes(I2C_HandleTypeDef *hi2c);
float ADXL343_ComputeTotalG(float xg, float yg, float zg);

// Conversion inline Valeur Brute -> g (pour ±2g Full Res, facteur de l'échelle est ~3.9 mg/LSB)
static inline float ADXL343_RawTo_g(int16_t raw_value)
{
	return ((float)raw_value) * 0.0039f;
}

#endif
```

### `ADXL343_driver.c`

```c
/*
 * ADXL343_driver.c
 *
 *  Created on: Nov 18, 2025
 *      Author: maram
 */

#include "ADXL343_driver.h"
#include <math.h>
#include <stdio.h> // Nécessaire pour printf

// Fonction utilitaire d'écriture I2C
static HAL_StatusTypeDef adxl_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(hi2c,
			ADXL343_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			&value,
			1,
			HAL_MAX_DELAY);
}

// Fonction utilitaire de lecture I2C
static HAL_StatusTypeDef adxl_read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
	return HAL_I2C_Mem_Read(hi2c,
			ADXL343_ADDR,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			value,
			1,
			HAL_MAX_DELAY);
}

// Initialisation de l'ADXL343
uint8_t ADXL343_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t devid = 0;

	// Vérifier l'ID du capteur (Doit être 0xE5)
	if (adxl_read(hi2c, ADXL_DEVID, &devid) != HAL_OK || devid != 0xE5)
		return 0; // Echec
	
	// Configurer le débit à 100 Hz
	if (adxl_write(hi2c, ADXL_BW_RATE, ADXL_DATA_RATE_100HZ) != HAL_OK)
		return 0;
	
	// Configurer le format de données (Full Res, ±2g)
	uint8_t data_format = ADXL_FULL_RES | ADXL_RANGE_2G;
	if (adxl_write(hi2c, ADXL_DATA_FORMAT, data_format) != HAL_OK)
		return 0;
	
	// Activer le mode mesure
	if (adxl_write(hi2c, ADXL_POWER_CTL, ADXL_POWER_MEASURE) != HAL_OK)
		return 0;

	return 1; // Succès
}

// Lecture des axes X, Y, Z
uint8_t ADXL343_ReadAxes(I2C_HandleTypeDef *hi2c, adxl343_axes_t *axes)
{
	uint8_t buf[6];

	// Lecture des 6 registres de données (0x32 à 0x37)
	if (HAL_I2C_Mem_Read(hi2c, ADXL343_ADDR, ADXL_DATAX0, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY) != HAL_OK)
		return 0;

	// Reconstruction des valeurs 16 bits (LSB d'abord)
	axes->x = (int16_t)((buf[1] << 8) | buf[0]);
	axes->y = (int16_t)((buf[3] << 8) | buf[2]);
	axes->z = (int16_t)((buf[5] << 8) | buf[4]);

	return 1;
}

// Configuration de la détection de choc (Tap)
HAL_StatusTypeDef ADXL343_ConfigShock(I2C_HandleTypeDef *hi2c, float thresh_g, float dur_ms)
{
	HAL_StatusTypeDef ret;
	
	// Configuration du seuil de choc
	// Le facteur d'échelle pour THRESH_TAP est 62.5 mg/LSB
	const float lsb_g = 0.0625f;
	uint8_t thresh = (uint8_t)(thresh_g / lsb_g);
	if (thresh == 0) thresh = 1;

	ret = adxl_write(hi2c, ADXL_THRESH_TAP, thresh);
	if (ret != HAL_OK) return ret;
	
	// Configuration de la durée du choc
	// Le facteur d'échelle pour DUR est 625 µs/LSB (0.625 ms)
	const float lsb_ms = 0.625f;
	uint8_t dur = (uint8_t)(dur_ms / lsb_ms);
	if (dur == 0) dur = 1;

	ret = adxl_write(hi2c,  ADXL_DUR, dur);
	if (ret != HAL_OK) return ret;

	// Activation des 3 axes pour la détection de tap
	ret = adxl_write(hi2c,  ADXL_TAP_AXES, 0x07); // X,Y,Z
	if (ret != HAL_OK) return ret;

	// Activation de l'interruption SINGLE_TAP
	uint8_t int_enable = ADXL_INT_DATA_READY | ADXL_INT_SINGLE_TAP;
	ret = adxl_write(hi2c, ADXL_INT_ENABLE, int_enable);
	if (ret != HAL_OK) return ret;
	
	// Mappage de l'interruption (Optionnel selon cablage)
	uint8_t int_map = 0x40; // Map vers INT2 par exemple
	ret = adxl_write(hi2c, ADXL_INT_MAP, int_map);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}

// Vérification si un choc a eu lieu
uint8_t ADXL343_CheckShock(I2C_HandleTypeDef *hi2c)
{
	uint8_t src = 0;
	// Lecture du registre de source d'interruption
	// Note: La lecture efface les bits d'interruption
	adxl_read(hi2c, ADXL_INT_SOURCE, &src);

	return (src & ADXL_INT_SINGLE_TAP) ? 1 : 0;
}

// Calcul de l'accélération totale (norme du vecteur)
float ADXL343_ComputeTotalG(float xg, float yg, float zg)
{
	return sqrtf((xg * xg) + (yg * yg) + (zg * zg));
}

// Affichage des résultats (nécessite la redirection de printf)
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
```

## 3. Exemple d'utilisation (`main.c`)

Voici comment intégrer le pilote dans votre fonction `main()`.

```c
/* Includes */
#include "ADXL343_driver.h"
// ... autres includes

int main(void)
{
  /* ... Initialisation HAL, Clock, etc ... */
  
  MX_I2C1_Init(); // Assurez-vous que l'I2C est initialisé
  
  /* ... Initialisation UART pour printf ... */

  printf("\r\nTest Accéléromètre ADXL343\r\n");

  // 1. Initialisation du capteur
  if (ADXL343_Init(&hi2c1) != 1) { // Remplacez &hi2c1 par votre instance I2C
      printf("Erreur Init ADXL343 !\r\n");
      while(1); // Bloquer en cas d'erreur
  } else {
      printf("ADXL343 Init OK\r\n");
  }

  // 2. Configuration de la détection de choc (Seuil 0.3g, Durée 50ms)
  if (ADXL343_ConfigShock(&hi2c1, 0.3f, 50.0f) != HAL_OK) {
      printf("Erreur Config Shock !\r\n");
  } else {
      printf("Config Shock OK\r\n");
  }

  while (1)
  {
      // 3. Vérifier s'il y a un choc
      if (ADXL343_CheckShock(&hi2c1))
      {
          printf("CHOC DETECTE !\r\n");
          // Action : Clignoter une LED, etc.
      }

      // 4. Afficher les données d'accélération
      ADXL343_PrintAxes(&hi2c1);

      HAL_Delay(100);
  }
}
```
