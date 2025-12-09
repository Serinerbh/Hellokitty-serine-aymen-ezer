/*
 * hc05_usart.h
 *
 *  Created on: Dec 3, 2025
 *      Author: Aymen
 */

#ifndef __HC05_USART_H
#define __HC05_USART_H

#include "stm32g4xx_hal.h" // Adaptez l'include selon votre série STM32 (ex: stm32f1xx_hal.h)

// Définition du périphérique USART utilisé (USART3)
#define HC05_UART_HANDLE  huart3

// Déclaration de l'instance UART qui doit être définie dans main.c ou ailleurs
// Si vous utilisez la couche HAL générée par CubeMX, elle est souvent nommée huartX.
extern UART_HandleTypeDef HC05_UART_HANDLE;

/**
 * @brief Initialise le périphérique USART3 (PB10/TX, PB11/RX) pour le HC-05.
 * @note La configuration des broches GPIO et de l'UART est généralement faite par
 * CubeMX, mais cette fonction peut contenir une initialisation manuelle.
 * @param BaudRate Le débit binaire (par défaut 9600 ou 38400 pour le mode AT)
 */
void HC05_Init(uint32_t BaudRate);

/**
 * @brief Envoie une chaîne de caractères via USART3.
 * @param pData Pointeur vers la chaîne de caractères (doit être terminée par '\0').
 */
void HC05_SendString(char *pData);

/**
 * @brief Tente de recevoir une chaîne de caractères via USART3.
 * @param pRxBuffer Pointeur vers le tampon de réception.
 * @param Size Taille maximale du tampon de réception.
 * @param Timeout Temps maximum d'attente en ms.
 * @return HAL_StatusTypeDef Statut de la réception.
 */
HAL_StatusTypeDef HC05_ReceiveString(char *pRxBuffer, uint16_t Size, uint32_t Timeout);

#endif /* __HC05_USART_H *//* INC_HC05_USART_H_ */
