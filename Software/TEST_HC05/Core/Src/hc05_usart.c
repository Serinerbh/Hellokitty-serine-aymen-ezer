/*
 * hc05_usart.c
 *
 *  Created on: Dec 3, 2025
 *      Author: Aymen
 */

#include "hc05_usart.h"
#include <string.h> // Pour la fonction strlen

// Si votre huart3 est déclarée dans main.c (ou ailleurs), vous devez la déclarer ici
// pour pouvoir l'utiliser. La ligne 'extern' dans le .h gère cela.

/**
 * @brief Initialisation du module HC-05.
 * @param BaudRate Le débit binaire.
 */
void HC05_Init(uint32_t BaudRate)
{
    // Si vous n'utilisez pas CubeMX, vous devez initialiser ici :
    // 1. Activation de l'horloge des GPIO B.
    // 2. Configuration de PB10 (TX) et PB11 (RX) en mode Alternate Function (AF).
    // 3. Activation de l'horloge de l'USART3.
    // 4. Configuration des paramètres de l'USART3 (BaudRate, 8N1, etc.).

    // Si vous utilisez CubeMX, l'instance huart3 est déjà configurée.
    // Vous pouvez optionnellement reconfigurer le BaudRate ici si besoin :
    HC05_UART_HANDLE.Init.BaudRate = BaudRate;
    HAL_UART_Init(&HC05_UART_HANDLE);
}

/**
 * @brief Envoie une chaîne de caractères via USART3.
 * @param pData Pointeur vers la chaîne de caractères.
 */
void HC05_SendString(char *pData)
{
    uint16_t len = strlen(pData);
    // Utilisation de HAL_UART_Transmit en mode polling
    HAL_UART_Transmit(&HC05_UART_HANDLE, (uint8_t*)pData, len, 100); // Timeout de 100ms
}

/**
 * @brief Tente de recevoir une chaîne de caractères via USART3 (mode Polling).
 * @note Cette fonction ne gère pas de délimiteur de fin de chaîne (comme '\n' ou '\r')
 * et attend que la taille "Size" soit atteinte ou que le timeout soit écoulé.
 * Pour une implémentation plus pratique, les interruptions sont préférables.
 * @param pRxBuffer Pointeur vers le tampon de réception.
 * @param Size Taille maximale du tampon de réception.
 * @param Timeout Temps maximum d'attente en ms.
 * @return HAL_StatusTypeDef Statut de la réception.
 */
HAL_StatusTypeDef HC05_ReceiveString(char *pRxBuffer, uint16_t Size, uint32_t Timeout)
{
    // Utilisation de HAL_UART_Receive en mode polling
    HAL_StatusTypeDef status = HAL_UART_Receive(&HC05_UART_HANDLE, (uint8_t*)pRxBuffer, Size, Timeout);

    // En cas de succès, assurez-vous de terminer la chaîne par '\0'
    if (status == HAL_OK)
    {
        pRxBuffer[Size - 1] = '\0'; // Assurez-vous que la dernière position est '\0'
    }

    return status;
}

// Pour une gestion plus réaliste de la réception (détection de fin de ligne ou de caractère),
// on utiliserait les interruptions (HAL_UART_Receive_IT) ou le DMA, en implémentant
// la callback de réception : void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart).
