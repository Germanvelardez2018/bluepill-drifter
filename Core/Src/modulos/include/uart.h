/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"



/***
 * Activo las interrupciones del UART1.
 * Se utilizan para capturar los comandos que llegan del broker mqtt
*/
void uart1_set_irq();

/**
 * Desactivo las interrupciones de UART1
*/
void uart1_clear_irq();


/***
 * Inicio el hardware UART1
*/
void MX_USART1_UART_Init(void);

/**
 * Inicio el hardware UART2
*/
void MX_USART2_UART_Init(void);



#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
