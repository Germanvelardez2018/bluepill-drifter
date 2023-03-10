/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"


/**
*@brief Inicializo el periferico que soporta el debug: UART2 por default
*/
void debug_init();


/**
*@brief Desinicializo el periferico que soporta el debug: UART2 por default
*/
void  debug_deinit();


/**
 * @brief Imprimo mensaje en formato string por el puerto de debug de la app (UART2 DEFAULT)
*/
void debug_print(uint8_t* buffer);


/**
 * @brief Imprimo mensaje en formato uint8_t por el puerto de debug de la app (UART2 DEFAULT)
*/
void debug_print_raw(uint8_t* buffer,size_t len);


#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */
