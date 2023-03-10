

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLOCK_MASTER_H
#define __CLOCK_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"



/**
 *  Configura el clock principal del dispositivo
 *  @note Opcion 8, 4 2 y 1 Mhz. Se debe reinicializar perifericos (ejemplo uart,spi, etc) 
 *          cada vez que cambia el clock
 * @param clock_param_t clock a utilizar.
*/

void clock_master_set(clock_param_t clock_param_t);
#ifdef __cplusplus
}
#endif

#endif /* __CLOCK_MASTER_H */
