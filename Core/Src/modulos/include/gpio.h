
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"
/**
 * @brief Inicializacion de hardware
 * 
*/
 void GPIO_Init(void);





void gpio_irq_on();

void gpio_irq_off();


#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
