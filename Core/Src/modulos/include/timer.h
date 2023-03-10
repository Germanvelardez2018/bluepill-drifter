
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif




/* Includes ------------------------------------------------------------------*/

#include "core.h"


/**
  * @brief Inicializo el hardware del timer 1
  * @param None
  * @retval None
  */
 void TIM1_Init(void);
/**
  * @brief Inicializo el hardware del timer 2
  * @param None
  * @retval None
  */
 void TIM3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
