
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"




/**
  * @brief I2C2 Inicializacion de hardware
  * @param None
  * @retval None
  */
 void I2C_Init(void);


/***
 * @brief I2C2 deinit
*/
void I2C_Deinit();

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
