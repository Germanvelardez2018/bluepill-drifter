


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"




/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
 void SPI_Init(void);


/**
 * @brief SPI deinit
*/
void SPI_deinit();
#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */