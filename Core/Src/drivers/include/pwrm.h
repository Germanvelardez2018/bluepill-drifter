
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWRM_H
#define __PWRM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"


uint8_t pwrm_get_itime();

void prwm_set_itime(uint8_t itime);


/***
 * @brief Inicializa el modulo de gestion de consumo
 * 
*/
void pwrm_init();




void sleep_interval();

/**
 * @brief Espero por el GPS
*/
void wait_for_gps();


/***
 * @brief Espero que el modulo encienda
*/
void wait_for_sim();




#ifdef __cplusplus
}
#endif

#endif /* __PWRM_H */











