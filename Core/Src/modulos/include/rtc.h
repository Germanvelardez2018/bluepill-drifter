
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H
#define __RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"

/**
 *  @brief Configura la hora, minutos y segundos del RTC
*/
void rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);


/**
 *  Obtengo la hora, minutos y segundos del RTC
*/
void rtc_get_time(uint8_t* hours, uint8_t* minutes, uint8_t* seconds);


/**
 * @brief Configurar alarma
*/
void rtc_set_alarm(uint8_t hours, uint8_t minutes, uint8_t seconds);


/**
 * @brief Inicio el RTC. Incluye inicializacion de hardware y set time inicial
*/
void rtc_init();

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H */

#include "core.h"