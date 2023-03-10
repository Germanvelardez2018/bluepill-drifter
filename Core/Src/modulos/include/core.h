/****
 *  Archivo con las definiciones nucleo de la aplicacion
 *
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CORE_H
#define __CORE_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* Includes ------------------------------------------------------------------*/

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "debug.h"

/*-----------------------------------------------------------------------------*/

/* Defines ------------------------------------------------------------------*/
#define PRIVATE static

// Macros

// BIT(x) regresa el bit x puesto a uno y los demas bits en cero, ej. BIT(3) regresa 00001000
#define BIT(x) (1 << (x))
// BIT_GET(x,b) regresa el bit b-esimo de x ej. BIT_GET(PINC,3)
#define BIT_GET(x, b) ((x)&BIT(b))
// BIT_SET(x,b) establece en '1' el bit b de x ej. BIT_SET(PORTD,4)
#define BIT_SET(x, b) ((x) |= BIT(b))
// BIT_CLEAR(x,b) establece a '0' el bit b de x
#define BIT_CLEAR(x, b) ((x) &= ~BIT(b))
// BIT_TOGGLE(x,b) invierte el valor del bit b de x a su complemento,
#define BIT_TOGGLE(x, b) ((x) ^= BIT(b))
// BIT_WRITE(x,b,v) establece el valor 'v' de
#define BIT_WRITE(x, b, v) ((v) ? BIT_SET(x, b) : BIT_CLEAR(x, b))

// ES_PAR(x) regresa 0 cuando x no es par y algo diferente de 0 cuando es par
#define ES_PAR(x) (!BIT_GET(x, 0))
// Private functions
#define PRIVATE static

    //! Usado en clock master
    typedef enum
    {
        CLOCK_05MHZ = RCC_SYSCLK_DIV512,
        CLOCK_1MHZ = RCC_SYSCLK_DIV8,
        CLOCK_2MHZ = RCC_SYSCLK_DIV4,
        CLOCK_4MHZ = RCC_SYSCLK_DIV2,
        CLOCK_8MHZ = RCC_SYSCLK_DIV1

    } clock_param_t;

    typedef enum
    {
        LEVEL_LOW = 0,
        LEVEL_HIGH
    } level_t;

    typedef enum
    {
        LOW_LEVEL = 0,
        NORMAL_LEVEL = 1
    } battery_state_t;


    
  #define  FSM_CHECK_ONLY      ((uint8_t) 0)
  #define  FSM_SAVE_DATA       ((uint8_t) 1)
  #define  FSM_UPLOAD          ((uint8_t) 2)
   




    typedef enum{
        SIZE_PAGE_256 = 0,
        SIZE_PAGE_264 = 1
    } size_page_t;


    typedef enum
    {
        SIM_OFF,
        SIM_ON

    } sim_t;

    typedef enum
    {
        CONN_4G_OFF,
        CONN_4G_ON
    } internet_state_t;

    typedef enum
    {
        MQTT_DISCONNECTED,
        MQTT_CONNECTED
    } mqtt_state_t;

    typedef enum
    {
        GPS_OFF,
        GPS_ON
    } gps_state_t;

    typedef enum
    {
        SLEEP,
        RUN
    } mcu_state_t;









#define delay(t)                             HAL_Delay(t)


#ifdef __cplusplus
}
#endif

#endif /* __CORE_H */