/**
 * @file mmap.h
 * @author German Velardez (gvelardez@inti.gob.ar)
 * @brief  Mapa de memoria
 * @version 0.1
 * @date 2022-08-24
 * 
 * @copyright Copyright (c) 2022
 * 
 *
 */









/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAP_MEMORY_H
#define __MAP_MEMORY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"

/**
 * 
 * @note La memoria AT45DB041E tiene 2048 paginas de 256/264bytes cada una.
         El buffer 1 se usara en las funciones de escritur  
         El contador de datos almacenados se guardara en buffer2
         
         Guardo los primeros 10 registros para usos en configuraciones en el futuro
*/


#define MMAP_OFFSET_CONFIG                                  (uint16_t) 10

// De 0 A 47   // Registros especiales, No Data

//Intervalo en minutos para mediciones
#define MMAP_INTERVAL                                       (MMAP_OFFSET_CONFIG + 7)
// Numero de datos almacenados antes de enviar a servidor
#define MMAP_MAX_AMOUNT_DATA                                (MMAP_OFFSET_CONFIG + 8)
// Flag de estado de conexion
#define MMAP_F_CONNECTION                                   (MMAP_OFFSET_CONFIG + 3)
// Flag de estado de bateria: 3 Normal, 2 medio, 1 baja
#define MMAP_F_BATTERY_STATE                                (MMAP_OFFSET_CONFIG + 4)
#define MMAP_FSM_STATE                                      (MMAP_OFFSET_CONFIG + 5)

// Almacena los 6 bytes del ofsset  del acelerometro. Eje X, eje y, eje z 
#define MMAP_OFFSET_ACELEROMETER                              (MMAP_OFFSET_CONFIG + 10)
#define MMAP_OFFSET_ACELEROMETER_X                            (MMAP_OFFSET_ACELEROMETER + 1)
#define MMAP_OFFSET_ACELEROMETER_Y                            (MMAP_OFFSET_ACELEROMETER + 2)
#define MMAP_OFFSET_ACELEROMETER_Z                            (MMAP_OFFSET_ACELEROMETER + 3)

#define MMAP_MAX_COUNTER_INTERVAL                             (MMAP_OFFSET_ACELEROMETER_Z + 1)                                                            
#define MMAP_INTERVAL_TIME                                   (MMAP_MAX_COUNTER_INTERVAL + 2)  

// Formato para almacenar en paginar
#define FORMAT_MSG                                          ("\nx:%.2f, y:%.2f , z:%.2f , t:%.2f\n,\nGPS:%s\n")
// De 48 a 2047 // Datos almacenado
#define MMAP_DATA_OFSSET                                    ((uint16_t)48)
#define MMAP_DATA_ADDRESS(pos)                              (pos + MMAP_DATA_OFSSET)


#define MMAP_DEFAULT_INTERVAL                               (1)
#define MMAP_DEFAULT_MAX_AMOUNT_DATA                        (5)
#define MMAP_DEFAULT_BATTERY                                NORMAL_LEVEL
#define MMAP_DEFAULT_OFFSET_ACCELEROMETER                   ((int16_t)0)
#define MMAP_DEFAULT_COUNTER                                (0)


#ifdef __cplusplus
}
#endif

#endif /* __MAP_MEMORY_H */