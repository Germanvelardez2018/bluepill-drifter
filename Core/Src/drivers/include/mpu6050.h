/**
 * @file mpu6050.h
 * @author German Velardez (you@domain.com)
 * @brief  Archivo de configuracion del modulo MPU6050
 * @version 0.1
 * @date 2022-09-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */





/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "core.h"
/* Definiciones --------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/



/**
 * @brief Configuramos el periferico y el modulo a utilizar
 * 
 * @return ** status_t 
 */
void mpu6050_init();



/**
 * 
 * 
*/
void mpu6050_deinit();




/**
 * @brief Calibracion del sensor. Los offset se guardan en memoria flash
 * 
*/
void mpu6050_calibrate_and_save_offset();



/***
 * Obtengo la medicion en formato predefinido string
 * @param buffer: Donde se almacena datos
*/
void mpu6050_get_measure(uint8_t* buffer);


/**
 * @brief Salimos del modo de bajo consumo
 * 
 * @return ** status_t 
 */
void mpu6050_resume();


/**
 * @brief Entramos en el modo de bajo consumo
 * 
 * @return ** status_t 
 */
void mpu6050_sleep();





#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */