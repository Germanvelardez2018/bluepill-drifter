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
#ifndef __MEMORY_H
#define __MEMORY_H

#ifdef __cplusplus
extern "C" {
#endif



#include "core.h"
/* Includes ------------------------------------------------------------------*/





void mem_s_init();



void mem_s_deinit();






void mem_s_get_itime(uint8_t* itime);
void mem_s_set_itime(uint8_t* itime);



void mem_s_get_cmax_interval(uint8_t* cmax_interval);
void mem_s_set_cmax_interval(uint8_t* cmax_interval);


void mem_s_get_fsm_state(uint8_t* fsm_state);
void mem_s_set_fsm_state(uint8_t* fsm_state);



void mem_s_get_counter(uint8_t* counter);
void mem_s_set_counter(uint8_t* counter);



// Interval
void mem_s_get_interval(uint8_t* interval);
void mem_s_set_interval(uint8_t* interval);


void mem_s_get_max_amount_data(uint8_t* max_amount_data);
void mem_s_set_max_amount_data(uint8_t* max_amount_data);







/**
 * @brief  Seteo el offset en eje x del sensor MPU6050
 * 
 * @param x 
 */
void mem_s_get_x_offset(int16_t* x);

/**
 * @brief Seteo el offset en eje Y del sensor MPU6050
 * 
 * @param y 
 */
void mem_s_get_y_offset(int16_t* y);


/**
 * @brief Seteo el offset en eje Z del sensor MPU6050
 * 
 * @param z 
 */
void mem_s_get_z_offset(int16_t* z);




void mem_s_set_x_offset(int16_t x);


void mem_s_set_y_offset(int16_t y);


void mem_s_set_z_offset(int16_t z);






void mem_s_get_max_accelerometer_offset(int16_t* x,int16_t* y, int16_t* z );
void mem_s_set_max_accelerometer_offset(int16_t* x, int16_t* y,int16_t* z);









void mem_write_data(uint8_t* buffer, uint32_t page);






void mem_read_data(uint8_t* buffer, uint32_t page);

/***
 * 
 * 
*/
uint8_t* mem_s_get_all_data(uint32_t counter);



void mem_test();

void mem_full_clear();










#ifdef __cplusplus
}
#endif

#endif /* __MEMORY_H */