
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT45DB_H
#define __AT45DB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "core.h"
#include "at45db.h"



// READ FUNCTIONS
#define      CMD_READBUFF1                    (0xD4)
#define      CMD_READBUFF2                    (0xD6) // D3 low frecuenccia
#define      CMD_READPAGE                     (0x52)      
#define      CMD_READPAGEHF                   (0x0B)  // Read page in high freq.
#define      CMD_READPAGELF                   (0x03)  // Read pafe in low freq.

//WRITE FUNCTIONS
#define    CMD_WRITEPAGE_B1                   (0x82)
#define    CMD_WRITEPAGE_B2                   (0x85)
#define    CMD_WRITEBUFF1                     (0x84) //
#define    CMD_WRITEBUFF2                     (0x87) //
#define    CMD_PAGETOBUFF1                    (0x53)
#define    CMD_PAGETOBUFF2                    (0x55)

#define    CMD_PAGETHROUGHBUFF1               (0x83)  // Write tn page through buffer 1 with automatic erase
#define    CMD_PAGETHROUGHBUFF2               (0X86)  // Write to page through buffer 2 with automatic erase

#define    CMD_PAGEBUFF1                      (0X88)
#define    CMD_PAGEBUFF1                      (0X89)

#define    CMD_PAGEBUFF1POS                   (0x02) // ! Permite seleecionar la posicion inicial

// COMPARE FUNCTIONS
#define    CMD_CMPBUFF1                      (0x60)
#define    CMD_CMPBUFF2                      (0x61)

// STATE
#define    CMD_GETSTATUS                    (0xD7)   //! status register 2 bytes
#define    CMD_RESUMEN                      (0xAB)
#define    CMD_LOWPOWER                     (0xB9)  
#define    CMD_READID                       (0x9F)
// ID_DEVICE

#define DEVICE_ID_0                          0x1F    //id byte  1
#define DEVICE_ID_1                          0x24    //id byte  2
#define DEVICE_ID_2                          0x00    //id byte  3
#define EDI                                  0x01    // id byte 4



// CONFIG SIZE FROM 265 TO 255 BYTE PER PAGE
#define CMD_RESIZE                          (0x3D, 0x2A, 0x80, 0xA6)
#define CMD_READSTATUS                      (0x57)

// ERASE MEMORY

#define CMD_ERASEPAGE                       (0X81)
#define CMD_ERASEBLOCK                      (0x50)
#define CMD_ERASESECTOR                     (0x7C)
#define SEC_ERASECHIP                       (0x9A, 0x80, 0x94, 0xC7) //! dont use this array to init a buffer. It won't work


// MASK
#define AT45DB_STATUS_COMP                  (1 << 6) /* COMP */
#define AT45DB_STATUS_READY                 (1 << 7) /* RDY/BUSY */
#define AT45DB_STATUS_PGSIZE                (1 << 0) /* PAGE SIZE */
#define AT45DB_STATUS_PROTECT               (1 << 1) /* PROTECT */
#define AT45DB_STATUS_COMP                  (1 << 6) /* COMP */
#define AT45DB_STATUS_READY                 (1 << 7) /* RDY/BUSY */


#define dummyByte                           (0xFF)
#define AT45DB_TIMEOUT                       (50)









/**
 *  Inicio la memoria. Llama a inicializar el hardware SPI
*/
void at45db_init();


/**
 * Deshabilito la memoria
 * 
*/
void at45db_deinit();






/***
 *  Escribo en el buffer 1 de la memoria. 
 *  Se utiliza en las variables que se leen y escriben con mucha frecuencia
 *  Ejemplo contador de muestras
*/
void at45db_write_buffer1(uint8_t* data,uint8_t len, uint8_t pos);


/***
 *  Leo el buffer 1 de la memoria
 * 
*/
void at45db_read_buffer1(uint8_t* data,uint8_t len, uint8_t pos);

/***
 * Escribo buffer de memoria 2
*/
void at45db_write_buffer2(uint8_t* data,uint8_t len, uint8_t pos);

/***
 * Leo buffer de memoria 2
*/
void at45db_read_buffer2(uint8_t* data,uint8_t len, uint8_t pos);

/***
 * 
 * Escribo datos en una pagina de la memoria. Tiene 255 posiciones.
* @param data: Array de datos a escribir
* @param len : tamanio del array de datos
* @param pag: Numero de pagina. (Se aplica un OFFSET predefinido)
* @param pos: Posicion desde donde empezar a escribir
*/
void at45db_write_page(uint8_t* data, uint8_t len, uint32_t pag,uint8_t pos);


/***
 * 
 * Lep datos en una pagina de la memoria. Tiene 255 posiciones.
* @param data: Array de datos a escribir
* @param len : tamanio del array de datos
* @param pag: Numero de pagina. (Se aplica un OFFSET predefinido)
* @param pos: Posicion desde donde empezar a escribir
*/
void at45db_read_page(uint8_t* data, uint8_t len, uint32_t pag,uint8_t pos);


/**
 *  Despierta la memoria.
*/
void at45db_resumen();


/**
 * Manda la memoria a sleep.
*/
void at45db_sleep();


/**
 * Borrado completo de la memoria.
*/
void at45db_full_erase_memory();


void at45db_clear_page( uint32_t pag,uint8_t pos);

// Funcion para borrar paginas( No se borran realmente, se marcan como disponibles)
 uint16_t get_counter();
#ifdef __cplusplus
}
#endif

#endif /* __AT45DB_H */
