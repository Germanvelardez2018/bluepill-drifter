/**
 * @file AT45DB041.c
 * @author German Velardez  (gvelardez@inti.gob.ar.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */



#include "at45db.h"
#include "gpio.h"
#include "spi.h"
#include "mapmem.h"
extern   SPI_HandleTypeDef hspi1;



// Configuracion de SPI
#define AT45DB_SPI                            &hspi1
#define CS_Pin                                SPI1_CS_Pin
#define CS_GPIO_Port                          SPI1_CS_Port          




PRIVATE inline void hardware_init(){
    SPI_Init();
}

PRIVATE inline void hardware_deinit(){
    SPI_deinit();
} 

PRIVATE  inline void  gpio_write(level_t value){
    HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,(GPIO_PinState)value);
}


PRIVATE  inline void  spi_write(uint8_t* buffer, size_t len ){
    HAL_SPI_Transmit(AT45DB_SPI,buffer,len,1000);

}

PRIVATE  inline void  spi_read(uint8_t* buffer, size_t len){
    HAL_SPI_Receive(AT45DB_SPI,buffer,len,1000);  
}


PRIVATE uint8_t  at45db_get_status(){
    uint8_t first_byte_status[2] = {0};
    uint8_t cmd = CMD_GETSTATUS;
    gpio_write(0);
    spi_write(&cmd,1); // Leo los 2 bytes de registro de status
    spi_read(&first_byte_status,2); // Leo los 2 bytes de registro de status
    gpio_write(1);
    return first_byte_status[0];
}


PRIVATE uint8_t at45db_is_ready(){
    uint8_t ret= 0; // Device bussy
    ret = at45db_get_status() ;
    return (ret & AT45DB_STATUS_READY);
}


PRIVATE void at45db_wait(uint16_t timeout){
    uint8_t ret = at45db_is_ready();
    while(!(ret = at45db_is_ready) || (timeout != 0) ){
        timeout --;
        delay(1);
    }
}



PRIVATE uint8_t  at45db_check_id(){
    uint8_t ret = 0;
    uint8_t data[4]={0};
    uint8_t cmd = CMD_READID;
    gpio_write(0);
    spi_write(&cmd,1);
    spi_read(&data,4);
    gpio_write(1);
    if(data[0] == DEVICE_ID_0   
    || data[1] == DEVICE_ID_1   
    || data[2] == DEVICE_ID_2        
    || data[3] == EDI) {
        ret = 1; // id checked. OK
    }
    return ret;
}




 PRIVATE uint8_t at45db_set_size_page( ){
    gpio_write(0);
    static uint8_t at45db_pgsize_cmd[] = {0x3D, 0x2A, 0x80, 0xA6};  // 256
    spi_write(at45db_set_size_page,4);
    gpio_write(1);
    at45db_wait(AT45DB_TIMEOUT);
}





void at45db_init(){
    hardware_init();
    at45db_set_size_page();
}


void at45db_deinit(){
    hardware_deinit();
}








void at45db_write_buffer1(uint8_t* data,uint8_t len, uint8_t pos){
    uint8_t cmd[4] ={0};
    cmd[0] = CMD_WRITEBUFF1;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] =  pos  ;
    gpio_write(0);
    spi_write(cmd,4);
    delay(1);
    spi_write(data,len);
    gpio_write(1);
    at45db_wait(AT45DB_TIMEOUT);
}


void at45db_read_buffer1(uint8_t* data,uint8_t len, uint8_t pos){
    uint8_t cmd[5] ={0};
    cmd[0] = CMD_READBUFF1;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = pos ;
    gpio_write(0);
    spi_write(cmd,5);
    delay(1);
    spi_read(data,len);
    gpio_write(1);
    at45db_wait(AT45DB_TIMEOUT);
}


void at45db_write_buffer2(uint8_t* data,uint8_t len, uint8_t pos){
         uint8_t cmd[4] ={0};
        cmd[0] = CMD_WRITEBUFF2;
        cmd[1] = 0;
        cmd[2] = 0;
        cmd[3] = pos ;
        gpio_write(0);
        spi_write(cmd,4);
        spi_write(data,len);
        gpio_write(1);
        at45db_wait(AT45DB_TIMEOUT);
}


void at45db_read_buffer2(uint8_t* data,uint8_t len, uint8_t pos){
        uint8_t cmd[5] ={0};
        cmd[0] = CMD_READBUFF2;
        cmd[1] = 0;
        cmd[2] = 0;
        cmd[3] =  pos  ;
        gpio_write(0);
        spi_write(cmd,5);
        spi_read(data,len);
        gpio_write(1);
        at45db_wait(AT45DB_TIMEOUT);
}



void at45db_write_page(uint8_t* data, uint8_t len, uint32_t pag,uint8_t pos){
        uint8_t _len = len ;  // 
        uint32_t address =  (pag << 8) | (uint32_t)pos ;   //          | xxxx xxxx |  xxxx xppp |pppp pppp |oooo oooo |
        uint8_t cmd[4] ={0};
        cmd[0] = CMD_WRITEPAGE_B1;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8) & 0xFF;
        cmd[3] =  address & 0xFF;
        gpio_write(0);
        spi_write(cmd,4);
        delay(1);
        spi_write(&_len,1);
        delay(1);
        spi_write(data,_len);
        delay(1);
        gpio_write(1);
        at45db_wait(AT45DB_TIMEOUT);       
}


void at45db_read_page(uint8_t* data, uint8_t len, uint32_t pag,uint8_t pos){
        uint32_t address =  (pag << 8) | (uint32_t)pos ;       //
        uint8_t cmd[5] ={0};
        uint8_t _len = 0;
        cmd[0] = CMD_READPAGEHF;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8) & 0xFF;
        cmd[3] =  address  & 0xFF;
        cmd[4] = dummyByte;
        gpio_write(0);
        spi_write(cmd,5);
        delay(1);
        spi_read(&_len,1);
        delay(1);
        spi_read(data,_len);
        delay(1);
        gpio_write(1);
        //wait
        at45db_wait(AT45DB_TIMEOUT);       
}



// "Borrar una pagina (la dejo disponible para escribir)"
void at45db_clear_page( uint32_t pag,uint8_t pos){
    // Basicamente escribo 0xFF en la primera posicion para denotar q esta disponible para escribir
        uint8_t _len = 0xFF ;  // 
        uint32_t address =  ((pag) << 8) | (uint32_t)pos ;   //          | xxxx xxxx |  xxxx xppp |pppp pppp |oooo oooo |
        uint8_t cmd[4] ={0};
        cmd[0] = CMD_WRITEPAGE_B1;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8) & 0xFF;
        cmd[3] =  address & 0xFF;
        gpio_write(0);
        spi_write(cmd,4);
        delay(1);
        spi_write(&_len,1);
        delay(1);
        gpio_write(1);
        at45db_wait(AT45DB_TIMEOUT);  

}

uint8_t at45db_is_clear(uint32_t pag,uint8_t pos){

        uint32_t address =  (pag << 8) | (uint32_t)pos ;       //
        uint8_t cmd[5] ={0};
        uint8_t _len = 0x00;
        cmd[0] = CMD_READPAGEHF;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8) & 0xFF;
        cmd[3] =  address  & 0xFF;
        cmd[4] = dummyByte;
        gpio_write(0);
        spi_write(cmd,5);
        delay(1);
        spi_read(&_len,1);
        gpio_write(1);
        //wait
        at45db_wait(AT45DB_TIMEOUT);  
        // si len es 0xFF, la memoria esta disponible
        uint8_t ret = (_len == 0xFF)?0:1;
        uint8_t b[100]={0};
        sprintf(b,"isclear:%d\r\n",_len);
        debug_print(b);
        return ret;

}

//Cuento cuantas posiciones de memoria tiene un valor diferente a 0x00
 uint16_t get_counter(){
    uint16_t counter = 0;
    #define COUNTERMAX   500
    at45db_resumen();
    uint8_t ret = 0;

    do{
        ret =  at45db_is_clear((uint32_t)(counter+MMAP_DATA_OFSSET),0);
        counter = counter +ret;
    }

    while(ret == 1 && (counter <= COUNTERMAX));

    at45db_sleep();
    return counter;
}





void at45db_resumen(){ 
        uint8_t cmd=CMD_RESUMEN;
        gpio_write(0);
        spi_write(&cmd,1);
        gpio_write(1);
}


void at45db_sleep(){
        uint8_t cmd=CMD_LOWPOWER;
        gpio_write(0);
        spi_write(&cmd,1);
        gpio_write(1);
}




void at45db_full_erase_memory(){
        uint8_t cmd[4]={0};
        cmd[0] = 0xC7;
        cmd[1] = 0x94;
        cmd[2] = 0x80;
        cmd[3] = 0x9a;
        gpio_write(0);
        spi_write(&cmd,4);
        gpio_write(1);
        at45db_wait(2500);
}