#include "mpu6050.h"
#include "i2c.h"
#include "debug.h"




extern I2C_HandleTypeDef hi2c2;



#define PRINT_FLAG_MPU6050                          (0)

#define MPU6050_INSTANCE                            (&hi2c2)
#define MPU6050_ADDRESS                             (0xD0)
#define MPU6050_AVAILABLE                           (0x68)
#define ACCEL_XOUT_H                                (0x3B)
#define ACCEL_XOUT_L                                (0x3C)
#define ACCEL_YOUT_H                                (0x3D)
#define ACCEL_YOUT_L                                (0x3E)
#define ACCEL_ZOUT_H                                (0x3F)
#define ACCEL_ZOUT_L                                (0x40)
#define ACCEL_TOUT_H                                (0x41)
#define ACCEL_TOUT_L                                (0x42)
#define CONFIG                                      (0x1A)
#define GYRO_CONFIG                                 (0x1B)
#define ACCEL_CONFIG                                (0x1C)
#define USER_CTRL                                   (0x6A)
#define SMPLRT_DIV                                  (0x19)
#define INT_PIN_CFG                                 (0x37)
#define INT_ENABLE                                  (0x38)
#define INT_STATUS                                  (0x3A)
#define FIFO_EN                                     (0x23)
#define FIFO_COUNTER_H                              (0x72)
#define FIFO_COUNTER_L                              (0x73)
#define FIFO_BUFFER                                 (0x74)
#define PWR_MGMT_1                                  (0x6B)
#define PWR_MGMT_2                                  (0x6C)
#define WHO_I_AM                                    (0x75) // Default: 0x68
#define MPU6050_TIMEOUT                             (500)
// Esta definido en el archivo i2c.c (en el metodo de inicializacion)
#define I2C_CLOCK                                   (10000) 
#define SENSOR_FORMAT                       (":%.2f,%.2f,%.2f,%.2f")                                                //t,x,y,z



//Nota, al iniciar el dispositivo se calibra suponiendo posicion x=0G, y=0G ,z1G.

PRIVATE int16_t offset_x,offset_y,offset_z ;

PRIVATE float SCALA_DIV = 16384.0;




PRIVATE inline void i2c_write_mem( uint8_t address_reg,uint8_t* buffer,uint32_t len){
    HAL_I2C_Mem_Write(MPU6050_INSTANCE,MPU6050_ADDRESS,address_reg,1,buffer,len,MPU6050_TIMEOUT);
}


PRIVATE inline void i2c_read_mem( uint8_t address_reg,uint8_t* buffer,uint32_t len){
   HAL_I2C_Mem_Read(MPU6050_INSTANCE,MPU6050_ADDRESS,address_reg,1,buffer,len,MPU6050_TIMEOUT);
}




/**
 * @brief Comprueba si el dispositivo esta ready
 * 
 * @return ** PRIVATE 
 */
PRIVATE uint8_t mpu6050_rdy(){
    uint8_t ret = 0;
    uint8_t value = 0;
    i2c_read_mem(WHO_I_AM,&value,1);
    ret = (value == MPU6050_AVAILABLE) ?1:0;
    return ret;
}


PRIVATE void mpu6050_set_scala()
{
    uint8_t config_scala = 0;
    BIT_SET(config_scala,3);
    SCALA_DIV = 16384.0;
    
    i2c_write_mem( ACCEL_CONFIG,&config_scala,1) ;
}


PRIVATE void  mpu6050_get_acceleration( int16_t* x, int16_t* y, int16_t* z){
    // Leer los 6 registros juntos
    uint8_t address_mem = ACCEL_XOUT_H;  // el primer elemento
    #define MEM_SIZE_ACCEL  (6)
    uint8_t data[MEM_SIZE_ACCEL]={0};
    i2c_read_mem(address_mem,data,MEM_SIZE_ACCEL);
    (*x) = (int16_t)(data[0] << 8 | data [1]);
    (*x) = (*x) + offset_x;
    (*y) = (int16_t)(data[2] << 8 | data [3]);
    (*y) = (*y) + offset_y;
    (*z) = (int16_t)(data[4] << 8 | data [5]);   
    (*z) = (*z) + offset_z;
    //el string de datos
    // 6 bytes =  |  XH  | XL |  YH | YL |  ZH | ZL |  
}




PRIVATE void  mpu6050_get_temperature( int16_t* temp){
    uint8_t address_mem = ACCEL_TOUT_H;  // el primer elemento
    #define MEM_SIZE_T  (2)
    uint8_t data[MEM_SIZE_T]={0};
    i2c_read_mem(address_mem,data,MEM_SIZE_T);
    (*temp) = (int16_t)((int16_t)(data[0] << 8) | ((int16_t)data[1]));
}


/**
 * @brief Calibracion con offset en sram
 * 
 * @param x_e 
 * @param y_e 
 * @param z_e 
 * @return ** PRIVATE 
 */
PRIVATE void mpu6050_calibration(int16_t x_e, int16_t y_e, int16_t z_e){

    // x_e, y_e, z_e son los valores esperando. En nuestro caso 0,0,g    
    // para calibracion en superficie horizontal
    #define CALIBRATION_ITER     1000
    int16_t x,y,z ;
    int16_t delta_x = 0;
    int16_t delta_y = 0;
    int16_t delta_z = 0;
    for(int16_t i= 0; i< CALIBRATION_ITER ; i++){
    mpu6050_get_acceleration(&x,&y,&z);
    delta_x = x_e - x;
    delta_y = y_e - y;
    delta_z = z_e - z;
    //corrijo los offset
    if(delta_x !=0) offset_x = (delta_x >0)? (offset_x+1): (offset_x-1);
    if(delta_y !=0) offset_y = (delta_y >0)? (offset_y+1): (offset_y-1);
    if(delta_z !=0) offset_z = (delta_z >0)? (offset_z+1): (offset_z-1);
    //recargo los offset
   //__MPU6050_set_offset(offset_x, offset_y, offset_z);
    }    
}



PRIVATE void mpu6050_get_offset(){
  
   // mem_s_get_x_offset( &offset_x); // guardo en memoria flash
   // mem_s_get_y_offset( &offset_y); // guardo en memoria flash
   // mem_s_get_z_offset( &offset_z); // guardo en memoria flash

}




// Calibramos en posicion estandar
PRIVATE void mpu6050_calibration_routine(){
    mpu6050_calibration(0, 0, (SCALA_DIV / 2.0));   // ESPERAMOS X=0G , Y=0G y Z=1G
}




void  mpu6050_resume(){
    uint8_t value = 0;
    i2c_read_mem(PWR_MGMT_1,&value,1);
    BIT_CLEAR(value,3);
    BIT_CLEAR(value,5); // DESHABILITAMOS EL MODO CICLICO
    BIT_CLEAR(value,6);
    i2c_write_mem(PWR_MGMT_1,&value,1);
}


void  mpu6050_sleep(){
    uint8_t value = 0;
    i2c_read_mem(PWR_MGMT_1,&value,1);
    BIT_CLEAR(value,5); // DESHABILITAMOS EL MODO CICLICO
    BIT_SET(value,6);
    i2c_write_mem(PWR_MGMT_1,&value,1); 
}





void mpu6050_get_measure(uint8_t* buffer){
    mpu6050_resume();
    int16_t temp =0;
    mpu6050_get_temperature(  &temp);
    float ft = (float)  ((temp/340.0)+ 36.53);
    int16_t x,y,z ;
    mpu6050_get_acceleration(  &x,&y,&z);
    float fx = (float) (x/(SCALA_DIV/2.0)); //      
    float fy = (float) (y/(SCALA_DIV/2.0)); // 
    float fz = (float) (z/(SCALA_DIV/2.0)); // 
    sprintf(buffer,SENSOR_FORMAT,ft,fx,fy,fz);
    mpu6050_sleep();
}




PRIVATE void hardware_init(){
    I2C_Init();
}
PRIVATE void hardware_deinit(){
    I2C_Deinit();

}
/**
 * @brief Configuramos el periferico y el modulo a utilizar
 * 
 * @return ** status_t 
 */
void mpu6050_init(){

    // Inicio hardware I2C
    hardware_init();
     // Secuencia de arranque
    uint8_t ret = 0;
    uint8_t config = 0;
    uint8_t counter = 0;
    #define TIMEOUT_MPU6050     3
    i2c_write_mem(PWR_MGMT_1,&config,1);
    while(ret  == 0 && counter < TIMEOUT_MPU6050){
        debug_print("mpu6050 is not ready \r\n");
        delay(250);
        ret= mpu6050_rdy();
        counter ++;
    }
    debug_print("mpu6050 is ready \r\n");
    mpu6050_resume();
    mpu6050_set_scala();
    // Calibramos
    mpu6050_calibrate_and_save_offset();
    mpu6050_get_offset();
    mpu6050_sleep();

}




 void mpu6050_deinit(){
     harware_deinit();
}




/**
 * @brief Calibro el sistema (opcional) y ob
 * 
 * 
*/
void mpu6050_calibrate_and_save_offset(){
    // Esta funcion guarda offset en static x, offset_x offset_y, offset_z
    mpu6050_calibration_routine(); 
}

