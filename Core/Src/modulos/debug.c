
#include "uart.h"
#include "debug.h"



extern UART_HandleTypeDef   huart2;
#define UART                huart2
#define PUART               &huart2
#define DEBUG_TIMEOUT       (500)
#define DEBUG_ON            1





/**
 * @details Configura el periferico Uart
 * 
 * **/
inline void uart_init(void){
    UART.Instance = USART2;
    UART.Init.BaudRate = 19200;
    UART.Init.WordLength = UART_WORDLENGTH_8B;
    UART.Init.StopBits = UART_STOPBITS_1;
    UART.Init.Parity = UART_PARITY_NONE;
    UART.Init.Mode = UART_MODE_TX_RX;
    UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&UART) != HAL_OK)
  {
    Error_Handler();
  }
    MX_USART2_UART_Init();
}


/**
 * @details Desconfigura el periferico Uart
 * 
 * **/
void uart_deinit(){
      HAL_UART_DeInit(PUART);
}


/**
 * @details Imprimo una cadena string por puerto uart
 * @param string Cadena string con caracter de finalizacion ('0')
 * **/
inline void uart_write_string(char* string){
    HAL_UART_Transmit(PUART,string,strlen(string),DEBUG_TIMEOUT);
}


/**
 * @details Imprimo un array de bytes por puerto uart
 * @param array buffer con bytes
 * @param len Tamanio del buffer
 * **/
inline void uart_write_raw(uint8_t* array,uint32_t len){
    HAL_UART_Transmit(PUART,array,len,DEBUG_TIMEOUT);
}








inline void debug_init(){
    #if (DEBUG_ON == 1)
    uart_init();
    #endif   
}


inline void debug_deinit(){
    #if (DEBUG_ON == 1)
    uart_deinit();
    #endif
    
}


inline void debug_print(uint8_t* buffer){
    #if (DEBUG_ON == 1)
    uart_write_string(buffer);
    #endif
    
}


inline void debug_print_raw(uint8_t* buffer,size_t len){
    #if (DEBUG_ON == 1)
    uart_write_raw(buffer, len);
    #endif
}