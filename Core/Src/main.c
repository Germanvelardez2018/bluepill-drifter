
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "debug.h"
#include "gpio.h"
#include "clock_master.h"
#include "time.h"
#include "sim.h"
#include "mpu6050.h"
#include "memory.h"
#include "fsm.h"
#include "rtc.h"
#include "pwrm.h"
#include "adc.h"
#include "drifter.h"




char* clist[9]={
    "CMD_INTERVAL_5M  ",
    "CMD_INTERVAL_15M ",
    "CMD_INTERVAL_30M ",
    "CMD_INTERVAL_60M ", 
    "CMD_SAVE_50D    ",
    "CMD_SAVE_150D    ",
    "CMD_SAVE_250D   ",
    "CMD_FORZAR_UPLOAD",
    "CMD_TEST_CMD     "
};





#define DRIFTER_VERSION                 "VERSION 1.0.5"
#define DRIFTER_STATUS_FORMAT           ("C:%d/%d,I:%d/%d,VB:%.2fV ,LCMD:%d")







PRIVATE  uint16_t ncounter= 0;




typedef enum{
    CMD_INTERVAL_5M   = 1,
    CMD_INTERVAL_15M  = 2,
    CMD_INTERVAL_30M  = 3,
    CMD_INTERVAL_60M  = 4, 
    CMD_SAVE_50D     = 5,
    CMD_SAVE_150D     = 6,
    CMD_SAVE_250D    = 7,
    CMD_FORZAR_UPLOAD = 8,
    CMD_TEST_CMD      = 9

} command_t;
















extern UART_HandleTypeDef huart1;




#define SENSOR_FORMAT ">%s,%s"
#define INIT_MSG "Drifter init \r\n"
#define CHECK_MSG_LEN (strlen(CHECK_MSG))
#define CHECK_TOPIC "S" // topic devices
#define DATA_TOPIC  "DATA"
#define CMD_TOPIC   "CMD"
#define RETURN_CMD  "RCMD"
#define BUFFER_SIZE 150

#define MQTT_SEND_CHECK() sim7000g_mqtt_publish(CHECK_TOPIC, get_state_device(), strlen(get_state_device()))

// #define MQTT_SEND_CHECK()               sim7000g_mqtt_publish("STATE", get_state_device(), strlen(get_state_device()))
#define MQTT_SEND_DATA(msg) sim7000g_mqtt_publish(DATA_TOPIC, msg, strlen(msg))

PRIVATE uint8_t buffer_upload[200];

#define ID_FORMAT ("C:%d/%d,I:%d,VB:%.2fV ,LCMD:%d")
#define STATE_DEVICE_LEN (100)
PRIVATE uint8_t state_device[STATE_DEVICE_LEN];
PRIVATE uint8_t counter = 0; //  Contador de muestras almacenadas
PRIVATE uint8_t cmax = 0;    // Maximo valor del contador de muestras almacenadas
PRIVATE uint8_t counter_interval = 0;
PRIVATE uint8_t cmax_interval = 0; // Contador maximo de intervalos
PRIVATE uint8_t last_comand = 0;
PRIVATE uint32_t bat = 0;

PRIVATE uint8_t *get_state_device(){
#define VBAT (13.93) // Se obtiene con medicion real
  bat = get_adc();

  float vbat = (bat * VBAT) / 4096;
  mem_s_get_max_amount_data(&cmax);
  uint8_t itime = pwrm_get_itime(); // tiempo del intervalo

  sprintf(state_device, ID_FORMAT,
          ncounter,
          cmax,
          itime,vbat, last_comand);
  return state_device;
}

#define delay_time (250)
#define delay_tiny (400)
#define min_delay (100)








PRIVATE void check_command()
{
  uint8_t p_cmd_buffer[25] = {0};
  uint8_t timeout = 0;
  uint8_t opt = 0;
  uint8_t out = 0;
  //Siempre leer bateria antes de encender el sim, voltaje mas estable
  delay(350);
  sim_gps_on();       // enciendo gps
  delay(350);
  sim_4g_connect();   // Conectarse a internet
  delay(min_delay);
  sim_mqtt_connect(); // Conectarse a servidor
  delay(200); 
  sim_sleep();
  delay(200);
  gpio_irq_on();
  delay(800); 
  sim7000g_mqtt_publish(CHECK_TOPIC, "OK", strlen("OK"));
  delay(800); 
  sim7000g_set_irt();
  delay(800); 
  debug_print(" sub a topic CMD \r\n");
  sim7000g_mqtt_subscription();
  while (opt == 0 && out == 0){
    get_copy_cmd_buffer(p_cmd_buffer);
    opt = sim7000g_get_parse(p_cmd_buffer);
    if (timeout == 10)break;
    timeout++;
    delay(250);
  }

  if (opt > 0)
  {
    uint8_t min = 0;
    uint8_t cmd_valid = 1;
     uint8_t itime = 1;
    if (last_comand != opt)
    {
      switch (opt)
      {

      case CMD_INTERVAL_5M:
        itime = 5;
        prwm_set_itime(itime);
        break;

      case CMD_INTERVAL_15M:
        itime = 15;
        prwm_set_itime(itime);
        break;

      case CMD_INTERVAL_30M:
        itime = 30;
        prwm_set_itime(itime);
        break;

      case CMD_INTERVAL_60M:
        itime = 60;
        prwm_set_itime(itime);
        break;

      case CMD_SAVE_50D:
        cmax = 50; // max data 50
        mem_s_set_max_amount_data(&cmax);
        break;

      case CMD_SAVE_150D:
        cmax=150;
        mem_s_set_max_amount_data(&cmax);
        break;

      case CMD_SAVE_250D:
        cmax=250;
        mem_s_set_max_amount_data(&cmax);
        break;

      case CMD_FORZAR_UPLOAD:
        fsm_set_state(FSM_UPLOAD); // Forzar extraccion
        break;

      case CMD_TEST_CMD:
        itime = 1;
        prwm_set_itime(itime);
       
        break;

      default:
        cmd_valid = 0;
        break;
      }

      if (cmd_valid){
        debug_print(clist[opt - 1]);
        sim7000g_mqtt_publish(RETURN_CMD, clist[opt-1], strlen(clist[opt-1]));
        last_comand = opt;
        delay(450);
      }
    }
  }
  // Unsub mqtt topic
  sim7000g_mqtt_unsubscription();
  delay(400);
  gpio_irq_off();
  debug_print("finalizo la sub a topic CMD \r\n");
}

PRIVATE void upload_routine()
{
  if (ncounter == 0){
    debug_print("Sin datos para extraer\r\n");
    MQTT_SEND_DATA("Sin datos para extraer");
    delay(500);
  }
  else{
    sprintf(buffer_upload, "EExtraer :%d datos\n", ncounter);
    ncounter --; // 1 dato, posicion 0
    debug_print(buffer_upload);
    uint8_t b512[512] = {0};

    uint16_t lastcounter = ncounter;

    do{
      // si devuelve 0 terminamos todo
      debug_print("generando buffer b512");
      ncounter = sim_buffer_512b(b512, 512,ncounter);
      debug_print(b512);
      MQTT_SEND_DATA(b512);
      delay(500); 
    }
    while (ncounter != 0);
    clear_memory(lastcounter);

  }
}





PRIVATE void save_data_routine()
{
  mpu6050_init(); 
//  mem_s_get_counter(&counter);
  uint8_t buffer[175] = {0};
  uint8_t sensor[50]={0};
  uint8_t gps[120]={0};
  mpu6050_get_measure(sensor);
  wait_for_gps();
  sim_gps_get_info(gps, 120);
  sprintf(buffer, SENSOR_FORMAT, gps, sensor);
  debug_print(buffer);
  mem_write_data(buffer, ncounter);
  sim_gps_off();
  mpu6050_deinit();
  ncounter = ncounter + 1;


  

}






PRIVATE void routine(){


  sim_init();         // Encender el modulo
  check_command();     //Verifico si tengo un comando nuevo
  MQTT_SEND_CHECK();
  delay(500);
  if(fsm_get_state() != FSM_UPLOAD){
    save_data_routine();
    if (counter >= cmax)  fsm_set_state(FSM_UPLOAD);
    else     sim_deinit();
  }  
  
  
 
  
}



PRIVATE void app_init(){
  HAL_Init();
  clock_master_set(CLOCK_2MHZ);
  GPIO_Init();
  mem_s_init();
  pwrm_init();
  debug_init();
  debug_print(INIT_MSG);
  fsm_init();

  ncounter = get_counter_memory();

  debug_print(get_state_device()); 
  //limpio 30 memoria y empiezo
  char b[20]={0};
  sprintf(b,"get counter:%d\r\n",ncounter);
  debug_print(b);
}


int main(void){
    app_init();
    uint8_t state = fsm_get_state();
    fsm_set_state(FSM_SAVE_DATA);



  while (1){
    switch (state){

          case FSM_SAVE_DATA:
            debug_print("\r\nFSM:SAVE DATA \r\n ");
            routine();
            
          break;

          case FSM_UPLOAD:
            debug_print("\r\nFSM: UPLOAD \r\n");
            upload_routine();
            sim_deinit();
            ncounter = 0;
            //counter = 0;
            //mem_s_set_counter(&counter);
            fsm_set_state(FSM_SAVE_DATA);
          break;


          default:
            debug_print("\r\nFSM: UNDEFINED \r\n");
          break;
    }

    state = fsm_get_state();
    if (state != FSM_UPLOAD){

      // Apago todo y me voy a dormir

      mem_s_deinit();
      sleep_interval();   // Solo en estado Check el micro entra en sleep
      mem_s_init();

    } 
  }
}








void Error_Handler(void)
{
  
  __disable_irq();
  while (1)
  {
  }
}
