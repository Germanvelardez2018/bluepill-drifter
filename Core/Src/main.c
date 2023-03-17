/* USER CODE BEGIN Header */
// Optimizacion de la aplicacion
/**
 * 1) Intervalos mas largo:
 *  checkout minimo 15 minutos (5 anterior version)
 * 2) Envios de texto mas compactos
 * 3) Deep sleep ajustado
 *
 *
 *
 *
 *
 */
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

extern UART_HandleTypeDef huart1;
#define MODO_1MCHECK  "\r\nCMD:Check 1m\r\n"
#define MODO_10MCHECK  "\r\nCMD:Check 10m\r\n"
#define MODO_30MCHECK  "\r\nCMD:Check 30m\r\n"


#define UPLOAD_CHECK "\r\nCMD:Forzar upload\r\n"
#define CMD_CHECK "\r\nCMD Nuevo intervalo:%d \r\n"
#define SENSOR_FORMAT ">%s,%s"
#define INIT_MSG "Drifter init \r\n"
#define CHECK_MSG_LEN (strlen(CHECK_MSG))
#define CHECK_TOPIC "S" // topic devices
#define DATA_TOPIC  "GPS"
#define CMD_TOPIC   "CMD"
#define RETURN_CMD  "RCMD"
#define BUFFER_SIZE 150

#define MQTT_SEND_CHECK() sim7000g_mqtt_publish(CHECK_TOPIC, get_state_device(), strlen(get_state_device()))

// #define MQTT_SEND_CHECK()               sim7000g_mqtt_publish("STATE", get_state_device(), strlen(get_state_device()))
#define MQTT_SEND_DATA(msg) sim7000g_mqtt_publish(DATA_TOPIC, msg, strlen(msg))

PRIVATE uint8_t buffer_upload[200];

#define ID_FORMAT ("C:%d/%d,I:%d/%d,VB:%.2fV ,LCMD:%d")
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
  mem_s_get_counter(&counter);
  mem_s_get_max_amount_data(&cmax);
  mem_s_get_cmax_interval(&cmax_interval);
  uint8_t itime = pwrm_get_itime(); // tiempo del intervalo

  sprintf(state_device, ID_FORMAT,
          counter,
          cmax,
          (cmax_interval * itime),itime,vbat, last_comand);
  return state_device;
}

#define delay_time (250)
#define delay_tiny (400)
#define min_delay (100)

PRIVATE void check_routine()
{
  uint8_t p_cmd_buffer[25] = {0};
  uint8_t timeout = 0;
  uint8_t opt = 0;
  uint8_t out = 0;
  //Siempre leer bateria antes de encender el sim, voltaje mas estable
  sim_init();         // Encender el modulo
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

      case 1:
        min = 20; // intervalo 20m
        sprintf(p_cmd_buffer, CMD_CHECK, min);
        cmax_interval = 2;
        mem_s_set_cmax_interval(&cmax_interval);
        break;

      case 2:
        min = 40; // intervalo 40m
        sprintf(p_cmd_buffer, CMD_CHECK, min);
        cmax_interval = 4;
        mem_s_set_cmax_interval(&cmax_interval);
        break;

      case 3:
        min = 60; // intervalo 60m
        sprintf(p_cmd_buffer, CMD_CHECK, min);
        cmax_interval = 6;
        mem_s_set_cmax_interval(&cmax_interval);
        break;

      case 4:
        sprintf(p_cmd_buffer, "CMD:data 20\r\n");
        cmax = 20; // max data 20
        mem_s_set_max_amount_data(&cmax);
        break;

      case 5:
        sprintf(p_cmd_buffer, "CMD:50 data\r\n");
        cmax = 50; // max data 50
        mem_s_set_max_amount_data(&cmax);
        break;

      case 6:
        sprintf(p_cmd_buffer, UPLOAD_CHECK);
        fsm_set_state(FSM_UPLOAD); // Forzar extraccion
        counter_interval = 0;      // evita que mande save data en vez de upload
        break;

      case 7:
        sprintf(p_cmd_buffer, MODO_1MCHECK);
        counter_interval = 0;      // evita que mande save data en vez de upload
        itime = 1;
        prwm_set_itime(itime);
        break;

      case 8:
        sprintf(p_cmd_buffer, MODO_10MCHECK);
        counter_interval = 0;      // evita que mande save data en vez de upload
        itime = 15;
        prwm_set_itime(itime);
        break;

      case 9:
        sprintf(p_cmd_buffer, MODO_30MCHECK);
        counter_interval = 0;      // evita que mande save data en vez de upload
        itime = 30;
        prwm_set_itime(itime);
        break;

      default:
        cmd_valid = 0;
        break;
      }

      if (cmd_valid){
        debug_print(p_cmd_buffer);
        sim7000g_mqtt_publish(RETURN_CMD, p_cmd_buffer, strlen(p_cmd_buffer));
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
  // EL modulo se mantiene prendido de la rutina anterior
  if (cmax < counter){
    counter = cmax;
    mem_s_set_counter(&counter);
  }

  if (counter == 0){
    debug_print("Sin datos para extraer\r\n");
    MQTT_SEND_DATA("Sin datos para extraer");
    delay(500);
  }
  else{
    sprintf(buffer_upload, "Extraer :%d datos\n", counter);
    counter --; // 1 dato, posicion 0
    debug_print(buffer_upload);
    uint8_t b512[512] = {0};

    do{
      // si devuelve 0 terminamos todo
      counter = sim_buffer_512b(b512, 512,counter);
     
      MQTT_SEND_DATA(b512);
      delay(500); 
    }
    while (counter != 0);
    debug_print("Finalizo deployd \r\n");
  }
}

PRIVATE void save_data_routine()
{
  mpu6050_init(); 

  mem_s_get_counter(&counter);
  uint8_t buffer[175] = {0};
  uint8_t sensor[50]={0};
  uint8_t gps[120]={0};
  mpu6050_get_measure(sensor);
  wait_for_gps();
  sim_gps_get_info(gps, 120);
  sprintf(buffer, SENSOR_FORMAT, gps, sensor);
  debug_print(buffer);
  mem_write_data(buffer, counter);
  sim_gps_off();
  mpu6050_deinit();
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
  debug_print(get_state_device());
  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
  HAL_Delay(200);
 
  
}


int main(void){



  app_init();
  uint8_t state = fsm_get_state();




  if (counter > cmax){
    sim_init();
    fsm_set_state(FSM_UPLOAD);
  }

  if (state == FSM_SAVE_DATA || state == FSM_UPLOAD){
    sim_init();
    delay(200);
    sim_gps_on();
    delay(200);
    sim_4g_connect();
    delay(200);
    sim_mqtt_connect();
    delay(200);
    sim_sleep();
    delay(200);
  }
  while (1){
    switch (fsm_get_state()){


          case FSM_CHECK_ONLY:
               debug_print("\r\nFSM:CHECK\r\n ");
               check_routine();
               MQTT_SEND_CHECK();
               delay(500);
               counter_interval = counter_interval + 1;
               if (counter_interval >= cmax_interval)fsm_set_state(FSM_SAVE_DATA);
               else{
                state = fsm_get_state();
                if (state == FSM_CHECK_ONLY)sim_deinit();
               }
          break;


          case FSM_SAVE_DATA:
            debug_print("\r\nFSM:SAVE DATA \r\n ");
            save_data_routine();
            counter = counter + 1;
            mem_s_set_counter(&counter);
            if (counter >= cmax)fsm_set_state(FSM_UPLOAD);
            else{
              fsm_set_state(FSM_CHECK_ONLY);
              sim_deinit();
            }
            counter_interval = 0;
          break;


          case FSM_UPLOAD:
            debug_print("\r\nFSM: UPLOAD \r\n");
            upload_routine();
            sim_deinit();
            counter = 0;
            mem_s_set_counter(&counter);
            fsm_set_state(FSM_CHECK_ONLY);
          break;


          default:
            debug_print("\r\nFSM: UNDEFINED \r\n");
          break;
    }
    uint8_t state = fsm_get_state();
    if (state == FSM_CHECK_ONLY){

      // Apago todo y me voy a dormir

      mem_s_deinit();

      sleep_interval();   // Solo en estado Check el micro entra en sleep

      mem_s_init();

    } 
  }
}










/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
