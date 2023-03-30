#include "sim.h"
#include "debug.h"
#include "memory.h"
#include "uart.h"

#define TEST_WITHOUT_INTERNET (0)

#define EXAMPLO_GPS_FORMAT ("1,1,20221216181549.000,-34.576180,-58.516807,16.000,0.00,0.0,1,,2.8,3.0,0.9,,10,4,,,40,,␍␊")

#define CMD_OPEN_APN_TUENTI "AT+CNACT=1,\"internet.movil\"\r\n"
#define CMD_OPEN_APN_PERSONAL "AT+CNACT=1,\"datos.personal.com\"\r\n"
#define CMD_GET_APN "AT+CNACT?\r\n"

#define CMD_TURN_OFF "AT+CPOWD=0\r\n" // con = 0el sim se apaga mas rapido
#define CMD_AT "AT\r\n"
#define CMD_OK "OK\r\n"
#define CMD_VERSION "ATI\r\n"
#define CMD_ECHO_ON "ATE1\r\n"
#define CMD_ECHO_OFF "ATE0\r\n"
#define CMD_GET_SIGNAL "AT+CSQ\r\n"
#define CMD_GPS_ON "AT+CGNSPWR=1\r\n"
#define CMD_GPS_OFF "AT+CGNSPWR=0\r\n"
#define CMD_GETGPSINFO "AT+CGNSINF\r\n"
#define CMD_GETOPERATOR "AT+COPS?\r\n"

#define CMD_MQTT_COMMIT "AT+SMCONN \r\n "
#define CMD_MQTT_SET_URL "AT+SMCONF=\"URL\",\"broker.hivemq.com\"\r\n"
#define CMD_ALL_CONFIG "AT+SMCONF=\"URL\",\"broker.hivemq.com\";SMCONF=\"QOS\",2 \r\n"
#define CMD_ALL_CONFI2 "AT+SMCONF=\"URL\",\"broker.hivemq.com\";SMCONF=\"QOS\",2;SMCONN \r\n"

#define CMD_MQTT_PUBLISH "AT+SMPUB=\"%s\",\"%d\",2,1 \r\n"
#define CMD_MQTT_SUBSCRIBE "AT+SMSUB=\"%s\",%d \r\n" // topic , QoS
#define CMD_MQTT_UMSUBSCRIBE "AT+SMUNSUB=\"%s\"\r\n" // topic , QoS

#define CMD_LOW_PWR_STATUS "AT+CPSMSTATUS=1\r\n"
#define CMD_LOW_PWR_ON "AT+CPSMS=1,\"01011111\",\"00000001\"\r\n"
#define CMD_LOW_PWR_OFF "AT+CPSMS=0 \r\n"
#define MQTT_SUB "AT+SMSUB=\"CMD\",2\r\n"
#define MQTT_UNSUB "AT+SMUNSUB=\"CMD\"\r\n"

extern UART_HandleTypeDef huart1;

#define IS_EQUAL (0)
#define SIM_UART &huart1
#define SIM_TIMEOUT 500
#define SIM_BUFFER_SIZE 150
#define SIM_DEFAULT_TIMEOUT 400
#define SIM_TIMEOUT_RX 800
#define SIM_TIMEOUT_TX 500

#define FLAG_PRINT_CHECK (0)
#define COMMAND_SIZE 25


PRIVATE uint8_t SIM_BUFFER[SIM_BUFFER_SIZE] = {0};
PRIVATE uint8_t buffer_cmd[COMMAND_SIZE] = {0};

#define SIM_BUFFER SIM_BUFFER
#define IRQ_ABORT                    \
    {                                \
        HAL_UART_Abort_IT(SIM_UART); \
    }
#define UART_WRITE(buffer, len, timeout)                   \
    {                                                      \
        IRQ_ABORT                                          \
        HAL_UART_Transmit(SIM_UART, buffer, len, timeout); \
    }

#define UART_READ(buffer, len, timeout)                   \
    {                                                     \
        IRQ_ABORT                                         \
        memset(buffer, 0, SIM_BUFFER_SIZE);               \
        HAL_UART_Receive(SIM_UART, buffer, len, timeout); \
    }

#define SEND_CMD(function, t)         \
    {                                 \
        uint8_t ret = 0;              \
        while ((ret = function) == 0) \
            delay(t);                 \
    }

//+CGNSINF: eliminado de la string
#define TITLE_OFFSET "+CGNSINF:"
#define OFFSET_GPS (strlen(TITLE_OFFSET) + 3) //(strlen("+CGNSINF:"))
#define OFFSET_GPS_END 21

#define GPS_PARSE(buffer, len)                   \
    {                                            \
        uint8_t l = strlen(SIM_BUFFER) + 1;      \
        strcpy(buffer, SIM_BUFFER + OFFSET_GPS); \
        buffer[l - OFFSET_GPS_END] = '\0';       \
    }                                            \
                                                          

#define RETEIN                                  (0)

// SI error retorna 0
static uint8_t get_parse(char *string){
    uint8_t ret = 0;
    ret = (uint8_t)atoi(&(string[17]));
    return ret;
}

uint8_t sim7000g_get_parse(char *string){
    uint8_t ret = get_parse(string);
    return ret;
}

void get_copy_cmd_buffer(uint8_t *buffer){
    strcpy(buffer, buffer_cmd);
    memset(buffer_cmd, 0, 20);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    buffer_cmd[18] = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle){
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    HAL_UART_Receive_IT(SIM_UART, buffer_cmd, 19); // 19
}

uint8_t *sim_get_cmd(){
    return buffer_cmd;
}

PRIVATE uint8_t check_response(char *response){
    uint32_t len_reponse = strlen(response);
    uint32_t len_buffer = strlen(SIM_BUFFER);
    uint32_t index = len_buffer - len_reponse;
    uint8_t ret = (strncmp(&(SIM_BUFFER[index]), response, len_reponse) == IS_EQUAL) ? 1 : 0;
    return ret;
}

uint8_t send_command(uint8_t *string_cmd, uint8_t *response_expected, size_t timeout, uint8_t print_flag){
    if (print_flag)
        debug_print("=>");
    if (print_flag)
        debug_print(string_cmd);
    UART_WRITE(string_cmd, strlen(string_cmd), timeout);
    UART_READ(SIM_BUFFER, SIM_BUFFER_SIZE, timeout);
    if (print_flag)
        debug_print(SIM_BUFFER);
    if (print_flag)
        debug_print("<=\n");
    uint8_t ret = check_response(response_expected);

    return ret;
}

PRIVATE void sim_turn_off(){
    send_command(CMD_TURN_OFF, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
}

void sim_init(){
    debug_print("init sim \r\n");
    HAL_GPIO_WritePin(SIM_BAT_ENABLE_GPIO_Port, SIM_BAT_ENABLE_Pin, 1);
    HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, 1);
    USART1_UART_Init();
    delay(30000);
    sim_echo_off();
}

void sim_deinit(){
    sim_turn_off();
    delay(800);
    HAL_GPIO_WritePin(SIM_BAT_ENABLE_GPIO_Port, SIM_BAT_ENABLE_Pin, 0);
    HAL_GPIO_WritePin(SIM_PWRKEY_GPIO_Port, SIM_PWRKEY_Pin, 0);
    USART1_Deinit();
}

inline void sim_version(){
    SEND_CMD(send_command(CMD_VERSION, CMD_OK, SIM_DEFAULT_TIMEOUT, 0), 250);
}

inline void sim_echo_off(){
    SEND_CMD(send_command(CMD_ECHO_OFF, CMD_OK, SIM_DEFAULT_TIMEOUT, 0), 250);
}

inline void sim_mqtt_connect(){
    send_command(CMD_MQTT_SET_URL, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(200);
#if (RETAIN == 1) // SE eliminan para ahorrar tiempo
    send_command("AT+SMCONF=\"QOS\",2\r\n", CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(250);
    send_command("AT+SMCONF=\"RETAIN\",1\r\n", CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(250);
#endif
    send_command(CMD_MQTT_COMMIT, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(600);
}

#define CMD_ADC_READ ("AT+CADC?\r\n")
inline void sim_adc_read(){
    send_command(CMD_ADC_READ, "OK\r\n", SIM_DEFAULT_TIMEOUT, 1);
}

inline void sim_4g_connect(){
    send_command(CMD_OPEN_APN_PERSONAL, "+APP PDP: ACTIVE\r\n", SIM_DEFAULT_TIMEOUT, 1);
}

inline void sim_at(){
    send_command("AT\r\n", CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
}

inline void sim_sleep(){
    send_command("AT+CEREG=4\r\n", CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(200);
    send_command(CMD_LOW_PWR_ON, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(200);
    send_command("AT+CEREG=0\r\n", CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
}

inline void sim_resumen()
{
    send_command(CMD_LOW_PWR_OFF, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
}




inline void sim_gps_on(){
    send_command(CMD_GPS_ON, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
    delay(200);    
}

inline void sim_gps_off(){
    send_command(CMD_GPS_OFF, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
}



inline void sim_gps_get_info(uint8_t *gps_buffer, size_t len){
    #define MIN_NMEA_LEN        (80)
    #define DELAY_GPS           (2000)
    #define GPS_MAX_ATTEMPT     (15)
    uint8_t counter = 0;
    uint8_t len_gps = 0;
    do{
        delay(DELAY_GPS);
        send_command(CMD_GETGPSINFO, CMD_OK, SIM_DEFAULT_TIMEOUT, 1);
        len_gps = strlen(SIM_BUFFER);
        counter++;
    }
    while (counter < GPS_MAX_ATTEMPT && len_gps < MIN_NMEA_LEN);
    GPS_PARSE(gps_buffer, len);
}



uint8_t *sim_get_gps_data(){
    return SIM_BUFFER;
}


void sim7000g_mqtt_publish(uint8_t *topic, uint8_t *payload, uint32_t len_payload){
    uint8_t buffer[50] = {0};
    sprintf(buffer, CMD_MQTT_PUBLISH, topic, len_payload);
    send_command(buffer, CMD_OK, 400, 1);
    send_command(payload, CMD_OK, 600, 1);
}


void sim7000g_mqtt_subscription(){
    send_command(MQTT_SUB, CMD_OK, 400, 1);
}


void sim7000g_mqtt_unsubscription(){
    send_command(MQTT_UNSUB, CMD_OK, 200, 1);
}


void sim7000g_set_irt(){
    send_command("AT+CFGRI=1\r\n", CMD_OK, 400, 1);
}


uint16_t sim_buffer_512b(uint8_t *buffer, uint32_t len, uint32_t c){

   // debug_print("dentro de la app para generar buffer");

    uint8_t b[540]={0};
    memset(buffer, 0, len);
    uint16_t ncounter = c;
    uint32_t pos=0;
    uint32_t l = 0;
    uint8_t data[200] = {0};
    for (uint8_t run = 1; run == 1;)
    {
        mem_read_data(data, ncounter);
        l = strlen(data);
        if ((pos + l) < (len - 1))
        {
            sprintf(&(buffer[pos]), "%s ", data);

            pos = (pos + l) + 1;
            if (ncounter >= 1)
                ncounter--; // if(ncounter != 0 && (ncounter - 1)>= 0)ncounter--;
            else run = 0;
        }
        else
        {
            run = 0;
        }
        buffer[len - 1] = 0;
    }

    return ncounter;
}
