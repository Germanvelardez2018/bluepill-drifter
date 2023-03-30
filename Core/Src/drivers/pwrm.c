#include "pwrm.h"
#include "rtc.h"
#include "timer.h"
#include "clock_master.h"

#define SLEEP_INTERVAL()          {\
                                HAL_SuspendTick();\
                                HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);\
                                }\

#define RESUME_FROM_SLEEP()       {\
                                HAL_ResumeTick();\
                                }\
                                


#define WAIT_FOR_GPS_TIME_S     (15)
#define WAIT_FOR_SIMCOM         (20)



#define PWRM_CMAX_INTERVAL_15M         (INTERVAL_TIME_M/15)
#define PWRM_CMAX_INTERVAL_30M         (INTERVAL_TIME_M/30)
#define PWRM_CMAX_INTERVAL_60M         (INTERVAL_TIME_M/60)

#define DEFAULT_ITIME     (5)


// Se utiliza para reiniciar wdt
extern TIM_HandleTypeDef htim3;
PRIVATE uint8_t __SLEEP__ = 0; 
PRIVATE uint8_t __ITIME__ = DEFAULT_ITIME;



PRIVATE void __wait_for_sim(){
   uint8_t h,m,s ;
  // Obtengo time actual
  rtc_get_time(&h,&m,&s);
  // Aumento el intervalo
  s = s + WAIT_FOR_SIMCOM;
  rtc_set_alarm(h,m,s);
  SLEEP_INTERVAL() ;
}


PRIVATE void __wait_for_gps(){
   uint8_t h,m,s ;
  // Obtengo time actual
  rtc_get_time(&h,&m,&s);
  // Aumento el intervalo
  s = s+ WAIT_FOR_GPS_TIME_S;
  rtc_set_alarm(h,m,s);
  SLEEP_INTERVAL() ;
}






PRIVATE void __update_interval(){
  uint8_t h,m,s;
  rtc_get_time(&h,&m,&s);    // Obtengo time actual
  m = m + __ITIME__;        // Aumento el intervalo
  rtc_set_alarm(h,m,s);
}


// Interrupciones que despiertan el micro. Disponibles : Timer y RTC 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
    RESUME_FROM_SLEEP();
    __SLEEP__ = 0;
}


void pwrm_init(){
  rtc_init();
  mem_s_get_itime(&__ITIME__);
  if(__ITIME__ >60){
    __ITIME__=DEFAULT_ITIME;
    mem_s_set_itime(&__ITIME__);
  }
}


void sleep_interval(){
  __update_interval();
  __SLEEP__ = 1;
  SLEEP_INTERVAL(); 
}


void wait_for_gps(){
  __wait_for_gps();
  __SLEEP__ = 1;
}


void wait_for_sim(){
  __wait_for_sim();
  __SLEEP__ = 1; 
}


uint8_t pwrm_get_itime(){
  return __ITIME__;
}


void prwm_set_itime(uint8_t itime){
  __ITIME__ = itime;
  mem_s_set_itime(&itime);
}