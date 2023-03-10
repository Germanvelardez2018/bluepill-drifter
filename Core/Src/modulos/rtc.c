#include "rtc.h"

RTC_HandleTypeDef hrtc;





void rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds){  
    
    RTC_TimeTypeDef sTime = {0};
    sTime.Hours   = hours;
    sTime.Minutes = minutes;
    sTime.Seconds = seconds;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}


void rtc_get_time(uint8_t* hours, uint8_t* minutes, uint8_t* seconds){  
    
    RTC_TimeTypeDef sTime = {0};
    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK){
        (*hours) =sTime.Hours ;
        (* minutes) =sTime.Minutes ;
        (*seconds) = sTime.Seconds ;
    } 
}


void rtc_set_alarm(uint8_t hours, uint8_t minutes, uint8_t seconds){  
    
    RTC_AlarmTypeDef sAlarm ={0};
    sAlarm.AlarmTime.Hours = hours;
    sAlarm.AlarmTime.Minutes = minutes;
    sAlarm.AlarmTime.Seconds = seconds;
    sAlarm.Alarm = 1;
    HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
    
}






/**
  * @brief This function handles RTC alarm interrupt through EXTI line 17.
  */
void RTC_Alarm_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(&hrtc);  
}




/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
   

  /* USER CODE END RTC_MspInit 1 */
  }

}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
 
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt DeInit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  
  }

}




 void MX_RTC_Init(void)
{

  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

}


void rtc_init(void){
  MX_RTC_Init();
  rtc_set_time(0,0,0);

}

