/******************************************************************************
 * Copyright (C) 2021, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************/

/******************************************************************************
 * @file   main.c
 *
 * @brief  Source file for RTC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "rtc.h"
#include "lpm.h"
#include "gpio.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')                                         
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
__IO uint8_t toggleCnt;

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 void App_GpioCfg(void);
 void App_RtcCfg(void);
 
/**
******************************************************************************
    ** \brief  主函数
    ** 
  ** @param  无
    ** \retval 无
    **
******************************************************************************/ 
int32_t main(void)
{
    //配置GPIO
    App_GpioCfg();       

    //配置RTC
    App_RtcCfg();    
    
    while (1)
    {
        if(toggleCnt > 0)          // toggleCnt 在中断中设定值
        {
            toggleCnt--;
            
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);     //输出高，点亮LED            
            delay1ms(500);
            
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);    //输出低，熄灭LED            
            delay1ms(500);
        }
    }        
}
 
 /**
******************************************************************************
    ** \brief  RTC中断入口函数
    ** 
  ** @param  无
    ** \retval 无
    **
******************************************************************************/  
void Rtc_IRQHandler(void)
{
    if (Rtc_GetAlmfItStatus() == TRUE) //闹铃中断
    {
        toggleCnt = 10;                //通知mian函数主循环中闪烁LED 10次
        Rtc_ClearAlmfItStatus();       //清中断标志位
    }
}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
void App_GpioCfg(void)
{
    stc_gpio_cfg_t         GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);   //GPIO外设时钟打开
    
    //LED指示灯配置
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);     //输出低，默认熄灭LED        
}

/**
******************************************************************************
  ** \brief  配置RTC
  ** 
  ** @param  无
  ** \retval 无
  **
******************************************************************************/ 
void App_RtcCfg(void)
{
    stc_rtc_initstruct_t RtcInitStruct;
    stc_rtc_alarmtime_t RtcAlmStruct;
    
    DDL_ZERO_STRUCT(RtcInitStruct);                      //变量初始值置零
    DDL_ZERO_STRUCT(RtcAlmStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开    
    
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);        //使能外部XTL时钟作为RTC时钟
    
    RtcInitStruct.rtcAmpm = RtcPm;                       //24小时制
    RtcInitStruct.rtcClksrc = RtcClkXtl;                 //外部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrds;         //周期中断类型PRDS
    RtcInitStruct.rtcPrdsel.rtcPrds = RtcNone;           //不产生周期中断
    RtcInitStruct.rtcTime.u8Second = 0x55;               //配置RTC时间2019年4月17日10:01:55
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // 使能时钟误差补偿
    RtcInitStruct.rtcCompValue = 0;                      //补偿值  根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    
    RtcAlmStruct.RtcAlarmSec = 0x05;
    RtcAlmStruct.RtcAlarmMinute = 0x02;
    RtcAlmStruct.RtcAlarmHour = 0x10;
    RtcAlmStruct.RtcAlarmWeek = 0x7f;                    //从周一到周日，每天10:02:05启动一次闹铃    
    Rtc_SetAlarmTime(&RtcAlmStruct);                     //配置闹铃时间    
    Rtc_AlmIeCmd(TRUE);                                  //使能闹钟中断
    
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //使能RTC中断向量
    Rtc_Cmd(TRUE);                                       //使能RTC开始计数
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


