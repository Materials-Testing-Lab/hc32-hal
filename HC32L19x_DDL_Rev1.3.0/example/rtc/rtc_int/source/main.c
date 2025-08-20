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
__IO uint8_t flag;
__IO uint8_t second, minute, hour, week, day, month, year;

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
    stc_rtc_time_t readtime;
    
    //使能外部XTL时钟作为RTC时钟
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);    
    
    //配置GPIO
    App_GpioCfg();       

    //配置RTC
    App_RtcCfg();    
    
    while (1)
    {
        if(flag == 1)                     // flag在中断中置1
        {
            flag = 0;
            Rtc_ReadDateTime(&readtime);  // 获取RTC时钟值
            second = readtime.u8Second;   // 读取时钟值
            minute = readtime.u8Minute;
            hour   = readtime.u8Hour;
            day    = readtime.u8Day;
            week   = readtime.u8DayOfWeek;
            month  = readtime.u8Month;
            year   = readtime.u8Year;
            
            //LED 闪烁
            if(TRUE == Gpio_GetInputIO(STK_LED_PORT, STK_LED_PIN))       //如果当前LED灯是点亮状态
            {
                Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);    //输出低，熄灭LED
                Gpio_WriteOutputIO(GpioPortA, GpioPin5, FALSE);          //PA05高电平
            }            
            else                                                         //如果当前LED灯是熄灭状态
            {
                Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);     //输出高，点亮LED
                Gpio_WriteOutputIO(GpioPortA, GpioPin5, TRUE);           //PA05低电平
            }
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
    if(Rtc_GetPridItStatus() == TRUE)
    {
        flag = 1;
        Rtc_ClearPrdfItStatus();             //清除中断标志位
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
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);     //输出高，点亮LED  
    
    //配置PA05引脚和LED控制引脚一样输出，方便示波器观察
    Gpio_Init(GpioPortA, GpioPin5, &GpioInitStruct);      
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
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开
    RtcInitStruct.rtcAmpm = RtcPm;                       //24小时制
    RtcInitStruct.rtcClksrc = RtcClkXtl;                 //外部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;         //周期中断类型PRDX
    RtcInitStruct.rtcPrdsel.rtcPrdx = 1u;                //周期中断时间间隔 1秒
    RtcInitStruct.rtcTime.u8Second = 0x55;               //配置RTC时间
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // 使能时钟误差补偿
    RtcInitStruct.rtcCompValue = 0;                      //补偿值  根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                                  //使能闹钟中断
    
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);               //使能RTC中断向量
    Rtc_Cmd(TRUE);                                       //使能RTC开始计数
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


