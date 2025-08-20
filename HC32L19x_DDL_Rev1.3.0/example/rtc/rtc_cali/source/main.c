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
#include "ddl.h"
#include "rtc.h"
#include "sysctrl.h"
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

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_GpioCfg(void);
void App_RtcCfg(void);
void App_ClkCfg(void);

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int32_t main(void)
{
    // 时钟相关配置
    App_ClkCfg();  

    // 端口相关配置    
    App_GpioCfg(); 

    // RTC寄存器相关配置    
    App_RtcCfg();
    
    // RTC的1Hz输出
    Rtc_Hz1Cmd(RtcHz1selHighPricision, TRUE);
    
    //RTC计数器的使能
    Rtc_Cmd(TRUE);  
    
    while (1)
    {        
        ;
    }
}

//端口相关配置
void App_GpioCfg(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    
    DDL_ZERO_STRUCT(GpioInitStruct);                     //初始值清零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//GPIO外设时钟打开
    
    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(GpioPortB,GpioPin14,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortB,GpioPin14,GpioAf5);         //1hz输出口PB14
}

//RTC寄存器相关配置
void App_RtcCfg(void)
{
    stc_rtc_initstruct_t RtcInitStruct;
    
    DDL_ZERO_STRUCT(RtcInitStruct);                      //初始值清零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);//RTC模块时钟打开 
    
    RtcInitStruct.rtcAmpm = RtcPm;                       //24小时制
    RtcInitStruct.rtcClksrc = RtcClkRcl;                 //内部低速时钟
    RtcInitStruct.rtcTime.u8Second = 0x00;               //配置时钟
    RtcInitStruct.rtcTime.u8Minute = 0x45;
    RtcInitStruct.rtcTime.u8Hour   = 0x17;
    RtcInitStruct.rtcTime.u8Day    = 0x16;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x03;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;           // 使能时钟误差补偿
    RtcInitStruct.rtcCompValue = 0;                      //补偿值  根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
}

//时钟配置
void App_ClkCfg(void)
{
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);       //内部高速时钟频率TRIM值加载    
    Sysctrl_SetRTCAdjustClkFreq(SysctrlRTC24MHz);  //补偿高速时钟源   
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);  //使能RCL时钟
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


