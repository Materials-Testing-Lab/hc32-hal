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
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
__IO uint8_t flag;
__IO uint8_t second, minute, hour, week, day, month, year;

/******************************************************************************
 * Local function prototypes ('static')
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
    Sysctrl_ClkSourceEnable(SysctrlClkXTL,TRUE);     //配置XTL作为RTC时钟
    
    App_GpioCfg();                                   //配置GPIO，按键按下，才可以进行运行
    
    App_RtcCfg();                                    //配置RTC        
        
    //while(Gpio_GetInputIO(GpioPortA,GpioPin7) == 1); //等待按键按下,按键一定要放在此处，确保进入低功耗模式之前，其它配置均执行完成
    
    Lpm_GotoDeepSleep(TRUE);                         //进入低功耗模式
    
    while (1)
    {
        ;
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
        if(TRUE == Gpio_GetInputIO(STK_LED_PORT, STK_LED_PIN))
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);    //输出低，熄灭LED
            Gpio_WriteOutputIO(GpioPortA, GpioPin5, FALSE);          //PA05高电平
        }            
        else
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);     //输出高，点亮LED
            Gpio_WriteOutputIO(GpioPortA, GpioPin5, TRUE);           //PA05低电平
        }        
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
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  //GPIO外设时钟打开
    
    //PD14 配置LED灯端口
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);   //配置LED灯默认熄灭状态
    //PA05 示波器观察引脚，配置输出信号和LED引脚保持一致
    Gpio_Init(GpioPortA, GpioPin5, &GpioInitStruct);
    //PA07 配置按键引脚   
    GpioInitStruct.enDir = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin7, &GpioInitStruct);   
    
    while(Gpio_GetInputIO(GpioPortA,GpioPin7) == 1); //等待按键按下
    
    //点亮LED ，PA05引脚输出高
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE); 
    Gpio_WriteOutputIO(GpioPortA, GpioPin5, TRUE);   
    
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
    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc,TRUE);   //RTC模块时钟打开
    RtcInitStruct.rtcAmpm = RtcPm;                          //24小时制
    RtcInitStruct.rtcClksrc = RtcClkXtl;                    //外部低速时钟
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;            //周期中断类型PRDX
    RtcInitStruct.rtcPrdsel.rtcPrdx = 9;                    //周期中断时间间隔 5秒
    RtcInitStruct.rtcTime.u8Second = 0x55;
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour   = 0x10;
    RtcInitStruct.rtcTime.u8Day    = 0x17;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month  = 0x04;
    RtcInitStruct.rtcTime.u8Year   = 0x19;
    RtcInitStruct.rtcCompen = RtcCompenEnable;
    RtcInitStruct.rtcCompValue = 0;                         //补偿值 根据实际情况进行补偿
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                                     //使能闹钟中断
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE);                  //使能RTC中断向量
    Rtc_Cmd(TRUE);                                          //使能RTC开始计数
    Rtc_StartWait();                                        //启动RTC计数，如果要立即切换到低功耗，需要执行此函数    
}



/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


