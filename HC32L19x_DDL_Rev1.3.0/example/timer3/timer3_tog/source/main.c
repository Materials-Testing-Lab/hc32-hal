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
 * @brief  Source file for TIMER3 example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "timer3.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/*******************************************************************************
 * TIM3中断服务函数
 ******************************************************************************/
void Tim3_IRQHandler(void)
{
     //Timer3 模式0 计数溢出中断，可在Timer3 配置函数中使能中断
     if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
     {
         
        Tim3_ClearIntFlag(Tim3UevIrq);
     }
}

//Timer3 CHx 端口配置
void App_LEDPortCfg(void)
{
    stc_gpio_cfg_t         stcTIM3Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM3Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    stcTIM3Port.enDir  = GpioDirOut;
    //PA08设置为TIM3_CH0A
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);
    
    //PA07设置为TIM3_CH0B
    Gpio_Init(GpioPortA, GpioPin7, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf4);
}

//Timer3 配置
void App_Timer3Cfg(uint16_t u16Period)
{
    uint16_t                  u16ArrValue;
    uint16_t                  u16CntValue;
    stc_tim3_mode0_cfg_t   stcTim3BaseCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTim3BaseCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE); //Timer3外设时钟使能
    
    stcTim3BaseCfg.enWorkMode = Tim3WorkMode0;               //定时器模式
    stcTim3BaseCfg.enCT       = Tim3Timer;                   //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv16;               //PCLK/16
    stcTim3BaseCfg.enCntMode  = Tim316bitArrMode;            //自动重载16位计数器/定时器
    stcTim3BaseCfg.bEnTog     = TRUE;
    stcTim3BaseCfg.bEnGate    = FALSE;
    stcTim3BaseCfg.enGateP    = Tim3GatePositive;
    
    Tim3_Mode0_Init(&stcTim3BaseCfg);                        //TIM3 的模式0功能初始化
        
    u16ArrValue = 0x10000 - u16Period;
    Tim3_M0_ARRSet(u16ArrValue);                             //设置重载值
    
    u16CntValue = 0x10000 - u16Period;
    Tim3_M0_Cnt16Set(u16CntValue);                           //设置计数初值
    
    //Tim3_ClearIntFlag(Tim3UevIrq);                         //清中断标志
    //EnableNvic(TIM3_IRQn, 3, TRUE);                        //TIM3 开中断
    //Tim3_Mode0_EnableIrq();                                //使能TIM3中断(模式0时只有一个中断)    
    
    Tim3_M0_Enable_Output(TRUE);                             //TIM3 端口输出使能
}

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
    App_LEDPortCfg();     //LED端口配置

    App_Timer3Cfg(25000); //Timer3 配置; 16分频,周期25000-->25000*(1/4M) * 16 = 100000us = 100ms
    
    Tim3_M0_Run();        //TIM3 运行。
    
    while (1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


