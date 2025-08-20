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
#include "flash.h"
#include "gpio.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
volatile uint32_t u32PwcCapValue;
volatile uint16_t u16TIM3_CntValue;

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
    static uint16_t u16TIM3_OverFlowCnt;
    
    static uint16_t u16TIM3_CapValue;
    
    //Timer3 模式1 计数溢出中断
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
        u16TIM3_OverFlowCnt++;         //计数脉宽测量开始边沿到结束边沿过程中timer的溢出次数
        
        Tim3_ClearIntFlag(Tim3UevIrq); //清除中断标志
    }

    //Timer3 模式1 PWC脉宽测量中断
    if(TRUE == Tim3_GetIntFlag(Tim3CA0Irq))
    {
        u16TIM3_CntValue = Tim3_M1_Cnt16Get();         //读取当前计数值
        u16TIM3_CapValue = Tim3_M1_PWC_CapValueGet();  //读取脉宽测量值
        
        u32PwcCapValue = u16TIM3_OverFlowCnt*0x10000 + u16TIM3_CapValue;
        
        u16TIM3_OverFlowCnt = 0;
        
        Tim3_ClearIntFlag(Tim3CA0Irq); //清除中断标志
    }
}

//时钟初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t         enFlashWait;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enFlashWait = FlashWaitCycle1;                      //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);                       // Flash 等待1个周期
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);           // 时钟切换
}


//Timer3 Port端口配置
void App_Timer3PortCfg(void)
{
    stc_gpio_cfg_t            stcTIM3Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM3Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //端口外设时钟使能
    
    stcTIM3Port.enDir  = GpioDirIn;
    //PA08设置为TIM3_CH0A
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);
    
    //PA07设置为TIM3_CH0B
    //Gpio_Init(GpioPortA, GpioPin7, &stcTIM3Port);
    //Gpio_SetAfMode(GpioPortA,GpioPin7,GpioAf4);
}

//Timer3 配置
void App_Timer3Cfg(void)
{
    uint16_t                  u16CntValue;
    stc_tim3_mode1_cfg_t      stcTim3BaseCfg;
    stc_tim3_pwc_input_cfg_t  stcTim3PwcInCfg;
    stc_gpio_cfg_t            stcTIM3Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    DDL_ZERO_STRUCT(stcTIM3Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3 外设时钟使能
    
    stcTim3BaseCfg.enWorkMode = Tim3WorkMode1;                //定时器模式
    stcTim3BaseCfg.enCT       = Tim3Timer;                    //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv1;                 //PCLK
    stcTim3BaseCfg.enOneShot  = Tim3PwcOneShotDetect;         //PWC循环检测
    
    Tim3_Mode1_Init(&stcTim3BaseCfg);                         //TIM3 的模式1功能初始化
    
    stcTim3PwcInCfg.enTsSel  = Tim3Ts6IAFP;                   //PWC输入选择 CHA
    stcTim3PwcInCfg.enIA0Sel = Tim3IA0Input;                  //CHA选择IA0
    stcTim3PwcInCfg.enFltIA0 = Tim3FltPCLKDiv16Cnt3;          //PCLK/16 3个连续有效
    //stcTim3PwcInCfg.enIB0Sel = Tim3IB0Input;                //CHB选择IB0
    //stcTim3PwcInCfg.enFltIB0 = Tim3FltPCLKDiv16Cnt3;        //PCLK/16 3个连续有效
    Tim3_M1_Input_Cfg(&stcTim3PwcInCfg);                      //PWC输入设置
    
    M0P_TIM3_MODE23->CRCH0_f.CSA = 1;                         //CHA设置为捕获模式
    
    Tim3_M1_PWC_Edge_Sel(Tim3PwcRiseToRise);                  //上升沿到上升沿捕获
    
    u16CntValue = 0;
    Tim3_M1_Cnt16Set(u16CntValue);                            //设置计数初值  
    
    Tim3_ClearIntFlag(Tim3UevIrq);                            //清Uev中断标志
    Tim3_ClearIntFlag(Tim3CA0Irq);                            //清捕捉中断标志
    Tim3_Mode1_EnableIrq(Tim3UevIrq);                         //使能TIM3溢出中断
    Tim3_Mode1_EnableIrq(Tim3CA0Irq);                         //使能TIM3捕获中断    
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                   //TIM3中断使能
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
    App_ClockCfg();      //时钟初始化
    
    App_Timer3PortCfg(); //Timer3 Port端口配置
    
    App_Timer3Cfg();     //Timer3 配置;
    
    Tim3_M1_Run();       //TIM3 运行
    
    while (1)
    {
        delay1ms(3000);
        Tim3_M1_Run();   //TIM3 运行
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


