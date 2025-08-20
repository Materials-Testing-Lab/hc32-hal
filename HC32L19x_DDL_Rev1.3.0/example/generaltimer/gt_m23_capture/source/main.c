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
 * @brief  Source file for General Timer example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "bt.h"
#include "gpio.h"
#include "flash.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint32_t  Bt_CapTemp_Value1,Bt_CapTemp_Value2;
volatile int32_t   Bt_Capture_Value;
volatile uint16_t  Bt_Uev_Cnt;

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
 * TIM0中断服务函数
 ******************************************************************************/
void Tim0_IRQHandler(void)
{
    static uint8_t i;
    
    //Timer0 模式23 捕获中断B
    if(TRUE == Bt_GetIntFlag(TIM0, BtCB0Irq))
    {
        if(0 == i)
        {
            Bt_CapTemp_Value1 = M0P_TIM0_MODE23->CCR0B_f.CCR0B;  //第一次捕获值
            Bt_Uev_Cnt = 0;
            
            i = 1;
        }
        else if(1 == i)
        {
            Bt_CapTemp_Value2 = M0P_TIM0_MODE23->CCR0B_f.CCR0B;  //第二次捕获值
            Bt_Capture_Value = Bt_Uev_Cnt * 0xFFFF + Bt_CapTemp_Value2 - Bt_CapTemp_Value1;  //两次捕获之间的差值
            
            i = 0;
        }
        
        Bt_ClearIntFlag(TIM0,BtCB0Irq); //清除中断标志
    }
    
    //Timer0 模式1 更新中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        Bt_Uev_Cnt++;
        Bt_ClearIntFlag(TIM0,BtUevIrq); //清除中断标志
    }
}

//时钟配置初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t      enWaitCycle;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enWaitCycle = FlashWaitCycle1;
    Flash_WaitCycle(enWaitCycle);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);  ///< 时钟切换
}

//端口配置初始化
void App_Timer0PortCfg(void)
{
    stc_gpio_cfg_t          stcTIM0Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM0Port);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    stcTIM0Port.enDir  = GpioDirIn;
#if 0
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);            //PA00设置为TIM0_CHA
#endif
    Gpio_Init(GpioPortA, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);            //PA01设置为TIM0_CHB
}

//Timer0 配置
void App_Timer0Cfg(void)
{
    uint16_t                   u16ArrValue;
    uint16_t                   u16CntValue;
    stc_bt_mode23_cfg_t        stcBtBaseCfg;
    stc_bt_m23_input_cfg_t     stcBtPortCapCfg;
    stc_gpio_cfg_t             stcTIM0Port;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcTIM0Port);
    DDL_ZERO_STRUCT(stcBtPortCapCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能
    
    stcBtBaseCfg.enWorkMode    = BtWorkMode2;              //锯齿波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv64;              //PCLK/64
    stcBtBaseCfg.enCntDir      = BtCntUp;                  //向上计数，在三角波模式时只读
    
    Bt_Mode23_Init(TIM0, &stcBtBaseCfg);                   //TIM0 的模式2功能初始化
    
    stcBtPortCapCfg.enCh0BCmpCap   = BtCHxCapMode;         //CHB通道设置为捕获模式
    stcBtPortCapCfg.enCH0BCapSel   = BtCHxCapFallRise;     //CHB通道上升沿下降沿捕获都使能
    stcBtPortCapCfg.enCH0BInFlt    = BtFltPCLKDiv16Cnt3;   //PCLK/16 3个连续有效
    stcBtPortCapCfg.enCH0BPolarity = BtPortPositive;       //正常输入输出
    
    Bt_M23_PortInput_Cfg(TIM0, &stcBtPortCapCfg);          //端口输入初始化配置
    
    u16ArrValue = 0xFFFF;
    Bt_M23_ARRSet(TIM0, u16ArrValue, TRUE);                //设置重载值,并使能缓存
    
    u16CntValue = 0;
    Bt_M23_Cnt16Set(TIM0, u16CntValue);                    //设置计数初值
    
    Bt_ClearAllIntFlag(TIM0);                              //清中断标志
    Bt_Mode23_EnableIrq(TIM0,BtUevIrq);                    //使能TIM0 UEV更新中断
    Bt_Mode23_EnableIrq(TIM0,BtCB0Irq);                    //使能TIM0 CB0比较/捕获中断
    EnableNvic(TIM0_IRQn, IrqLevel0, TRUE);                //TIM0中断使能
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
    App_ClockCfg();         //时钟配置初始化
    
    App_Timer0PortCfg();    //端口配置初始化
    
    App_Timer0Cfg();        //Timer0配置初始化
    
    Bt_M23_Run(TIM0);       //TIM0 运行

    while (1);

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


