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
 * @brief  Source file for SYSCTRL example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "sysctrl.h"
#include "gpio.h"
#include "flash.h"

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
//等待按键按下
static void _UserKeyWait(void)
{
    while(TRUE == Gpio_GetInputIO(STK_USER_PORT, STK_USER_PIN));
    delay1ms(500);
}
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_PortCfg(void);
void App_ClkDivInit(void);
void App_SystemClkInit_RCH(en_sysctrl_rch_freq_t enRchFreq);
void App_SystemClkInit_RCL(en_sysctrl_rcl_freq_t enRclFreq);
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq);
void App_SystemClkInit_XTL(void);
void App_SystemClkInit_PLL48M_byRCH(void);
void App_SystemClkInit_PLL48M_byXTH(void);

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** check Pxx to verify the clock frequency.
 **
 ******************************************************************************/
int32_t main(void)
{   
    //端口配置，按键按下，继续运行
    App_PortCfg();
    
    //时钟分频初始化
    App_ClkDivInit();
    
    //时钟切换
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_RCH(SysctrlRchFreq8MHz);
    
    _UserKeyWait(); //USER KEY 按下后继续执行    
    App_SystemClkInit_RCH(SysctrlRchFreq16MHz);
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_RCH(SysctrlRchFreq22_12MHz);
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_RCH(SysctrlRchFreq24MHz);
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_RCL(SysctrlRclFreq32768);
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_RCL(SysctrlRclFreq38400);
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_XTH(SysctrlXthFreq24_32MHz);
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_XTL();
    
    _UserKeyWait(); //USER KEY 按下后继续执行    
    App_SystemClkInit_PLL48M_byRCH();
    
    _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_RCH(SysctrlRchFreq4MHz);
    
    while (1)
    {
        ;
    }
}


//时钟初始化配置
void App_ClkDivInit(void)
{
    //时钟分频设置
    Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
}


//端口配置，按键按下，继续运行
void App_PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    
    
    ///<========================== 按键端口配置 ===========================
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO 初始化(在STK上外接KEY(USER))
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);
    ///< User KEY 按下后程序继续执行
    _UserKeyWait();
    
    ///<========================== 时钟输出端口模式配置 ===========================
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->无上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO PA01初始化
    Gpio_Init(GpioPortA, GpioPin1, &stcGpioCfg);    
    ///< 配置PA01复用功能为HCLK输出
    Gpio_SetAfMode(GpioPortA, GpioPin1, GpioAf6);
    
    ///< 使能HCLK从PA01输出
    Gpio_SfHClkOutputCfg(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
}


void App_SystemClkInit_RCH(en_sysctrl_rch_freq_t enRchFreq)
{  
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< 加载目标频率的RCH的TRIM值
    Sysctrl_SetRCHTrim(enRchFreq);
    ///< 使能RCH
    Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
    ///< 时钟切换到RCH
    Sysctrl_SysClkSwitch(SysctrlClkRCH);
    
    ///< HCLK不超过24M：此处设置FLASH读等待周期为0 cycle
    Flash_WaitCycle(FlashWaitCycle0);
    
}

void App_SystemClkInit_RCL(en_sysctrl_rcl_freq_t enRclFreq)
{
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL
    Sysctrl_SetRCLTrim(enRclFreq);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< HCLK不超过24M：此处设置FLASH读等待周期为0 cycle
    Flash_WaitCycle(FlashWaitCycle0);    
}

#ifdef SYSTEM_XTH
///<请注意根据外部晶振配置宏——[SYSTEM_XTH]
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq)
{
    ///<======================== 切换至XTH32MHz ==============================    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    if(SysctrlXthFreq24_32MHz == enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle1);    
    }
    
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(enXthFreq);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);

    Sysctrl_SysClkSwitch(SysctrlClkXTH);
    
    if(SysctrlXthFreq24_32MHz != enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle0);    
    }
}
#endif

#ifdef SYSTEM_XTL
void App_SystemClkInit_XTL(void)
{
    ///< 切换时钟前（根据外部低速晶振）设置XTL晶振参数，使能目标时钟，SYSTEM_XTL = 32768Hz
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp3, SysctrlXtalDriver3);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkXTL);

    Flash_WaitCycle(FlashWaitCycle0);    

}
#endif

void App_SystemClkInit_PLL48M_byRCH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< 加载目标频率的RCH的TRIM值
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);
    ///< 使能RCH
    Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    Flash_WaitCycle(FlashWaitCycle1);    
    
    ///< 使能PLL
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    ///< 时钟切换到PLL
    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}

#if (SYSTEM_XTH == 8000000u)
///<请注意根据外部晶振配置宏——[SYSTEM_XTH],如果使用PLL，XTH必须小于24MHz
void App_SystemClkInit_PLL48M_byXTH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;    //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul6;             //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    Flash_WaitCycle(FlashWaitCycle1);    

    ///< 使能PLL
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);    
    ///< 时钟切换到PLL
    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}
#endif

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



