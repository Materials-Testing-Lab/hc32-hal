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
 * @brief  Source file for DAC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "sysctrl.h"
#include "dac.h"
#include "flash.h"
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

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_SysClkInit(void);               ///< 系统时钟初始化
static void App_GpioInit(void);                 ///< GPIO初始化
static void App_DACInit(void);                  ///< DAC初始化

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
    App_SysClkInit();               ///< 系统时钟初始化
    App_GpioInit();                 ///< GPIO初始化
    App_DACInit();                  ///< DAC初始化

    while(1)
    {
        Dac_SoftwareTriggerCmd();   ///< 软件触发
        delay10us(12);
    }

}

/**
 ******************************************************************************
 ** \brief  初始化外部系统时钟
 **
 ** \return 无
 ******************************************************************************/
static void App_SysClkInit(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;
    stc_sysctrl_pll_cfg_t stcPLLCfg;


    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);    ///< 使能FLASH模块的外设时钟
    Flash_WaitCycle(FlashWaitCycle1);
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);             ///< PLL使用RCH作为时钟源，因此需要先设置RCH

    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     ///< RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  ///< PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              ///< 输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            ///< 4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);

    ///< 选择PLL作为HCLK时钟源;
    stcCfg.enClkSrc  = SysctrlClkPLL;
    ///< HCLK SYSCLK/2
    stcCfg.enHClkDiv = SysctrlHclkDiv1;
    ///< PCLK 为HCLK/8
    stcCfg.enPClkDiv = SysctrlPclkDiv1;
    ///< 系统时钟初始化
    Sysctrl_ClkInit(&stcCfg);
}



/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    // 使能GPIO模块的外设时钟

    Gpio_SetAnalogMode(GpioPortA, GpioPin4);       //PA04作为DAC的模拟输出
}

/**
******************************************************************************
    ** \brief  配置DAC
    **
    ** @param  无
    ** \retval 无
    **
******************************************************************************/
static void App_DACInit(void)
{
    stc_dac_cfg_t  dac_initstruct;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralDac, TRUE);     ///< 使能DAC模块的时钟

    dac_initstruct.boff_t = DacBoffDisable;
    dac_initstruct.ten_t  = DacTenEnable;
    dac_initstruct.sref_t = DacVoltageAvcc;
    dac_initstruct.wave_t = DacTrWaveEnable;      ///< 三角波触发
    dac_initstruct.mamp_t = DacMenp4095;
    dac_initstruct.tsel_t = DacSwTriger;          ///< 软件触发方式
    dac_initstruct.align  = DacRightAlign;        ///< 右对齐
    dac_initstruct.dhr12  = 0;

    Dac_Init(&dac_initstruct);
    Dac_Cmd(TRUE);
    Dac_DmaCmd(TRUE);                       ///< DAC通道DMA使能

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


