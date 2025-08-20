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
void App_PortCfg(void);
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
    ///< 配置LED端口
    App_PortCfg();
    
    ///< 内核函数，SysTick配置，定时1s，系统时钟默认RCH 4MHz
    SysTick_Config(SystemCoreClock);
    
    while (1)
    {
        ;
    }
}

//systick中断函数
void SysTick_IRQHandler(void)
{
    static boolean_t bFlag = TRUE;
    
    if(TRUE == bFlag)
    {
        bFlag = FALSE;
        Gpio_SetIO(STK_LED_PORT, STK_LED_PIN);    
    }
    else
    {
        bFlag = TRUE;
        Gpio_ClrIO(STK_LED_PORT, STK_LED_PIN);    
    }
}

//配置LED端口
void App_PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->下拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO (LED)初始化
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcGpioCfg);      
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



