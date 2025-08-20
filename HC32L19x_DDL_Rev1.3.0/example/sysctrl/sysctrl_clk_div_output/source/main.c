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
static void App_UserKeyWait(void);
static void App_PortCfg(void);
static void App_ClkCfg(void);
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
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
    //系统时钟配置
    App_ClkCfg();
    
    //端口配置：等待按键按下，配置相关时钟信号从IO口输出
    App_PortCfg();
    
    while (1)
    {
        ;
    }
}

//系统时钟配置
static void App_ClkCfg(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;
    
    ///< 选择内部RCH(系统默认RCH 4MHz)作为HCLK时钟源;
    stcCfg.enClkSrc    = SysctrlClkRCH;
    ///< HCLK SYSCLK/2
    stcCfg.enHClkDiv   = SysctrlHclkDiv2;
    ///< PCLK 为HCLK/8
    stcCfg.enPClkDiv   = SysctrlPclkDiv8;
    ///< 系统时钟初始化
    Sysctrl_ClkInit(&stcCfg);    
}

//端口配置：等待按键按下，配置相关时钟信号从IO口输出
static void App_PortCfg(void)
{
    stc_gpio_cfg_t pstcGpioCfg;
    
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    //USER KEY按下后继续执行程序
    App_UserKeyWait(); 

    ///< 端口方向配置->输出
    pstcGpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置->高驱动能力
    pstcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->无上下拉
    pstcGpioCfg.enPu = GpioPuDisable;
    pstcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    pstcGpioCfg.enOD = GpioOdDisable;    
    ///< GPIO IO PB00初始化
    Gpio_Init(GpioPortB, GpioPin0, &pstcGpioCfg);
    ///< GPIO IO PA01初始化
    Gpio_Init(GpioPortA, GpioPin1, &pstcGpioCfg);
    ///< GPIO IO PA02初始化
    Gpio_Init(GpioPortA, GpioPin2, &pstcGpioCfg);
    
    ///< 设置RCH从PB00输出
    Gpio_SetAfMode(GpioPortB, GpioPin0, GpioAf5);
    
    ///< 设置HCLK从PA01输出
    Gpio_SfHClkOutputCfg(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
    Gpio_SetAfMode(GpioPortA, GpioPin1, GpioAf6);
    
    ///< 设置PCLK从PA02输出
    Gpio_SfPClkOutputCfg(GpioSfPclkOutEnable, GpioSfPclkOutDiv1);
    Gpio_SetAfMode(GpioPortA, GpioPin2, GpioAf6);
}

//等待按键按下
static void App_UserKeyWait(void)
{
    stc_gpio_cfg_t pstcGpioCfg;
    
    ///< 端口方向配置->输出
    pstcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    pstcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    pstcGpioCfg.enPu = GpioPuEnable;
    pstcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    pstcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO 初始化(STK上外接KEY(USER))
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &pstcGpioCfg);
    ///< 检测电平(USER按键是否按下(低电平))
    while(TRUE == Gpio_GetInputIO(STK_USER_PORT, STK_USER_PIN));    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



