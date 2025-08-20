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
 * @brief  Source file for WDT example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/

#include "wdt.h"
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

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

uint8_t u32CountWdt;
uint8_t u8Flag=0xFF;

static void App_GpioInit(void);
static void App_WdtInit(void);

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
    ///< ===============================================
    ///< ============ 警告，警告，警告！！！=============
    ///< ===============================================
    ///< 本样例程序会进入深度休眠模式，因此以下两行代码起防护作用（防止进入深度
    ///< 休眠后芯片调试功能不能再次使用），
    ///< 在使用本样例时，禁止在没有唤醒机制的情况下删除以下两行代码。
    delay1ms(2000);
    
    ///< GPIO 初始化
    App_GpioInit();
    while(TRUE == Gpio_GetInputIO(STK_USER_PORT, STK_USER_PIN));

    ///< WDT 初始化
    App_WdtInit();
   
    ///< 启动 WDT
    Wdt_Start();
       
    ///< 进入深度休眠模式——使能唤醒后自动休眠特性   
    Lpm_GotoDeepSleep(TRUE);

    while (1)
    {
        ;
    }
}


///< WDT 中断服务程序
void Wdt_IRQHandler(void)
{
    if(Wdt_GetIrqStatus())
    {
        Wdt_IrqClr();       ///<清除 wdt 中断标记
        
        u8Flag = ~u8Flag;
        if(u8Flag)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);
        }
        else
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);
        }

        u32CountWdt++;
    }

}

static void App_WdtInit(void)
{
    ///< 开启WDT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt,TRUE);
    ///< WDT 初始化
    Wdt_Init(WdtIntEn, WdtT820ms);
    ///< 开启NVIC中断
    EnableNvic(WDT_IRQn, IrqLevel3, TRUE);
}



static void _LowPowerModeGpioSet(void)
{
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    //swd as gpio
    Sysctrl_SetFunc(SysctrlSWDUseIOEn, TRUE);
    
    //初始化IO配置(follow STK)
    M0P_GPIO->PAADS = 0;
    M0P_GPIO->PBADS = 0;
    M0P_GPIO->PCADS = 0;
    M0P_GPIO->PDADS = 0;
    M0P_GPIO->PEADS = 0;
    M0P_GPIO->PFADS = 0;
    
    M0P_GPIO->PADIR = 0XFFFF;
    M0P_GPIO->PBDIR = 0XFFFF;
    M0P_GPIO->PCDIR = 0XFFFF;
    M0P_GPIO->PDDIR = 0XFFFF;
    M0P_GPIO->PEDIR = 0XFFFF;
    M0P_GPIO->PFDIR = 0XFFFF;
    
    M0P_GPIO->PAPD = 0xFFFF;
    M0P_GPIO->PBPD = 0xFFFF;
    M0P_GPIO->PCPD = 0xFFFF;
    M0P_GPIO->PDPD = 0xFFFF;
    M0P_GPIO->PEPD = 0xFFFF;
    M0P_GPIO->PFPD = 0xFFFF;
    
}

static void App_GpioInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE); 
    
    _LowPowerModeGpioSet();
    
    ///< LED 初始化
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_WriteOutputIO(STK_LED_PORT,STK_LED_PIN,TRUE);
    Gpio_Init(STK_LED_PORT,STK_LED_PIN,&stcGpioCfg);
    
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    
    ///< GPIO IO USER KEY初始化
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);
    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


