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
 * @brief  Source file for LVD example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "lvd.h"
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
static void App_LvdPortInit(void);
static void App_LvdInit(void);

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int main(void)
{
    ///< LVD 端口初始化
    App_LvdPortInit();
    ///< LVD 初始化
    App_LvdInit();

    while (1)
    {
        ;
    }
}


///< LVD 中断服务函数
void Lvd_IRQHandler(void)
{
    Lvd_ClearIrq();

    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);    //中断受触发设置限制，LVD输出不受限制
    delay1ms(1000);
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);    //中断受触发设置限制，LVD输出不受限制
}

///< LVD相关端口初始化
static void App_LvdPortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    //开LVD时钟
    
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(STK_LED_PORT,STK_LED_PIN,&stcGpioCfg);
    
    ///< LVD OUT
    Gpio_Init(GpioPortA,GpioPin4,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin4,GpioAf6);
    
    ///< LVD INPUT SOURCE
    Gpio_SetAnalogMode(GpioPortB, GpioPin7);
}

static void App_LvdInit(void)
{
    stc_lvd_cfg_t stcLvdCfg;

    DDL_ZERO_STRUCT(stcLvdCfg);     //变量清0

    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);    //开LVD时钟

    stcLvdCfg.enAct        = LvdActMskInt;              ///< 配置触发产生中断
    stcLvdCfg.enInputSrc   = LvdInputSrcMskPB07;        ///< 配置LVD输入源
    stcLvdCfg.enThreshold  = LvdMskTH1_8V;              ///< 配置LVD基准电压
    stcLvdCfg.enFilter     = LvdFilterMskEnable;        ///< 滤波使能
    stcLvdCfg.enFilterTime = LvdFilterMsk28_8ms;        ///< 滤波时间设置
    stcLvdCfg.enIrqType    = LvdIrqMskFall;             ///< 中断触发类型
    Lvd_Init(&stcLvdCfg);
    
    ///< 中断开启
    Lvd_EnableIrq();
    Lvd_ClearIrq();
    EnableNvic(LVD_IRQn, IrqLevel3, TRUE);              ///< NVIC 中断使能
    
    ///< LVD 模块使能
    Lvd_Enable();
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


