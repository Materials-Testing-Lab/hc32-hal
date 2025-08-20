/******************************************************************************
 * Copyright (C) 2024, Xiaohua Semiconductor Co., Ltd. All rights reserved.
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
 * @brief  Source file for GPIO example
 *
 * @author MADS Team
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "bt.h"
#include "gpio.h"
#include "sysctrl.h"
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
#define IR_PORT     GpioPortB
#define IR_PIN      GpioPin9
#define IR_FUNC_SEL GpioAf2
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_IrFuncSet(void);
static void App_Timer0Cfg(uint16_t u16Period);

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
    // IR function setting
    App_IrFuncSet();
    delay1ms(1000);

    // Timer0 setting
    App_Timer0Cfg(52U);  // Timer0配置初始化(频率 = 4M/32分频/52= 2404)
    Bt_M0_Run(TIM0);     // TIM0 运行

    while (1)
    {
        ;
    }
}

// IR 功能设置
static void App_IrFuncSet(void)
{
    // 使能RCL 38400
    Sysctrl_SetRCLTrim(SysctrlRclFreq38400);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);

    // 开启 GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    // 配置复用功能为IR输出
    Gpio_SetAfMode(IR_PORT, IR_PIN, IR_FUNC_SEL);

    // 设置红外信号输出极性
    Gpio_SfIrPolCfg(GpioSfIrN);

    // 设置输出端口，端口功能输出为H电平
    Gpio_ClrIO(IR_PORT, IR_PIN);                                       // PxOUT=0，红外极性设置为反向电平，端口电平为H
    SetBit((uint32_t)&M0P_GPIO->PAADS + IR_PORT, IR_PIN, FALSE);       // 设置为数字端口
    SetBit((uint32_t)&M0P_GPIO->PADIR + IR_PORT, IR_PIN, GpioDirOut);  // 设置为输出端口
}

// Timer0中断服务程序：软件模拟发送UART数据0x55
void Tim0_IRQHandler(void)
{
    static uint8_t i = 0U;

    if (i == 0U)
    {
        Gpio_ClrIO(IR_PORT, IR_PIN);
        i = 1U;
    }
    else
    {
        Gpio_SetIO(IR_PORT, IR_PIN);
        i = 0U;
    }

    Bt_ClearIntFlag(TIM0, BtUevIrq);  // 中断标志清零
}

// Timer0配置初始化
static void App_Timer0Cfg(uint16_t u16Period)
{
    uint16_t           u16ArrValue;
    uint16_t           u16CntValue;
    stc_bt_mode0_cfg_t stcBtBaseCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE);  // Base Timer外设时钟使能

    DDL_ZERO_STRUCT(stcBtBaseCfg);
    stcBtBaseCfg.enWorkMode = BtWorkMode0;     // 定时器模式
    stcBtBaseCfg.enCT       = BtTimer;         // 定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS      = BtPCLKDiv32;     // PCLK/32
    stcBtBaseCfg.enCntMode  = Bt16bitArrMode;  // 自动重载16位计数器/定时器
    stcBtBaseCfg.bEnTog     = FALSE;
    stcBtBaseCfg.bEnGate    = FALSE;
    stcBtBaseCfg.enGateP    = BtGatePositive;
    Bt_Mode0_Init(TIM0, &stcBtBaseCfg);  // TIM0 的模式0功能初始化

    u16ArrValue = 0x10000 - u16Period;
    Bt_M0_ARRSet(TIM0, u16ArrValue);  // 设置重载值(ARR = 0x10000 - 周期)

    u16CntValue = 0x10000 - u16Period;
    Bt_M0_Cnt16Set(TIM0, u16CntValue);  // 设置计数初值

    Bt_ClearIntFlag(TIM0, BtUevIrq);         // 清中断标志
    Bt_Mode0_EnableIrq(TIM0);                // 使能TIM0中断(模式0时只有一个中断)
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);  // TIM0中断使能
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
