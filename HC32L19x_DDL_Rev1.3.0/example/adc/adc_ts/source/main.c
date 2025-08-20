/******************************************************************************
 * Copyright (C) 2023, Xiaohua Semiconductor Co., Ltd. All rights reserved.
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
 * @brief  Source file for TS example
 *
 * @author MADS Team
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "adc.h"
#include "bgr.h"
#include "bt.h"
#include "gpio.h"
#include "flash.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define ADC_CONV_TIMES 10 // ADC 单次采样次数
#define REF_VOL_SELECT REF_VOL_15V
#define REF_VOL_15V 0 // 1.5V参考电压
#define REF_VOL_25V 1 // 2.5V参考电压

#if (REF_VOL_SELECT == REF_VOL_15V)
#define VOL_REF 1.5          // 1.5V参考电压
#define CALI_ADDR 0x00100C34 // 1.5V校准值存放地址
#else
#define VOL_REF 2.5          // 2.5V参考电压
#define CALI_ADDR 0x00100C36 // 2.5V校准值存放地址
#endif
#define TEMPER_ROOM 25.0
#define TEMPER_PARA 0.0795

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
volatile float32_t f32TemperValue; // 温度值
volatile boolean_t bAdcConvFinish; // 总采样次数完成标志位
uint16_t u16AdcResultAvr; // ADC采样平均值
volatile uint16_t u16AdcResult[ADC_CONV_TIMES];
uint16_t u16AdcResultIdx = 0;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

// TIM0 中断服务程序
void Tim0_IRQHandler(void)
{
    // Timer0 模式0 溢出中断
    if (TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        Adc_SGL_Start();                 // 启动单次扫描采样
        Bt_ClearIntFlag(TIM0, BtUevIrq); // 中断标志清零
    }
}

// ADC 中断服务程序
void Adc_IRQHandler(void)
{
    if (TRUE == Adc_GetIrqStatus(AdcMskIrqSgl))
    {
        Adc_ClrIrqStatus(AdcMskIrqSgl); // 清除中断标志位

        u16AdcResult[u16AdcResultIdx] = Adc_GetSglResult(); // 获取采样值
        u16AdcResultIdx++;
        if (u16AdcResultIdx >= ADC_CONV_TIMES)
        {
            u16AdcResultIdx = 0;
            bAdcConvFinish = TRUE; // 采样次数转换完成标志位
        }
        Adc_SGL_Stop(); // ADC 单次转换停止
    }
}

// XTH配置初始化
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq)
{
    //======================== 切换至XTH32MHz ==============================
    // 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    if (SysctrlXthFreq24_32MHz == enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle1);
    }

    // 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(enXthFreq);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);

    Sysctrl_SysClkSwitch(SysctrlClkXTH);

    if (SysctrlXthFreq24_32MHz != enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle0);
    }
}

// Timer0配置初始化
void App_Timer0Cfg(uint16_t u16Period)
{
    uint16_t u16ArrValue;
    uint16_t u16CntValue;
    stc_bt_mode0_cfg_t stcBtBaseCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); // Base Timer外设时钟使能

    DDL_ZERO_STRUCT(stcBtBaseCfg);
    stcBtBaseCfg.enWorkMode = BtWorkMode0;   // 定时器模式
    stcBtBaseCfg.enCT = BtTimer;             // 定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS = BtPCLKDiv256;       // PCLK/256
    stcBtBaseCfg.enCntMode = Bt16bitArrMode; // 自动重载16位计数器/定时器
    stcBtBaseCfg.bEnTog = FALSE;
    stcBtBaseCfg.bEnGate = FALSE;
    stcBtBaseCfg.enGateP = BtGatePositive;
    Bt_Mode0_Init(TIM0, &stcBtBaseCfg); // TIM0 的模式0功能初始化

    u16ArrValue = 0x10000 - u16Period;
    Bt_M0_ARRSet(TIM0, u16ArrValue); // 设置重载值(ARR = 0x10000 - 周期)

    u16CntValue = 0x10000 - u16Period;
    Bt_M0_Cnt16Set(TIM0, u16CntValue); // 设置计数初值

    Bt_ClearIntFlag(TIM0, BtUevIrq);        // 清中断标志
    Bt_Mode0_EnableIrq(TIM0);               // 使能TIM0中断(模式0时只有一个中断)
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE); // TIM0中断使能
}

// ADC模块初始化
void App_AdcCfg(void)
{
    stc_adc_cfg_t stcAdcCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    Bgr_BgrEnable();        // 开启BGR
    Bgr_TempSensorEnable(); // 开启TS
    // ADC 初始化配置
    DDL_ZERO_STRUCT(stcAdcCfg);
    stcAdcCfg.enAdcMode = AdcSglMode;                  // 采样模式-单次转换模式
    stcAdcCfg.enAdcClkDiv = AdcMskClkDiv8;             // 采样分频-8
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk; // 采样周期数-8
#if (REF_VOL_SELECT == REF_VOL_15V)
    stcAdcCfg.enAdcRefVolSel = AdcMskRefVolSelInBgr1p5; // 参考电压选择-1.5V
#else
    stcAdcCfg.enAdcRefVolSel = AdcMskRefVolSelInBgr2p5; // 参考电压选择-2.5v
#endif
    stcAdcCfg.enAdcOpBuf = AdcMskBufEnable; // OP BUF配置-开
    stcAdcCfg.enInRef = AdcMskInRefEnable;  // 内部参考电压使能-开
    stcAdcCfg.enAdcAlign = AdcAlignRight;   // 转换结果对齐方式-右
    Adc_Init(&stcAdcCfg);
}

// ADC单次转换配置初始化
void App_AdcSglCfg(void)
{
    // ADC 采样通道配置
    Adc_CfgSglChannel(AdcAiTsInput);

    // ADC 中断使能
    Adc_ClrIrqStatus(AdcMskIrqSgl);
    Adc_EnableIrq();
    EnableNvic(ADC_DAC_IRQn, IrqLevel3, TRUE);
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
    uint16_t u16TmpIdx = 0;
    uint32_t u32TmpResult = 0;

    App_SystemClkInit_XTH(SysctrlXthFreq24_32MHz); // 时钟配置初始化
    App_AdcCfg();                                  // 片上温度传感器配置初始化
    App_AdcSglCfg();                               // ADC 单次扫描 功能配置
    App_Timer0Cfg(32000);                          // Timer0配置初始化(周期 = 32000*(1/32M)*256 = 256ms)

    Bt_M0_Run(TIM0); // TIM0 运行

    while (1)
    {
        if (bAdcConvFinish == TRUE)
        {
            bAdcConvFinish = FALSE;
            for (u16TmpIdx = 0, u32TmpResult = 0; u16TmpIdx < ADC_CONV_TIMES; u16TmpIdx++)
            {
                u32TmpResult = u32TmpResult + u16AdcResult[u16TmpIdx]; // 计算采样累加值
            }
            u16AdcResultAvr = (uint16_t)(u32TmpResult / ADC_CONV_TIMES);                                             // 计算采样平均值
            f32TemperValue = TEMPER_ROOM + TEMPER_PARA * VOL_REF * ((u16AdcResultAvr - (*(uint16_t *)(CALI_ADDR)))); // 输出温度值
        }
    }
}

/************************************************ ******************************
 * EOF (not truncated)
 ******************************************************************************/
