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
 * @brief  Source file for ADT example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adt.h"
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

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void Tim4_IRQHandler(void)
{
    static uint8_t i;
    
    //上溢中断
    if(TRUE == Adt_GetIrqFlag(M0P_ADTIM4, AdtOVFIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);  //PD14输出高
            i = 1;
        }
        else
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);  //PD14输出低
            i = 0;
        }

        Adt_ClearIrqFlag(M0P_ADTIM4, AdtOVFIrq);
    }
}


///< AdvTimer端口初始化
void App_AdtPortInit(void)
{
    stc_gpio_cfg_t           stcTIM4Port;
    stc_gpio_cfg_t           stcLEDPort;
    
    DDL_ZERO_STRUCT(stcTIM4Port);
    DDL_ZERO_STRUCT(stcLEDPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);  //端口外设时钟使能
    
    stcTIM4Port.enDir  = GpioDirIn;
    //PA03设置为TIRA
    Gpio_Init(GpioPortA, GpioPin3, &stcTIM4Port);
    
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);  //PD14初始化设置输出低
    stcLEDPort.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcLEDPort);     //PD14设置为输出
}


///< AdvTimer初始化
void App_AdvTimerInit(void)
{
    uint16_t                 u16Period;
    stc_adt_basecnt_cfg_t    stcAdtBaseCntCfg;
    stc_adt_CHxX_port_cfg_t  stcAdtTIM4ACfg;
    stc_adt_CHxX_port_cfg_t  stcAdtTIM4BCfg;
    stc_adt_port_trig_cfg_t  stcAdtPortTrigCfg;

    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtTIM4ACfg);
    DDL_ZERO_STRUCT(stcAdtTIM4BCfg);
    DDL_ZERO_STRUCT(stcAdtPortTrigCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE); //ADT外设时钟使能
    
    
    u16Period = 0x2000;
    Adt_SetPeriod(M0P_ADTIM4, u16Period);                      //周期设置
    
    stcAdtPortTrigCfg.enTrigSrc = AdtTrigxSelPA3;
    stcAdtPortTrigCfg.enFltClk = AdtFltClkPclk0Div16;
    stcAdtPortTrigCfg.bFltEn = TRUE;
    Adt_PortTrigCfg(AdtTrigA, &stcAdtPortTrigCfg);      //PA03设置为TRIA
    
    Adt_CfgHwCntUp(M0P_ADTIM4, AdtHwCntTimTriARise);    //硬件递加事件条件配置: TRIA端口上采样到上升沿
    
    Adt_ClearAllIrqFlag(M0P_ADTIM4);
    Adt_CfgIrq(M0P_ADTIM4, AdtOVFIrq, TRUE);    //AdvTimer4上溢中断使能
    EnableNvic(ADTIM4_IRQn, IrqLevel3, TRUE);   //AdvTimer4中断使能
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
    App_AdtPortInit();         //AdvTimer4 端口初始化
    
    App_AdvTimerInit();        //AdvTimer4 初始化
    
    Adt_StartCount(M0P_ADTIM4);   //AdvTimer4 运行
    
    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


