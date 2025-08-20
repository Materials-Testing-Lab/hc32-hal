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
 * @brief  Source file for ADC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adc.h"
#include "gpio.h"
#include "bgr.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
volatile uint32_t u32AdcRestult0;
volatile uint32_t u32AdcRestult2;
volatile uint32_t u32AdcRestult5;
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
void App_PortInit(void);
void App_AdcInit(void);
void App_AdcSqrCfg(void);

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
    
    ///< 端口初始化
    App_PortInit();
    
    ///< ADC模块 初始化
    App_AdcInit();
    
    ///< ADC 顺序扫描模式 配置
    App_AdcSqrCfg();
    
    while(1)
    {        
        ///< 等待外部IO触发
        if(TRUE == Adc_GetIrqStatus(AdcMskIrqSqr))
        {
            Adc_ClrIrqStatus(AdcMskIrqSqr);
            Gpio_ClearIrq(STK_USER_PORT, STK_USER_PIN);    
            ///< 获取采样值
            u32AdcRestult0   = Adc_GetSqrResult(AdcSQRCH0MUX);
            u32AdcRestult2   = Adc_GetSqrResult(AdcSQRCH1MUX);
            u32AdcRestult5   = Adc_GetSqrResult(AdcSQRCH2MUX);
        }

    }
}

///< 端口初始化
void App_PortInit(void)
{    
    stc_gpio_cfg_t          stcGpioCfg;
        
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //PA00 (AIN0)
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //PA02 (AIN2)
    Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //PA05 (AIN5)
    
    ///< GPIO IO USER KEY初始化
    stcGpioCfg.enDir = GpioDirIn;                   ///< GPIO 输入 
    stcGpioCfg.enDrv = GpioDrvL;                    ///< GPIO低驱动能力
    stcGpioCfg.enPu = GpioPuDisable;                ///< GPIO无上拉
    stcGpioCfg.enPd = GpioPdDisable;                ///< GPIO无下拉
    stcGpioCfg.enOD = GpioOdDisable;                ///< GPIO开漏输出关闭
    stcGpioCfg.enCtrlMode = GpioAHB;                ///< AHB 总线控制模式
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);          ///< GPIO 初始化
    Gpio_EnableIrq(STK_USER_PORT, STK_USER_PIN, GpioIrqFalling);  ///< GPIO IO中断使能
}

///< ADC模块 初始化
void App_AdcInit(void)
{
    stc_adc_cfg_t              stcAdcCfg;

    ///< 开启ADC/BGR外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE); 
    
    Bgr_BgrEnable();        ///< 开启BGR
    
    ///< ADC 初始化配置
    stcAdcCfg.enAdcMode         = AdcScanMode;              ///<采样模式-扫描
    stcAdcCfg.enAdcClkDiv       = AdcMskClkDiv1;            ///<采样分频-1
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk;      ///<采样周期数-8
    stcAdcCfg.enAdcRefVolSel    = AdcMskRefVolSelAVDD;      ///<参考电压选择-VCC
    stcAdcCfg.enAdcOpBuf        = AdcMskBufDisable;         ///<OP BUF配置-关
    stcAdcCfg.enInRef           = AdcMskInRefDisable;       ///<内部参考电压使能-关
    stcAdcCfg.enAdcAlign        = AdcAlignRight;            ///<转换结果对齐方式-右
    Adc_Init(&stcAdcCfg);                                   ///<
}

///< ADC 顺序扫描模式 配置
void App_AdcSqrCfg(void)
{
    stc_adc_sqr_cfg_t          stcAdcSqrCfg;
    
    ///< 顺序扫描模式功能及通道配置
    ///< 注意：扫描模式下，当配置转换次数为n时，转换通道的配置范围必须为[SQRCH(0)MUX,SQRCH(n-1)MUX]
    stcAdcSqrCfg.bSqrDmaTrig = FALSE;
    stcAdcSqrCfg.enResultAcc = AdcResultAccDisable;
    stcAdcSqrCfg.u8SqrCnt    = 3;
    Adc_SqrModeCfg(&stcAdcSqrCfg);
    
    ///< 配置顺序扫描转换通道
    Adc_CfgSqrChannel(AdcSQRCH0MUX, AdcExInputCH0);
    Adc_CfgSqrChannel(AdcSQRCH1MUX, AdcExInputCH2);
    Adc_CfgSqrChannel(AdcSQRCH2MUX, AdcExInputCH5);
    
    ///< 顺序扫描触发端口选择
    Adc_SqrExtTrigCfg(AdcMskTrigPA07, TRUE);

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


