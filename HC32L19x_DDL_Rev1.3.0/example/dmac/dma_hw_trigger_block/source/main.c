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
 * @brief  Source file for DMAC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
* Include files
******************************************************************************/
#include "ddl.h"
#include "dmac.h"
#include "adc.h"
#include "gpio.h"
#include "bgr.h"
/******************************************************************************
* Local pre-processor symbols/macros ('#define')                            
******************************************************************************/

/******************************************************************************
* Global variable definitions (declared in header file with 'extern')
******************************************************************************/
static volatile uint16_t u16AdcRestult0;
static volatile uint16_t u16AdcRestult2;
static volatile uint16_t u16AdcRestult5;

//static stc_adc_irq_t stcAdcIrqFlag;
/******************************************************************************
* Local type definitions ('typedef')                                         
******************************************************************************/

/******************************************************************************
* Local variable definitions ('static')                                      *
******************************************************************************/
static uint32_t ADC_Result_Array[16] = {0};
/******************************************************************************
* Local pre-processor symbols/macros ('#define')                             
******************************************************************************/

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
******************************************************************************/
void App_PortCfg(void);
void App_AdcInit(void);
void App_AdcSqrCfg(void);
void App_DmaCfg(void);
/**
******************************************************************************
** \brief  Main function of project
**
** \return uint32_t return value, if needed
**
******************************************************************************/
int32_t main(void)
{        
    // 端口功能配置
    App_PortCfg();
    
    // ADC模块初始化
    App_AdcInit();
    
    // ADC 顺序扫描功能 配置
    App_AdcSqrCfg();
    
    // DMA通道配置， ADC SQR触发DMA传输
    App_DmaCfg();
    
    // 启动DMA通道传输，因为DMA配置中 TC = 3,所以需触发三次
    Adc_SQR_Start();    //启动ADC顺序扫描转换，转换完成后，自动启动DMA传输
    delay1ms(500);
    Adc_SQR_Start();    //启动ADC顺序扫描转换，转换完成后，自动启动DMA传输
    delay1ms(500);
    Adc_SQR_Start();    //启动ADC顺序扫描转换，转换完成后，自动启动DMA传输

    // 等待传输完成
    while(Dma_GetStat(DmaCh0) != DmaTransferComplete);
    
    while (1)
    {
      ;
    }
}

// 端口功能配置
void App_PortCfg(void)
{
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  // 开启GPIO时钟模块
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //配置PA00 (AIN0)
    Gpio_SetAnalogMode(GpioPortA, GpioPin2);        //配置PA02 (AIN2)
    Gpio_SetAnalogMode(GpioPortA, GpioPin5);        //配置PA05 (AIN5)
}

// ADC模块初始化
void App_AdcInit(void)
{
    stc_adc_cfg_t              stcAdcCfg;

    DDL_ZERO_STRUCT(stcAdcCfg);  // 初始值清零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr,TRUE);   // 开启ADC和BGR时钟模块
    
    Bgr_BgrEnable();            //使能BGR
    
    stcAdcCfg.enAdcMode = AdcScanMode;                 //扫描模式
    stcAdcCfg.enAdcClkDiv = AdcMskClkDiv1;             //ADC时钟分频为PCLK
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle8Clk; //采样时钟选择为8个采样时钟
    stcAdcCfg.enAdcRefVolSel = AdcMskRefVolSelAVDD;    //ADC参考电压为AVDD
    stcAdcCfg.enAdcOpBuf = AdcMskBufDisable;           //关闭放大器BUF  
    stcAdcCfg.enInRef = AdcMskInRefDisable;            //内部参考电压关闭
    stcAdcCfg.enAdcAlign = AdcAlignRight;              //ADC运算结果右对齐
  
    Adc_Init(&stcAdcCfg);      //初始化ADC
}

// ADC 顺序扫描功能 配置
void App_AdcSqrCfg(void)
{
    stc_adc_sqr_cfg_t          stcAdcSqrCfg;
    
    DDL_ZERO_STRUCT(stcAdcSqrCfg);  //结构体变量 初始值清零

    //配置顺序扫描转换通道,转换通道0 4 7配置为AIN0;转换通道1 5 8配置为AIN2;转换通道2 6 9配置为AIN5
    Adc_CfgSqrChannel(AdcSQRCH0MUX, AdcExInputCH0);    
    Adc_CfgSqrChannel(AdcSQRCH1MUX, AdcExInputCH2);
    Adc_CfgSqrChannel(AdcSQRCH2MUX, AdcExInputCH5);
    Adc_CfgSqrChannel(AdcSQRCH4MUX, AdcExInputCH0);
    Adc_CfgSqrChannel(AdcSQRCH5MUX, AdcExInputCH2);
    Adc_CfgSqrChannel(AdcSQRCH6MUX, AdcExInputCH5);
    Adc_CfgSqrChannel(AdcSQRCH7MUX, AdcExInputCH0);
    Adc_CfgSqrChannel(AdcSQRCH8MUX, AdcExInputCH2);
    Adc_CfgSqrChannel(AdcSQRCH9MUX, AdcExInputCH5);
    
    EnableNvic(ADC_DAC_IRQn, IrqLevel3, TRUE);    //使能NVIC对应中断位
    
    Adc_EnableIrq();  //使能ADC中断功能

    stcAdcSqrCfg.enResultAcc = AdcResultAccDisable;//禁用ADC转换结果自动累加功能
    stcAdcSqrCfg.u8SqrCnt = 9;                     //ADC顺序扫描转换次数
    stcAdcSqrCfg.bSqrDmaTrig = TRUE;               //ADC顺序扫描转换完成后，触发DMA数据传输
    Adc_SqrModeCfg(&stcAdcSqrCfg);                 //配置顺序扫描转换模式
}

// DMA通道配置， ADC SQR触发DMA传输
void App_DmaCfg(void)
{
    stc_dma_cfg_t           stcDmaCfg;    
    
    DDL_ZERO_STRUCT(stcDmaCfg);                     //结构体变量 初始值清零

    // 使能 DMA时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);
    
    stcDmaCfg.enMode =  DmaMskBlock;                            //选择块传输
    stcDmaCfg.u16BlockSize = 0x03u;                             //块传输个数
    stcDmaCfg.u16TransferCnt = 0x03u;                           //Block模式，一次传输数据大小为 3,传输三次
    stcDmaCfg.enTransferWidth = DmaMsk32Bit;                    //传输数据的宽度，此处选择字(32Bit)宽度
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;                 //源地址自增
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrInc;                 //目的地址自增
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadDisable; //禁止重新加载传输目的地址
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;   //使能重新加载传输源地址
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;      //使能重新加载BC/TC值
    stcDmaCfg.u32SrcAddress = (uint32_t) &(M0P_ADC->SQRRESULT0);//指定传输源地址
    stcDmaCfg.u32DstAddress = (uint32_t)&ADC_Result_Array[0];   //指定传输目的地址
    stcDmaCfg.enTransferMode = DmaMskContinuousTransfer;        //DMAC 在传输完成时不清除 CONFA:ENS 位。这个功能允许连续传输而不需要 CPU 干预。
    stcDmaCfg.enRequestNum = DmaADCSQRTrig;                     //设置为ADC SQR触发
    
    Dma_InitChannel(DmaCh0,&stcDmaCfg);       //初始化DMA通道0
  
    //使能DMA，使能DMA0
    Dma_Enable();
    Dma_EnableChannel(DmaCh0);
}

/******************************************************************************
* Adc 中断处理函数
******************************************************************************/
void Adc_IRQHandler(void)
{    
    if(TRUE == Adc_GetIrqStatus(AdcMskIrqSqr))  //adc顺序扫描完成中断
    {
        //获取采样值
        u16AdcRestult0 = Adc_GetSqrResult(AdcSQRCH0MUX);  
        u16AdcRestult2 = Adc_GetSqrResult(AdcSQRCH1MUX);
        u16AdcRestult5 = Adc_GetSqrResult(AdcSQRCH2MUX);
            
        Adc_ClrIrqStatus(AdcMskIrqSqr);  //清中断状态位
    }        
    
}

/******************************************************************************
* EOF (not truncated)
******************************************************************************/


