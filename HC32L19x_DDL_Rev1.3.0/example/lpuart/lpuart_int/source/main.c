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
 * @brief  Source file for LPUART example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "lpuart.h"
#include "lpm.h"
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
uint8_t u8TxData[2] = {0x00,0x55};
uint8_t u8RxData = 00;
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_LpUartPortCfg(void);
void App_LpUartCfg(void);
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
    ///< 端口配置
    App_LpUartPortCfg();
    
    ///< LPUART配置
    App_LpUartCfg();

    //发送数据
    LPUart_SendDataIt(M0P_LPUART1, 0x55);  
    
    while(1)
    {
        ;
    }
}

///<LPUART1 中断服务函数
void LpUart1_IRQHandler(void)
{
    if(LPUart_GetStatus(M0P_LPUART1, LPUartTC))
    {
        LPUart_ClrStatus(M0P_LPUART1, LPUartTC);   ///<清发送中断请求
        
        LPUart_DisableIrq(M0P_LPUART1,LPUartTxIrq);///<禁止发送中断
        LPUart_EnableIrq(M0P_LPUART1,LPUartRxIrq); ///<使能接收中断
    }
    
    if(LPUart_GetStatus(M0P_LPUART1, LPUartRC))    ///接收数据
    {
        LPUart_ClrStatus(M0P_LPUART1, LPUartRC);   ///<清接收中断请求
        u8RxData = LPUart_ReceiveData(M0P_LPUART1);///读取数据
        
        LPUart_DisableIrq(M0P_LPUART1,LPUartRxIrq);///<禁止接收中断
        LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq); ///<使能发送中断
        
        LPUart_SendDataIt(M0P_LPUART1, ~u8RxData); ///把接收数据取反，并发送       
    }
}

///< LPUART配置
void App_LpUartCfg(void)
{
    stc_lpuart_cfg_t  stcCfg;

    DDL_ZERO_STRUCT(stcCfg);
    
    ///<外设模块时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart1,TRUE);    
    
    ///<LPUART 初始化
    stcCfg.enStopBit = LPUart1bit;                   ///<1停止位    
    stcCfg.enMmdorCk = LPUartEven;                   ///<偶校验
    stcCfg.stcBaud.enSclkSel = LPUartMskPclk;        ///<传输时钟源
    stcCfg.stcBaud.u32Sclk = Sysctrl_GetPClkFreq();  ///<PCLK获取
    stcCfg.stcBaud.enSclkDiv = LPUartMsk4Or8Div;     ///<采样分频
    stcCfg.stcBaud.u32Baud = 9600;                   ///<波特率
    stcCfg.enRunMode = LPUartMskMode3;               ///<工作模式
    LPUart_Init(M0P_LPUART1, &stcCfg);
     
    ///<LPUART 中断使能
    LPUart_ClrStatus(M0P_LPUART1,LPUartRC);          ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART1,LPUartTC);          ///<清发送中断请求
    LPUart_DisableIrq(M0P_LPUART1,LPUartRxIrq);      ///<禁止接收中断
    LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq);       ///<使能发送中断
    EnableNvic(LPUART1_IRQn,IrqLevel3,TRUE);         ///<系统中断使能
}

///< 端口配置
void App_LpUartPortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    ///<TX
    stcGpioCfg.enDir =  GpioDirOut;
    stcGpioCfg.bOutputVal = TRUE;
    Gpio_Init(GpioPortA,GpioPin0,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf2); //配置PA00为LPUART1_TX
    
    //<RX
    stcGpioCfg.enDir =  GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin1,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf2); //配置PA01为LPUART1_RX
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


