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
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
uint8_t u8TxData[2] = {0x00,0x55};
uint8_t u8RxData = 00;
uint8_t u8TxCnt=0,u8RxCnt=0;

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
void App_LpUartDeepSleepCfg(void);
void App_LpUartPortCfg(void);
void App_LpUartClkCfg(void);
void App_LpUartCfg(void);

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 **
 ******************************************************************************/
int32_t main(void)
{    
    ///< LPUART时钟配置，传输时钟选择RCL
    App_LpUartClkCfg();       
    
    ///< LPUART IO口相关配置，需按键按下，才能继续运行
    App_LpUartPortCfg();  
    
    ///< LPUART 功能配置
    App_LpUartCfg();
    
    ///< 发送数据0x55到上位机
    LPUart_SendDataIt(M0P_LPUART1, 0x55);

    ///< 进入深度休眠模式
    Lpm_GotoDeepSleep(TRUE);
    
    while(1)
    {
        ;
    }
}

///< LPUART时钟配置
void App_LpUartClkCfg(void)
{
    Sysctrl_SetRCLTrim(SysctrlRclFreq38400);        ///< 配置RCL时钟为38.4kHz
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);   ///< 使能RCL时钟
}

///< LPUART配置
void App_LpUartCfg(void)
{
    stc_lpuart_cfg_t  stcCfg;

    DDL_ZERO_STRUCT(stcCfg);                        ///< 结构体变量初始值置零
    
    ///<外设模块时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart1,TRUE);  

    ///<LPUART 初始化
    stcCfg.enStopBit = LPUart1bit;                   ///<1停止位    
    stcCfg.enMmdorCk = LPUartEven;                   ///<偶校验
    stcCfg.stcBaud.enSclkSel = LPUartMskRcl;         ///<传输时钟源
    stcCfg.stcBaud.u32Sclk = 38400;                  ///<RCL时钟频率 38400Hz
    stcCfg.stcBaud.enSclkDiv = LPUartMsk4Or8Div;     ///<采样分频
    stcCfg.stcBaud.u32Baud = 9600;                   ///<波特率
    stcCfg.enRunMode = LPUartMskMode3;               ///<工作模式
    LPUart_Init(M0P_LPUART1, &stcCfg);
     
    ///<LPUART 中断使能
    LPUart_ClrStatus(M0P_LPUART1,LPUartRC);         ///<清接收中断请求
    LPUart_ClrStatus(M0P_LPUART1,LPUartTC);         ///<清发送中断请求
    LPUart_DisableIrq(M0P_LPUART1,LPUartRxIrq);     ///<禁止接收中断
    LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq);      ///<使能发送中断
    EnableNvic(LPUART1_IRQn,IrqLevel3,TRUE);        ///<系统中断使能
}


///< 端口配置
void App_LpUartPortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    DDL_ZERO_STRUCT(stcGpioCfg);                            ///< 结构体变量初始值置零
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 使能GPIO时钟
    
    ///<深度休眠模式外部端口配置(STK)
    App_LpUartDeepSleepCfg();    
    
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

///<深度休眠模式外部端口配置(STK)
void App_LpUartDeepSleepCfg(void)
{    
    //初始化IO配置(follow STK)
    M0P_GPIO->PAADS = 0;        ///< 不用的引脚全部配置成 数字端口
    M0P_GPIO->PBADS = 0;
    M0P_GPIO->PCADS = 0;
    M0P_GPIO->PDADS = 0;
    M0P_GPIO->PEADS = 0;
    M0P_GPIO->PFADS = 0;
    
    M0P_GPIO->PADIR = 0XFFFF;   ///< 不用的引脚全部配置成 输入IO
    M0P_GPIO->PBDIR = 0XFFFF;
    M0P_GPIO->PCDIR = 0XFFFF;
    M0P_GPIO->PDDIR = 0XFFFF;
    M0P_GPIO->PEDIR = 0XFFFF;
    M0P_GPIO->PFDIR = 0XFFFF;
    
    M0P_GPIO->PAPD = 0xFF7F;    ///< 不用的引脚全部配置成 使能下拉
    M0P_GPIO->PBPD = 0xFFFF;
    M0P_GPIO->PCPD = 0xFFFF;
    M0P_GPIO->PDPD = 0xFFFF;
    M0P_GPIO->PEPD = 0xFFFF;
    M0P_GPIO->PFPD = 0xFFFF;
    
    //配置GPIO PA07为输入(USER KEY)防止进入低功耗无法调试
    M0P_GPIO->PAADS_f.PA07   = 0;
    M0P_GPIO->PA07_SEL       = 0;
    M0P_GPIO->PADIR_f.PA07   = 1;
    M0P_GPIO->PAPU_f.PA07    = 1;
    while(M0P_GPIO->PAIN_f.PA07);  ///< 等待按键按下，只有按键按下后才能继续运行到低功耗模式

    //swd接口配置为gpio，注意接下来不能进行debug调试
    M0P_SYSCTRL->SYSCTRL2 = 0X5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0XA5A5;
    M0P_SYSCTRL->SYSCTRL1_f.SWD_USE_IO = 1;

}

///<LPUART1 中断服务函数
void LpUart1_IRQHandler(void)
{
    if(LPUart_GetStatus(M0P_LPUART1, LPUartTC))       ///接收数据中断
    {
        LPUart_ClrStatus(M0P_LPUART1, LPUartTC);      ///<清发送中断请求
        
        LPUart_DisableIrq(M0P_LPUART1,LPUartTxIrq);   ///<禁止发送中断
        LPUart_EnableIrq(M0P_LPUART1,LPUartRxIrq);    ///<使能接收中断
    }
    
    if(LPUart_GetStatus(M0P_LPUART1, LPUartRC))       ///接收数据中断
    {
        LPUart_ClrStatus(M0P_LPUART1, LPUartRC);      ///<清接收中断请求
        u8RxData = LPUart_ReceiveData(M0P_LPUART1);   ///读取数据
        
        LPUart_DisableIrq(M0P_LPUART1,LPUartRxIrq);   ///<禁止接收中断
        LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq);    ///<使能发送中断
        
        LPUart_SendDataIt(M0P_LPUART1, ~u8RxData);    ///把接收数据取反，并发送         
    }

}




/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


