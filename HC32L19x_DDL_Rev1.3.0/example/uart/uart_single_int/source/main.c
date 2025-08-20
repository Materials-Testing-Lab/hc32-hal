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
 * @brief  Source file for UART example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "uart.h"
#include "gpio.h"
#include "sysctrl.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/
#define     T1_PORT                 (3)
#define     T1_PIN                  (3)

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
//UART0收发相关变量定义
volatile uint8_t uart0_u8TxData[2] = {0x11,0x22};   //UART0待发送数据
volatile uint8_t uart0_u8RxData[2] = {0x00};
volatile uint8_t uart0_u8TxCnt=0, uart0_u8RxCnt=0;
//UART1收发相关变量定义
volatile uint8_t uart1_u8TxData[2] = {0x33,0x44};   //UART1待发送数据
volatile uint8_t uart1_u8RxData[2] = {0x00};
volatile uint8_t uart1_u8TxCnt=0, uart1_u8RxCnt=0;

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_UartCfg(void);
void App_PortInit(void);


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
    //<串口端口初始化
    App_PortInit();    
    
    //串口配置，UART0:先发送，后接收;  UART1:先接收，后发送
    App_UartCfg();

    //启动UART0发送第一个数据，接下来的数据在中断中发送
    Uart_SendDataIt(M0P_UART0, uart0_u8TxData[0]); 
    
    while(1)
    {
        if(uart1_u8RxCnt>1)                                //如果UART1接收到两个字节
        {
            uart1_u8RxCnt = 0;
            Uart_SendDataIt(M0P_UART1, uart1_u8TxData[0]); //启动UART1发送第一个数据，接下来的数据在中断中发送 
        }
    }
}

//UART0中断服务函数
void Uart0_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART0, UartRC))
    {
        uart0_u8RxData[uart0_u8RxCnt] = Uart_ReceiveData(M0P_UART0);   //UART0接收数据
        uart0_u8RxCnt++; 
        if (uart0_u8RxCnt > 1)                                         //已接收两个字节
        {
            Uart_DisableIrq(M0P_UART0, UartRxIrq);                     //禁止UART0接收功能
        }
        Uart_ClrStatus(M0P_UART0, UartRC);                             //清除中断状态标志位
    }
    
    if(Uart_GetStatus(M0P_UART0, UartTC))
    {
        uart0_u8TxCnt++;                                               //主函数中已发送第一个字节，此处要计数器加一
        if (uart0_u8TxCnt > 1)                                         //如果已发送完两个字节
        {
            Uart_DisableIrq(M0P_UART0, UartTxIrq);                     //禁止UART0发送功能
            Uart_EnableIrq(M0P_UART0, UartRxIrq);                      //使能UART0接收功能
        }else
        {
            Uart_SendDataIt(M0P_UART0, uart0_u8TxData[uart0_u8TxCnt]); //UART0发送数据            
        }
        Uart_ClrStatus(M0P_UART0, UartTC);                             //清除中断状态标志位
    }

}

//UART1中断服务函数
void Uart1_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART1, UartRC))
    {
        Uart_ClrStatus(M0P_UART1, UartRC);
        uart1_u8RxData[uart1_u8RxCnt] = Uart_ReceiveData(M0P_UART1); //UART1接收数据
        uart1_u8RxCnt++; 
        if (uart1_u8RxCnt > 1)                                       //已接收两个字节
        {
            Uart_DisableIrq(M0P_UART1, UartRxIrq);                   //禁止UART1接收功能
            Uart_EnableIrq(M0P_UART1, UartTxIrq);                    //使能UART1发送功能
        }
    }
    
    if(Uart_GetStatus(M0P_UART1, UartTC))
    {
        Uart_ClrStatus(M0P_UART1, UartTC);
        uart1_u8TxCnt++;                                            //主函数中已发送第一个字节，计数器在发送数据之前加一
        if (uart1_u8TxCnt > 1)                                      //已发送完两个字节
        {
            Uart_DisableIrq(M0P_UART1, UartTxIrq);                  //禁止UART0发送功能
        }else
        {
            Uart_SendDataIt(M0P_UART1, uart1_u8TxData[uart1_u8TxCnt]);//UART1发送数据            
        }
    }

}

//GPIO端口配置函数
void App_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//使能GPIO模块时钟
    
    stcGpioCfg.enDir = GpioDirOut;                        //GPIO 输出
    stcGpioCfg.bOutputVal = TRUE;
    stcGpioCfg.enOD  = GpioOdEnable;                      //GPIO开漏输出使能
    stcGpioCfg.enPu  = GpioPuEnable;                      //必须设置成上拉，否则不能正常工作
    
    ///<UART0 TX
    Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf1);         //配置端口为URART0_TX
    
    ///<UART1 TX
    Gpio_Init(GpioPortA, GpioPin2, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin2, GpioAf1);         //配置端口为URART1_TX
}

//串口模块功能配置
void App_UartCfg(void)
{
    stc_uart_cfg_t    stcCfg;

    DDL_ZERO_STRUCT(stcCfg);
    
    ///< 开启外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);//使能uart0模块时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);//使能uart1模块时钟
    

    ///<UART Init
    stcCfg.enRunMode        = UartMskMode3;          //模式3
    stcCfg.enStopBit        = UartMsk1bit;           //1bit停止位
    stcCfg.enMmdorCk        = UartMskEven;           //偶检验
    stcCfg.stcBaud.u32Baud  = 9600;                  //波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       //通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); //获得外设时钟（PCLK）频率值
        
    Uart_Init(M0P_UART0, &stcCfg);                   ///<UART0串口初始化
    Uart_Init(M0P_UART1, &stcCfg);                   ///<UART1串口初始化
        
    Uart_HdModeEnable(M0P_UART0);                    //使能UART0单线半双工模式
    Uart_HdModeEnable(M0P_UART1);                    //使能UART1单线半双工模式
    
    ///<UART0中断使能
    Uart_ClrStatus(M0P_UART0,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART0,UartTC);                ///<清接收请求
    Uart_DisableIrq(M0P_UART0,UartRxIrq);            ///<禁用串口接收中断
    Uart_EnableIrq(M0P_UART0,UartTxIrq);             ///<使能串口发送中断    
    EnableNvic(UART0_2_IRQn, IrqLevel3, TRUE);       ///<系统中断使能
    
    ///<UART1中断使能
    Uart_ClrStatus(M0P_UART1,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART1,UartTC);                ///<清接收请求
    Uart_EnableIrq(M0P_UART1,UartRxIrq);             ///<使能串口接收中断
    Uart_DisableIrq(M0P_UART1,UartTxIrq);            ///<使能串口发送中断    
    EnableNvic(UART1_3_IRQn, IrqLevel3, TRUE);       ///<系统中断使能

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


