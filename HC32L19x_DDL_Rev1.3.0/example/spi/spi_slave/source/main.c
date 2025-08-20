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
 * @brief  Source file for SPI example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "spi.h"
#include "gpio.h"
#include "reset.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint8_t rx_buf[10]={0};
const uint8_t tx_buf[10]={1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
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
static void App_GpioInit(void);
static void App_SPIInit(void);


/**
******************************************************************************
    ** \brief  主函数
    ** 
    ** @param  无
    ** \retval 无
    **
******************************************************************************/ 
int32_t main(void)
{
    uint16_t tmp;
    tmp = 0;

    App_GpioInit();
    App_SPIInit();
    
    while(1 == Gpio_GetInputIO(GpioPortB, GpioPin12));//等待片选信号生效
    ///< 从机接收主机数据
    Spi_ReceiveBuf(M0P_SPI1, rx_buf, 10);   

    App_SPIInit();
    Spi_Slave_DummyWriteData(M0P_SPI1, tx_buf[0]);      ///<如果在接收一帧数据中第一字节数据，则需要在接收前预置数据
    while(1 == Gpio_GetInputIO(GpioPortB, GpioPin12));  //等待片选信号生效    
    ///< 从机向主机发送数据
    Spi_SendBuf(M0P_SPI1, (uint8_t*)(&tx_buf[1]), 9);
    
    for(tmp = 0; tmp<10; tmp++)                              //判断发送的数据与接收的数据是否相等
    {
        if(rx_buf[tmp] == tx_buf[tmp])             
            continue;
        else
            break;
    }
    
    if(tmp == 10)                                    //如果接收到的数据与发送的数据相等则点亮板上蓝色LED
        Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);    
    
    while(1)
    {
    }
}


/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    //SPI1引脚配置：从机
    GpioInitStruct.enDir = GpioDirIn;
    Gpio_Init(STK_SPI1_CS_PORT, STK_SPI1_CS_PIN, &GpioInitStruct);
    Gpio_SfSsnCfg(GpioSpi1,GpioSfSsnExtClkPB12);         //配置SPI1_CS
                                                               
    Gpio_Init(STK_SPI1_SCK_PORT, STK_SPI1_SCK_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(STK_SPI1_SCK_PORT, STK_SPI1_SCK_PIN, GpioAf1);        //配置SPI1_SCK
                                                               
    Gpio_Init(STK_SPI1_MOSI_PORT, STK_SPI1_MOSI_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(STK_SPI1_MOSI_PORT, STK_SPI1_MOSI_PIN, GpioAf1);        //配置SPI1_MOSI    
                                                               
    GpioInitStruct.enDir = GpioDirOut;                         
    Gpio_Init(STK_SPI1_MISO_PORT, STK_SPI1_MISO_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(STK_SPI1_MISO_PORT, STK_SPI1_MISO_PIN, GpioAf1);        //配置SPI1_MISO    

    //PD14:板上蓝色LED
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);     //输出高，熄灭LED        
}

/**
 ******************************************************************************
 ** \brief  配置SPI
 **
 ** \return 无
 ******************************************************************************/
static void App_SPIInit(void)
{
    stc_spi_cfg_t  SpiInitStruct;   

    ///< 打开外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi1, TRUE);

    ///<复位模块
    Reset_RstPeripheral0(ResetMskSpi1);
    
    //SPI1模块配置：从机
    SpiInitStruct.enSpiMode = SpiMskSlave;   //配置位主机模式
    SpiInitStruct.enPclkDiv = SpiClkMskDiv2;  //波特率：PCLK/2
    SpiInitStruct.enCPHA    = SpiMskCphafirst;//第一边沿采样
    SpiInitStruct.enCPOL    = SpiMskcpollow;  //极性为低
    Spi_Init(M0P_SPI1, &SpiInitStruct);    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


