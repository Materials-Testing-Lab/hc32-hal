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
 * @brief  Source file for I2C example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "i2c.h"
#include "gpio.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define I2C_SLAVEADDR 0xA0
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
static uint8_t u8Senddata[10] = {0x12,0x34,0x77,0x66,0x55,0x44,0x33,0x22,0x11,0x99};
uint8_t u8Recdata[10]={0x00};

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
en_result_t I2C_MasterWriteData(M0P_I2C_TypeDef* I2CX,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);
en_result_t I2C_MasterReadData(M0P_I2C_TypeDef* I2CX,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);
void App_I2cCfg(void);
void App_PortCfg(void);
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
    ///< I2C 模块配置
    App_I2cCfg();

    ///< IO端口配置
    App_PortCfg();
        
    ///< 向I2C总线发起开始信号
    I2C_SetFunc(M0P_I2C1,I2cStart_En);       

    ///< eeprom写数据
    I2C_MasterWriteData(M0P_I2C1,0x00,u8Senddata,10);  
    delay1ms(100);
    
    ///< eeprom读数据
    I2C_MasterReadData(M0P_I2C1,0x00,u8Recdata,10);    
    
    while(1)
    {
        ;
    }  
}


///< IO端口配置
void App_PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);                 ///< 初始化结构体变量的值为0
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 开启GPIO时钟门控 
        
    stcGpioCfg.enDir = GpioDirOut;               ///< 端口方向配置->输出      
    stcGpioCfg.enOD = GpioOdEnable;              ///< 端口开漏输出配置->开漏输出使能      
    stcGpioCfg.enPu = GpioPuEnable;              ///< 端口上拉配置->使能
    stcGpioCfg.enPd = GpioPdDisable;             ///< 端口下拉配置->禁止
    stcGpioCfg.bOutputVal = TRUE;
   
    Gpio_Init(GpioPortA, GpioPin11,&stcGpioCfg); ///< 初始化PA11
    Gpio_Init(GpioPortA, GpioPin12,&stcGpioCfg); ///< 初始化PA12
    
    Gpio_SetAfMode(GpioPortA, GpioPin11,GpioAf3);///< 配置PA11为SCL
    Gpio_SetAfMode(GpioPortA, GpioPin12,GpioAf3);///< 配置PA12为SDA    
}

///< I2C 模块配置
void App_I2cCfg(void)
{
    stc_i2c_cfg_t stcI2cCfg;
    
    DDL_ZERO_STRUCT(stcI2cCfg);                             ///< 初始化结构体变量的值为0
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c1,TRUE);  ///< 开启I2C1时钟门控   
        
    stcI2cCfg.u32Pclk = Sysctrl_GetPClkFreq();              ///< 获取PCLK时钟       
    stcI2cCfg.u32Baud = 100000;                             ///< 波特率100kHz
    stcI2cCfg.enMode  = I2cMasterMode;                      ///< I2C主机模式
    stcI2cCfg.u8SlaveAddr = 0x55;                           ///< 从地址，主模式无效
    stcI2cCfg.bGc = FALSE;                                  ///< 广播地址应答使能关闭，主模式无效
    I2C_Init(M0P_I2C1, &stcI2cCfg);                         ///< 模块初始化    
}

/**
 ******************************************************************************
 ** \brief  主机接收函数
 **
 ** \param u8Addr从机内存地址，pu8Data读数据存放缓存，u32Len读数据长度
 **
 ** \retval 读数据是否成功
 **
 ******************************************************************************/
 en_result_t I2C_MasterReadData(M0P_I2C_TypeDef* I2CX,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    
    I2C_SetFunc(I2CX,I2cStart_En);
    
    while(1)
    {
        while(0 == I2C_GetIrq(I2CX))
        {}
        u8State = I2C_GetState(I2CX);
        switch(u8State)
        {
            case 0x08:                                 ///< 已发送起始条件，将发送SLA+W
                I2C_ClearFunc(I2CX,I2cStart_En);
                I2C_WriteByte(I2CX,I2C_SLAVEADDR);
                break;
            case 0x18:                                 ///< 已发送SLA+W,并接收到ACK
                I2C_WriteByte(I2CX,u8Addr);            ///< 发送从机内存地址
                break;
            case 0x28:                                 ///< 已发送数据，接收到ACK, 此处是已发送从机内存地址u8Addr并接收到ACK
                I2C_SetFunc(I2CX,I2cStart_En);         ///< 发送重复起始条件
                break;
            case 0x10:                                 ///< 已发送重复起始条件
                I2C_ClearFunc(I2CX,I2cStart_En);
                I2C_WriteByte(I2CX,I2C_SLAVEADDR|0x01);///< 发送SLA+R，开始从从机读取数据
                break;
            case 0x40:                                 ///< 已发送SLA+R，并接收到ACK
                if(u32Len>1)
                {
                    I2C_SetFunc(I2CX,I2cAck_En);       ///< 使能主机应答功能
                }
                break;
            case 0x50:                                 ///< 已接收数据字节，并已返回ACK信号
                pu8Data[u8i++] = I2C_ReadByte(I2CX);
                if(u8i==u32Len-1)
                {
                    I2C_ClearFunc(I2CX,I2cAck_En);     ///< 已接收到倒数第二个字节，关闭ACK应答功能
                }
                break;
            case 0x58:                                 ///< 已接收到最后一个数据，NACK已返回
                pu8Data[u8i++] = I2C_ReadByte(I2CX);
                I2C_SetFunc(I2CX,I2cStop_En);          ///< 发送停止条件
                break;
            case 0x38:                                 ///< 在发送地址或数据时，仲裁丢失
                I2C_SetFunc(I2CX,I2cStart_En);         ///< 当总线空闲时发起起始条件
                break;
            case 0x48:                                 ///< 发送SLA+R后，收到一个NACK
                I2C_SetFunc(I2CX,I2cStop_En);          ///< 发送停止条件
                I2C_SetFunc(I2CX,I2cStart_En);         ///< 发送起始条件
                break;
            default:
                I2C_SetFunc(I2CX,I2cStart_En);         ///< 其他错误状态，重新发送起始条件
                break;
        }
        I2C_ClearIrq(I2CX);                            ///< 清除中断状态标志位
        if(u8i==u32Len)                                ///< 数据全部读取完成，跳出while循环
        {
                break;
        }
    }
    enRet = Ok;
    return enRet;
}
/**
 ******************************************************************************
 ** \brief  主机发送函数
 **
 ** \param u8Addr从机内存地址，pu8Data写数据，u32Len写数据长度
 **
 ** \retval 写数据是否成功
 **
 ******************************************************************************/
en_result_t I2C_MasterWriteData(M0P_I2C_TypeDef* I2CX,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State, u8Flag=FALSE;
    I2C_SetFunc(I2CX,I2cStart_En);
    while(1)
    {
        while(0 == I2C_GetIrq(I2CX))
        {;}
        u8State = I2C_GetState(I2CX);
        switch(u8State)
        {
            case 0x08:                               ///< 已发送起始条件
                I2C_ClearFunc(I2CX,I2cStart_En);
                I2C_WriteByte(I2CX,I2C_SLAVEADDR);   ///< 从设备地址发送
                break;
            case 0x18:                               ///< 已发送SLA+W，并接收到ACK
                I2C_WriteByte(I2CX,u8Addr);          ///< 从设备内存地址发送
                break;
            case 0x28:                               ///< 上一次发送数据后接收到ACK
                if (u8i < u32Len)
                {
                    I2C_WriteByte(I2CX,pu8Data[u8i++]);  ///< 继续发送数据
                }else
                {
                    I2C_SetFunc(I2CX,I2cStop_En);     ///< 出停止条件
                    u8Flag = TRUE;
                }
                break;
            case 0x20:                               ///< 上一次发送SLA+W后，收到NACK
            case 0x38:                               ///< 上一次在SLA+读或写时丢失仲裁
                I2C_SetFunc(I2CX,I2cStart_En);       ///< 当I2C总线空闲时发送起始条件
                break;
            case 0x30:                               ///< 已发送I2Cx_DATA中的数据，收到NACK，将传输一个STOP条件
                I2C_SetFunc(I2CX,I2cStop_En);        ///< 发送停止条件
                break;
            default:
                break;
        }
        
        I2C_ClearIrq(I2CX);                          ///< 清除中断状态标志位
        
        if(u8Flag == TRUE)  ///< 数据发送完成
        {   
            break;
        }
    }
    enRet = Ok;
    return enRet;
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


