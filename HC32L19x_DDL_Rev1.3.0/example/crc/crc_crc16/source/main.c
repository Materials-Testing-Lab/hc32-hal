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
 * @brief  Source file for CRC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "crc.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
 
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
//待编码数据
uint8_t  au8CrcTestData[8] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0};
//待编码数据数组长度
uint32_t u32TestDataLen    = 8;
uint16_t u16RefCrc16       = 0x5234;
uint16_t u16ErrRefCrc16    = 0xEEEE;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

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
    uint16_t    u16CrcResult = 0;
    volatile uint8_t u8TestFlag = 0;
    volatile en_result_t enResult = Error;
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralCrc, TRUE);       ///< 开启CRC外设时钟模块
    
    /* 8位位宽下输入数据测试示例 */
    u16CrcResult =  CRC16_Get8(au8CrcTestData, u32TestDataLen);  ///< 产生CRC16编码
    if(u16RefCrc16 == u16CrcResult)                              ///< 判断CRC16编码值是否正确
    {
        u8TestFlag = Ok;
    }    
    ///< 检验待校验数据与校验值是否匹配，如果匹配返回Ok,否则返回Error
    enResult = CRC16_Check8(au8CrcTestData, u32TestDataLen, u16CrcResult);  
 
    
    /* 16位位宽下输入数据测试示例 */
    u16CrcResult =  CRC16_Get16((uint16_t *)au8CrcTestData, u32TestDataLen/2);  ///< 产生CRC16编码
    if(u16RefCrc16 == u16CrcResult)                                             ///< 判断CRC16编码值是否正确
    {
        u8TestFlag = Ok;
    }    
    ///< 检验待校验数据与校验值是否匹配，如果匹配返回Ok,否则返回Error
    enResult = CRC16_Check16((uint16_t *)au8CrcTestData, u32TestDataLen/2, u16CrcResult);

    /* 32位位宽下输入数据测试示例 */
    u16CrcResult =  CRC16_Get32((uint32_t *)au8CrcTestData, u32TestDataLen/4);  ///< 产生CRC16编码
    if(u16RefCrc16 == u16CrcResult)                                             ///< 判断CRC16编码值是否正确
    {
        u8TestFlag = Ok;
    }    
    ///< 检验待校验数据与校验值是否匹配，如果匹配返回Ok,否则返回Error
    enResult = CRC16_Check32((uint32_t *)au8CrcTestData, u32TestDataLen/4, u16CrcResult);    
    while (1)
    {
        ;
    }
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


