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

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
volatile uint16_t u16Adt4CntState;
volatile uint16_t u16Adt5CntState;
volatile uint16_t u16Adt6CntState;

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

///< AdvTimer4/5/6 软件同步启动、停止、清零
void App_AdvTimer_All_SwSync_Start(void)
{
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_sw_sync_t stcAdtSwSync;
    stc_adt_sw_sync_t stcAdtSwSState;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtSwSync);
    DDL_ZERO_STRUCT(stcAdtSwSState);   
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);             //端口外设时钟使能

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能

    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeB;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div2;
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM5, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                    //ADT载波、计数模式、时钟配置
    
    stcAdtSwSync.bAdTim4 = TRUE;
    stcAdtSwSync.bAdTim5 = TRUE;
    stcAdtSwSync.bAdTim6 = TRUE;
    // start
    Adt_SwSyncStart(&stcAdtSwSync);                          //软件同步开始
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
    
    delay1ms(1000);
    
    // stop
    Adt_SwSyncStop(&stcAdtSwSync);                           //软件同步停止
    //delay1ms(20);
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
    
    // clear
    Adt_SwSyncClear(&stcAdtSwSync);                          //软件同步清零

    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
}
 
///< AdvTimer5/6 软件同步启动、停止、清零
void App_AdvTimer_Tim5Tim6_SwSync_Start(void)
{
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_sw_sync_t stcAdtSwSync;
    stc_adt_sw_sync_t stcAdtSwSState;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtSwSync);
    DDL_ZERO_STRUCT(stcAdtSwSState);   
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);             //端口外设时钟使能

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能

    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeB;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div2;
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM5, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                    //ADT载波、计数模式、时钟配置
    
    stcAdtSwSync.bAdTim4 = FALSE;
    stcAdtSwSync.bAdTim5 = TRUE;
    stcAdtSwSync.bAdTim6 = TRUE;
    // start
    Adt_SwSyncStart(&stcAdtSwSync);                          //软件同步开始
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
    
    delay1ms(1000);
    
    // stop
    Adt_SwSyncStop(&stcAdtSwSync);                           //软件同步停止
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
    
    // clear
    Adt_SwSyncClear(&stcAdtSwSync);                          //软件同步清零
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
}

///< AdvTimer4/6 软件同步启动、停止、清零
void App_AdvTimer_Tim4Tim6_SwSync_Start(void)
{
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_sw_sync_t stcAdtSwSync;
    stc_adt_sw_sync_t stcAdtSwSState;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtSwSync);
    DDL_ZERO_STRUCT(stcAdtSwSState);   
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);             //端口外设时钟使能

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能

    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeB;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div2;
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM5, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                    //ADT载波、计数模式、时钟配置
    
    stcAdtSwSync.bAdTim4 = TRUE;
    stcAdtSwSync.bAdTim5 = FALSE;
    stcAdtSwSync.bAdTim6 = TRUE;
    //start
    Adt_SwSyncStart(&stcAdtSwSync);                          //软件同步开始
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
    
    delay1ms(1000);
    // stop
    Adt_SwSyncStop(&stcAdtSwSync);                          //软件同步停止
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);             //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                    //读取软件同步状态
    
    // clear
    Adt_SwSyncClear(&stcAdtSwSync);                          //软件同步清零
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
}

///< AdvTimer4/5 软件同步启动、停止、清零
void App_AdvTimer_Tim4Tim5_SwSync_Start(void)
{
    stc_adt_basecnt_cfg_t stcAdtBaseCntCfg;
    stc_adt_sw_sync_t stcAdtSwSync;
    stc_adt_sw_sync_t stcAdtSwSState;
    
    DDL_ZERO_STRUCT(stcAdtBaseCntCfg);
    DDL_ZERO_STRUCT(stcAdtSwSync);
    DDL_ZERO_STRUCT(stcAdtSwSState);   
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);             //端口外设时钟使能

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdvTim, TRUE);  //ADT外设时钟使能

    stcAdtBaseCntCfg.enCntMode = AdtTriangleModeB;
    stcAdtBaseCntCfg.enCntDir = AdtCntUp;
    stcAdtBaseCntCfg.enCntClkDiv = AdtClkPClk0Div2;
    Adt_Init(M0P_ADTIM4, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM5, &stcAdtBaseCntCfg);
    Adt_Init(M0P_ADTIM6, &stcAdtBaseCntCfg);                    //ADT载波、计数模式、时钟配置
    
    stcAdtSwSync.bAdTim4 = TRUE;
    stcAdtSwSync.bAdTim5 = TRUE;
    stcAdtSwSync.bAdTim6 = FALSE;
    // start
    Adt_SwSyncStart(&stcAdtSwSync);                          //软件同步开始
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
    delay1ms(1000);
    // stop
    Adt_SwSyncStop(&stcAdtSwSync);                          //软件同步停止
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                    //读取软件同步状态
    
    // clear
    Adt_SwSyncClear(&stcAdtSwSync);                          //软件同步清零
    
    u16Adt4CntState = Adt_GetCount(M0P_ADTIM4);              //读取计数值
    u16Adt5CntState = Adt_GetCount(M0P_ADTIM5);
    u16Adt6CntState = Adt_GetCount(M0P_ADTIM6);
    Adt_GetSwSyncState(&stcAdtSwSState);                     //读取软件同步状态
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

   
    //AdvTimer4/5/6 软件同步启动、停止、清零
    App_AdvTimer_All_SwSync_Start();
    
    //AdvTimer5/6 软件同步启动、停止、清零
    //App_AdvTimer_Tim5Tim6_SwSync_Start();
    
    //AdvTimer4/6 软件同步启动、停止、清零
    //App_AdvTimer_Tim4Tim6_SwSync_Start();
    
    //AdvTimer4/5 软件同步启动、停止、清零
    //App_AdvTimer_Tim4Tim5_SwSync_Start();
    
    while(1);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


