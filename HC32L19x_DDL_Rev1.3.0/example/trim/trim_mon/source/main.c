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
 * @brief  Source file for TRIM example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "trim.h"
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

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static volatile uint8_t u8TrimTestFlag   = 0;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_SysClkInit(void);
static void App_TrimCalInit(void);
static void App_UserKeyInit(void);

/*******************************************************************************
 * TRIM 中断服务程序
 ******************************************************************************/   
void ClkTrim_IRQHandler(void)
{ 
    if(Trim_GetIntFlag(TrimXTLFault))
    {
        Trim_ClearIntFlag(TrimXTLFault);
        Trim_Stop();
        ///< 监测到异常，并标记
        u8TrimTestFlag = 0xFFu;
    }

}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return int32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int32_t main(void)
{
    ///< 系统时钟初始化
    App_SysClkInit();
    ///< 按键功能初始化
    App_UserKeyInit();
    ///< Trim 校准功能初始化
    App_TrimCalInit();    
    
    ///< 等待USER KEY按下(低电平)后程序继续执行
    while(TRUE == Gpio_GetInputIO(STK_USER_PORT, STK_USER_PIN));
 
    ///< 开启时钟监测流程
    Trim_Run();
    
    while(1)
    {
        ///< 监测异常并结束
        if(0xFF == u8TrimTestFlag)
        {
            break;
        }
    }
    
    while(1);
}



static void App_SysClkInit(void)
{    
    ///< 使能被监测时钟，本样例使用XTL
    ///< XTL 配置及使能
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp3, SysctrlXtalDriver3);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE); 
}

static void App_UserKeyInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
    
    ///< GPIO IO USER KEY初始化
    Gpio_Init(STK_USER_PORT, STK_USER_PIN, &stcGpioCfg);
    
}

static void App_TrimCalInit(void)
{
    stc_trim_cfg_t stcCfg;

    //打开TRIM外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTrim, TRUE);
     
    ///< 时钟监测流程
    stcCfg.enMON     = TrimMonEnable;
    stcCfg.enREFCLK  = TrimRefMskIRC10K;    //参考时钟为内部10K
    stcCfg.enCALCLK  = TrimCalMskXTL;
    stcCfg.u32RefCon = 10000u;              //监测周期为1s
    stcCfg.u32CalCon = 100u;                //配置为默认值
    Trim_Init(&stcCfg);
    
    ///< 打开TRIM中断使能
    Trim_EnableIrq();
    ///< 使能并配置TRIM 系统中断
    EnableNvic(CLKTRIM_IRQn, IrqLevel3, TRUE);
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


