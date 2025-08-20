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
 * @brief  Source file for PCA example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "pca.h"
#include "lpm.h"
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
 
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void App_PcaInit(void);

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
    App_PcaInit();
    
    Pca_StartPca(TRUE);
    
    while (1)
    {
        ;
    }
}


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 /**
******************************************************************************
    ** \brief  RTC中断入口函数
    ** 
  ** @param  无
    ** \retval 无
    **
******************************************************************************/ 
void Pca_IRQHandler(void)
{
    if(Pca_GetItStatus(PcaCcf2) != FALSE)
    {
        Pca_ClrItStatus(PcaCcf2);
    }
    else if(Pca_GetItStatus(PcaCf) != FALSE)
    {
        Pca_ClrItStatus(PcaCf);    
    }
}

/**
 ******************************************************************************
 ** \brief  配置PCA
 **
 ** \return 无
 ******************************************************************************/
static void App_PcaInit(void)
{
    stc_pcacfg_t  PcaInitStruct;
    
    //使能PCA外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
    
    PcaInitStruct.pca_clksrc = PcaPclkdiv32;
    PcaInitStruct.pca_cidl   = FALSE;
    PcaInitStruct.pca_ecom   = PcaEcomEnable;        //允许比较器功能
    PcaInitStruct.pca_capp   = PcaCappDisable;        //禁止上升沿捕获
    PcaInitStruct.pca_capn   = PcaCapnDisable;        //禁止下降沿捕获
    PcaInitStruct.pca_mat    = PcaMatEnable;        //禁止匹配功能
    PcaInitStruct.pca_tog    = PcaTogDisable;        //禁止翻转控制功能
    PcaInitStruct.pca_pwm    = PcaPwm8bitDisable;    //使能PWM控制输出
    PcaInitStruct.pca_epwm   = PcaEpwmEnable;        //禁止16bitPWM输出
    PcaInitStruct.pca_carr   = 0x1000;
    PcaInitStruct.pca_ccap   = 0x0120;
    Pca_M2Init(&PcaInitStruct);  

    Pca_ClrItStatus(PcaCcf2);
    EnableNvic(PCA_IRQn, IrqLevel3, TRUE);
    Pca_ConfModulexIt(PcaModule2, TRUE);
    Pca_ConfPcaIt(TRUE);    
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


