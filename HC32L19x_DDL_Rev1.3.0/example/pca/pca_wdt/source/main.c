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
#define FEEDDOG 

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


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void App_GpioInit(void); 
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
    ///< 端口初始化
    App_GpioInit();
    ///< PCA初始化
    App_PcaInit();
    ///< PCA 运行
    Pca_StartPca(TRUE);
    
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);
    
#ifdef FEEDDOG    
    while(1)
    {

        if(Pca_GetCnt()>=1000)
        {
            Pca_SetCnt(0);
        }
        
    }    
#else
    while(1);
#endif
}


/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    stc_gpio_cfg_t    GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);
    Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);
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
    PcaInitStruct.pca_epwm   = PcaEpwmDisable;        //禁止16bitPWM输出
    PcaInitStruct.pca_ccap   = 4000;
    Pca_M4Init(&PcaInitStruct);   

    Pca_Set4Wdte(TRUE); ///< 看门狗使能
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


