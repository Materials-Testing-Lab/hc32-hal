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
 * @brief  Source file for VC example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "vc.h"
#include "gpio.h"
#include "adc.h"
#include "bt.h"
#include "flash.h"
#include "bgr.h"
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
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
//时钟初始化配置
void App_ClkDivInit(void);
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq);
static void App_GpioInit(void);
static void App_BTimM23Init(void);
static void App_VC0Init(void);

int main(void)
{
    ///< 系统时钟初始化
    App_ClkDivInit();
    App_SystemClkInit_XTH(SysctrlXthFreq24_32MHz);
    ///< GPIO初始化
    App_GpioInit();
    ///< Base Timer MODE23 初始化
    App_BTimM23Init();
    ///< VC0 初始化
    App_VC0Init();

    ///< VC开启
    Vc_Cmd(VcChannel0, TRUE);
    ///< BT启动运行
    Bt_M23_Run(TIM0);

    while (1)
    {
        ;
    }
}

//时钟初始化配置
void App_ClkDivInit(void)
{
    //时钟分频设置
    Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
}

///<请注意根据外部晶振配置宏——[SYSTEM_XTH]
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq)
{
    ///<======================== 切换至XTH32MHz ==============================
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    if(SysctrlXthFreq24_32MHz == enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle1);
    }

    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(enXthFreq);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);

    Sysctrl_SysClkSwitch(SysctrlClkXTH);

    if(SysctrlXthFreq24_32MHz != enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle0);
    }
}

static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;

    DDL_ZERO_STRUCT(GpioInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);   //GPIO 外设时钟使能

    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin6,&GpioInitStruct);
    Gpio_SetAfMode(GpioPortA,GpioPin6,GpioAf5);              //VC_CH0输出VC0_OUT

    GpioInitStruct.enDir = GpioDirIn;
    Gpio_SetAnalogMode(GpioPortC,GpioPin0);                  //VC_CH0的P端模拟输入

    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &GpioInitStruct);   //PD14设置为LED输出

    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin0, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);              //PA00设置为TIM0_CHA

    Gpio_Init(GpioPortA, GpioPin1, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);              //PA01设置为TIM0_CHB
}


void App_BTimM23Init(void)
{
    uint16_t                      u16ArrValue;
    uint16_t                      u16CompareAValue;
    uint16_t                      u16CompareBValue;
    uint16_t                      u16CntValue;
    uint8_t                       u8ValidPeriod;
    stc_bt_mode23_cfg_t        stcBtBaseCfg;
    stc_bt_m23_OCREF_Clr_cfg_t stcBtOCREFClrCfg;
    stc_bt_m23_compare_cfg_t   stcBtPortCmpCfg;

    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcBtOCREFClrCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能

    stcBtBaseCfg.enWorkMode    = BtWorkMode2;              //锯齿波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv1;               //PCLK
    stcBtBaseCfg.enCntDir      = BtCntUp;                  //向上计数，在三角波模式时只读
    stcBtBaseCfg.enPWMTypeSel  = BtIndependentPWM;         //独立输出PWM
    stcBtBaseCfg.enPWM2sSel    = BtSinglePointCmp;         //单点比较功能
    stcBtBaseCfg.bOneShot      = FALSE;                    //循环计数
    stcBtBaseCfg.bURSSel       = FALSE;                    //上下溢更新

    Bt_Mode23_Init(TIM0, &stcBtBaseCfg);                   //TIM0 的模式0功能初始化

    stcBtOCREFClrCfg.enOCRefClrSrcSel = BtOC_Ref_Clr;
    stcBtOCREFClrCfg.bVCClrEn = TRUE;
    Bt_M23_OCRefClr(TIM0,&stcBtOCREFClrCfg);

    u16ArrValue = 0x9000;
    Bt_M23_ARRSet(TIM0, u16ArrValue, TRUE);                //设置重载值,并使能缓存

    u16CompareAValue = 0x6000;
    Bt_M23_CCR_Set(TIM0, BtCCR0A, u16CompareAValue);       //设置比较值A
    u16CompareBValue = 0x3000;
    Bt_M23_CCR_Set(TIM0, BtCCR0B, u16CompareBValue);       //设置比较值B


    stcBtPortCmpCfg.enCH0ACmpCtrl   = BtPWMMode2;          //OCREFA输出控制OCMA:PWM模式2
    stcBtPortCmpCfg.enCH0APolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCh0ACmpBufEn   = TRUE;                //A通道缓存控制
    stcBtPortCmpCfg.enCh0ACmpIntSel = BtCmpIntNone;        //A通道比较控制:无

    stcBtPortCmpCfg.enCH0BCmpCtrl   = BtPWMMode2;          //OCREFB输出控制OCMB:PWM模式2
    stcBtPortCmpCfg.enCH0BPolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCH0BCmpBufEn   = TRUE;                //B通道缓存控制使能
    stcBtPortCmpCfg.enCH0BCmpIntSel = BtCmpIntNone;        //B通道比较控制:无

    Bt_M23_PortOutput_Cfg(TIM0, &stcBtPortCmpCfg);      //比较输出端口配置

    u8ValidPeriod = 0;                                     //事件更新周期设置，0表示锯齿波每个周期更新一次，每+1代表延迟1个周期
    Bt_M23_SetValidPeriod(TIM0,u8ValidPeriod);             //间隔周期设置

    u16CntValue = 0;
    Bt_M23_Cnt16Set(TIM0, u16CntValue);                    //设置计数初值

    Bt_M23_EnPWM_Output(TIM0, TRUE, FALSE);                //TIM0 端口输出使能
}

/**
 ******************************************************************************
 ** \brief  初始化VC0
 **
 ** \return 无
 ******************************************************************************/
void App_VC0Init(void)
{
    stc_vc_channel_cfg_t VcInitStruct;
    DDL_ZERO_STRUCT(VcInitStruct);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralVcLvd, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);

    Bgr_BgrEnable();                 //BGR必须使能

    VcInitStruct.enVcChannel = VcChannel0;
    VcInitStruct.enVcCmpDly  = VcDelay10mv;          //VC0迟滞电压约为10mV
    VcInitStruct.enVcBiasCurrent = VcBias10ua;       //VC0功耗为10uA
    VcInitStruct.enVcFilterTime  = VcFilter28us;     //VC输出滤波时间约为28us
    VcInitStruct.enVcInPin_P     = VcInPCh0;         //VC0_CH的P端连接PC00
    VcInitStruct.enVcInPin_N     = AiLdo;            //VC0_CH的N端连接内核1.5V
    VcInitStruct.enVcOutCfg      = VcOutTIM0RCLR;    //输出到BT REFCLR
    VcInitStruct.bFlten          = TRUE;             //使能滤波
    Vc_Init(&VcInitStruct);

    ///< VC0触发类型配置
    Vc_CfgItType(VcChannel0, VcIrqRise);          //配置VC0为上升沿触发类型
    Vc_ClearItStatus(Vc0_Intf);
    Vc_ItCfg(VcChannel0, TRUE);
}
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


