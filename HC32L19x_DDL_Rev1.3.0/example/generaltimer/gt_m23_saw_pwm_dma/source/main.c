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
 * @brief  Source file for General Timer example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "bt.h"
#include "gpio.h"
#include "flash.h"
#include "adc.h"
#include "dmac.h"
/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
volatile uint32_t u32Tim0PWMDuty[2];
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/*******************************************************************************
 * TIM0中断服务函数
 ******************************************************************************/
void Tim0_IRQHandler(void)
{
    static uint8_t i;
    
    //Timer0 模式23 更新中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        if(0 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, TRUE);  //LED 引脚输出高电平
            
            u32Tim0PWMDuty[0] = 0x3000;
            u32Tim0PWMDuty[1] = 0x6000;
            
            i++;
        }
        else if(1 == i)
        {
            Gpio_WriteOutputIO(STK_LED_PORT, STK_LED_PIN, FALSE);  //LED 引脚输出低电平
            
            u32Tim0PWMDuty[0] = 0x6000;
            u32Tim0PWMDuty[1] = 0x3000;
            
            i = 0;
        }
        
        Bt_ClearIntFlag(TIM0,BtUevIrq);  //清中断标志
    }
}

//时钟配置初始化
void App_ClockCfg(void)
{
    en_flash_waitcycle_t      enWaitCycle;
    stc_sysctrl_pll_cfg_t     stcPLLCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    
    enWaitCycle = FlashWaitCycle1;
    Flash_WaitCycle(enWaitCycle);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    Sysctrl_SysClkSwitch(SysctrlClkPLL);  ///< 时钟切换
}

//端口配置初始化
void App_Timer0PortCfg(void)
{
    stc_gpio_cfg_t             stcTIM0Port;
    stc_gpio_cfg_t             stcLEDPort;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM0Port);
    DDL_ZERO_STRUCT(stcLEDPort);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    
    stcLEDPort.enDir  = GpioDirOut;
    Gpio_Init(STK_LED_PORT, STK_LED_PIN, &stcLEDPort);     //PD14设置为LED输出
    
    stcTIM0Port.enDir  = GpioDirOut;
    
    Gpio_Init(GpioPortA, GpioPin0, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf7);            //PA00设置为TIM0_CHA
    
    Gpio_Init(GpioPortA, GpioPin1, &stcTIM0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf3);            //PA01设置为TIM0_CHB
}

//Timer0 配置
void App_Timer0Cfg(uint16_t u16Period, uint16_t u16CHxACompare, uint16_t u16CHxBCompare)
{
    uint16_t                   u16CntValue;
    uint8_t                    u8ValidPeriod;
    stc_bt_mode23_cfg_t        stcBtBaseCfg;
    stc_bt_m23_compare_cfg_t   stcBtPortCmpCfg;
    stc_bt_m23_adc_trig_cfg_t  stcAosTrigCfg;
    stc_bt_m23_trig_dma_cfg_t  stcTrigDmaCfg;
    
    //结构体初始化清零
    DDL_ZERO_STRUCT(stcBtBaseCfg);
    DDL_ZERO_STRUCT(stcBtPortCmpCfg);
    DDL_ZERO_STRUCT(stcAosTrigCfg);
    DDL_ZERO_STRUCT(stcTrigDmaCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE);   //Base Timer外设时钟使能
        
    stcBtBaseCfg.enWorkMode    = BtWorkMode2;              //锯齿波模式
    stcBtBaseCfg.enCT          = BtTimer;                  //定时器功能，计数时钟为内部PCLK
    stcBtBaseCfg.enPRS         = BtPCLKDiv1;               //PCLK
    stcBtBaseCfg.enCntDir      = BtCntUp;                  //向上计数，在三角波模式时只读
    stcBtBaseCfg.enPWMTypeSel  = BtIndependentPWM;         //独立输出PWM
    stcBtBaseCfg.enPWM2sSel    = BtSinglePointCmp;         //单点比较功能
    stcBtBaseCfg.bOneShot      = FALSE;                    //循环计数
    stcBtBaseCfg.bURSSel       = FALSE;                    //上下溢更新
    Bt_Mode23_Init(TIM0, &stcBtBaseCfg);                   //TIM0 的模式23功能初始化
    
    Bt_M23_ARRSet(TIM0, u16Period, TRUE);                  //设置重载值,并使能缓存
    
    Bt_M23_CCR_Set(TIM0, BtCCR0A, u16CHxACompare);         //设置比较值A
    u32Tim0PWMDuty[0] = u16CHxACompare;
    
    Bt_M23_CCR_Set(TIM0, BtCCR0B, u16CHxBCompare);         //设置比较值B
    u32Tim0PWMDuty[1] = u16CHxBCompare;
    
    
    stcBtPortCmpCfg.enCH0ACmpCtrl   = BtPWMMode2;          //OCREFA输出控制OCMA:PWM模式2
    stcBtPortCmpCfg.enCH0APolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCh0ACmpBufEn   = TRUE;                //A通道缓存控制
    stcBtPortCmpCfg.enCh0ACmpIntSel = BtCmpIntNone;        //A通道比较控制:无
    
    stcBtPortCmpCfg.enCH0BCmpCtrl   = BtPWMMode2;          //OCREFB输出控制OCMB:PWM模式2
    stcBtPortCmpCfg.enCH0BPolarity  = BtPortPositive;      //正常输出
    stcBtPortCmpCfg.bCH0BCmpBufEn   = TRUE;                //B通道缓存控制使能
    stcBtPortCmpCfg.enCH0BCmpIntSel = BtCmpIntNone;        //B通道比较控制:无
    
    Bt_M23_PortOutput_Cfg(TIM0, &stcBtPortCmpCfg);         //比较输出端口配置
    
    stcAosTrigCfg.bEnTrigADC = TRUE;
    stcAosTrigCfg.bEnUevTrigADC = TRUE;
    Bt_M23_TrigADC_Cfg(TIM0, &stcAosTrigCfg);   //AOS输出信号选择
    
    stcTrigDmaCfg.bUevTrigDMA = TRUE;
    Bt_M23_EnDMA(TIM0, &stcTrigDmaCfg);         //UEV更新触发DMA使能
        
    u8ValidPeriod = 0;                                     //事件更新周期设置，0表示锯齿波每个周期更新一次，每+1代表延迟1个周期
    Bt_M23_SetValidPeriod(TIM0,u8ValidPeriod);             //间隔周期设置
        
    u16CntValue = 0;
    Bt_M23_Cnt16Set(TIM0, u16CntValue);                    //设置计数初值
    
    Bt_ClearAllIntFlag(TIM0);                              //清中断标志
    Bt_Mode23_EnableIrq(TIM0,BtUevIrq);                    //使能TIM0 UEV更新中断
    EnableNvic(TIM0_IRQn, IrqLevel0, TRUE);                //TIM0中断使能
}

// DMA通道配置， Timer0 uev 触发 DMA传输
void App_DmaCfg(void)
{
    stc_dma_cfg_t           stcDmaCfg;    
    
    DDL_ZERO_STRUCT(stcDmaCfg);                     //结构体变量 初始值清零

    // 使能 DMA时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);
    
    stcDmaCfg.enMode =  DmaMskBlock;                            //选择块传输
    stcDmaCfg.u16BlockSize = 0x02u;                             //块传输个数
    stcDmaCfg.u16TransferCnt = 0x01u;                           //Block模式，一次传输数据大小为 2,传输1次
    stcDmaCfg.enTransferWidth = DmaMsk32Bit;                    //传输数据的宽度，此处选择字(32Bit)宽度
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;                 //源地址自增
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrInc;                 //目的地址自增
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable;  //使能重新加载传输目的地址
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;   //使能重新加载传输源地址
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;      //使能重新加载BC/TC值
    stcDmaCfg.u32SrcAddress = (uint32_t) &(u32Tim0PWMDuty[0]);  //指定传输源地址
    stcDmaCfg.u32DstAddress = (uint32_t)&M0P_TIM0_MODE23->CCR0A;  //指定传输目的地址
    stcDmaCfg.enTransferMode = DmaMskContinuousTransfer;        //DMAC 在传输完成时不清除 CONFA:ENS 位。这个功能允许连续传输而不需要 CPU 干预。
    stcDmaCfg.enRequestNum = DmaTIM0BTrig;                      //设置为Timer0 UEV 触发
    
    Dma_InitChannel(DmaCh0,&stcDmaCfg);       //初始化DMA通道0
  
    //使能DMA，使能DMA0
    Dma_Enable();
    Dma_EnableChannel(DmaCh0);
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
    App_ClockCfg();                       //时钟初始化
    
    App_Timer0Cfg(48000, 24000, 24000);   //Timer0 配置:周期 48000(锯齿波: 1K); 通道A比较值24000; 通道B比较值24000
    
    App_Timer0PortCfg();                  //Timer3 Port端口配置
    
    Bt_M23_EnPWM_Output(TIM0, TRUE, FALSE);    //TIM0 端口输出使能
    
    App_DmaCfg();                         //DMA传输初始化
    
    Bt_M23_Run(TIM0);                          //TIM0 运行

    while (1);

}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


