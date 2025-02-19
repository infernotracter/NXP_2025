/******************** LQ_K60_函数库 v1.0 ********************
 * 文件名           ：PLL.c
 * 功能             ：设置工作时钟模式，锁相环，即超频
 * 备注             ：官方例程上修改
 * 日期             ：2015-10-16
 * 实验平台         ：龙丘 k60 vg板 
 * 开发环境         ：IAR 7.3
 * 作者             ：龙丘技术 006
 * 淘宝店           ：https://longqiu.taobao.com
 * 龙丘智能车讨论群 ：202949437
*************************************************************/
#include "include.h"
#include "MK60_PLL.h"

uint8_t core_clk;//单位MHZ
uint8_t bus_clk;//单位MHZ

/*************************************************************************
*                        龙丘智能科技有限公司
*
*  函数名称：pll_init
*  功能说明：时钟初始化，用于设定频率。
*  参数说明：PLL_?
*  函数返回：无
*  修改时间：2016-8-20
*  备    注：内核时钟（系统时钟）=外部时钟（50M晶振频率）/ (pll_prdiv+1)*(pll_vdiv+16);
             MCG=PLL, core = MCG, bus = MCG/5, FlexBus = MCG/3, Flash clock= MCG/8
*  例    子：pll_init(PLL180);
*************************************************************************/
void PLL_Init(clk_option opt)
{
     char pll_prdiv;
     char pll_vdiv;

     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                 | SIM_SCGC5_PORTB_MASK
                 | SIM_SCGC5_PORTC_MASK
                 | SIM_SCGC5_PORTD_MASK
                 | SIM_SCGC5_PORTE_MASK );

    u32 temp_reg;

    if(opt!= PLLUNULL )
    {
        //设置PLL时钟
        switch(opt)
        {
        case PLL80:
            pll_prdiv       = 4;
            pll_vdiv        = 0;
            break;
        case PLL90:
            pll_prdiv       = 4;
            pll_vdiv        = 2;
            break;
        case PLL100:
            pll_prdiv       = 4;
            pll_vdiv        = 4;
            break;
        case PLL120:
            pll_prdiv       = 4;
            pll_vdiv        = 8;
            break;
        case PLL130:
            pll_prdiv       = 4;
            pll_vdiv        = 10;
            break;
        case PLL140:
            pll_prdiv       = 4;
            pll_vdiv        = 12;
            break;
        case PLL150:
            pll_prdiv       = 4;
            pll_vdiv        = 14;
            break;
        case PLL160:
            pll_prdiv       = 4;
            pll_vdiv        = 16;
            break;
        case PLL170:
            pll_prdiv       = 4;
            pll_vdiv        = 18;
            break;
        case PLL180:
            pll_prdiv       = 4;
            pll_vdiv        = 20;
            break;
        case PLL200:
            pll_prdiv       = 4;
            pll_vdiv        = 24;
            break;
       case PLL225:
            pll_prdiv       = 4;
            pll_vdiv        = 28;
            break;
        case PLL220:
            pll_prdiv       = 4;
            pll_vdiv        = 29;
            break;
        case PLL230:
            pll_prdiv       = 4;    //稳定
            pll_vdiv        = 30;
            break;
       case PLL235:
            pll_prdiv       = 4;    //不稳定
            pll_vdiv        = 31;
            break;
       case PLL237_5:               //很不稳定
            pll_prdiv       = 3;
            pll_vdiv        = 22;
            break;
        default:               break;//(初始化未成功，系统默认系统时钟为180M)

        }
    }
    MCG_C1 = MCG_C1_CLKS(2) ;//选择外部时钟


    MCG_C5 = MCG_C5_PRDIV(pll_prdiv);//晶振为50M，分频结果范围要在8M~16M 此时为 50/(prdiv+1)


   temp_reg = FMC_PFAPR;

    //通过M&PFD置位M0PFD来禁止预取功能
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;
    ///设置系统分频器
    //MCG=PLL, core = MCG,  
    SIM_CLKDIV1 =  SIM_CLKDIV1_OUTDIV1(0)    //core = MCG
                 | SIM_CLKDIV1_OUTDIV2(1)    //bus = MCG/2,
                 | SIM_CLKDIV1_OUTDIV3(2)    //FlexBus = MCG/(2+1)
                 | SIM_CLKDIV1_OUTDIV4(7);   //Flash clock= MCG/8

    //从新存FMC_PFAPR的原始值
    FMC_PFAPR = temp_reg;

    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(pll_vdiv);//PLL =  50M/(prdiv+1) * (pll_vdiv+16)

    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set
    while (!(MCG_S & MCG_S_LOCK0_MASK)){}; // Wait for LOCK bit to set


    MCG_C1=0x00;

    //等待时钟状态位更新
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};


    core_clk = 50 * ( pll_vdiv+16 )/2/(pll_prdiv+1);
    bus_clk =core_clk/2;   

    
 /*
        //设置跟踪时钟为内核时钟
    SIM_SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;
    //在PTA6引脚上使能TRACE_CLKOU功能
    PORTA_PCR6 = ( PORT_PCR_MUX(0x7));
    //使能FlexBus模块时钟
    SIM_SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
    //在PTA6引脚上使能FB_CLKOUT功能
    PORTC_PCR3 = ( PORT_PCR_MUX(0x5));
    */
}